/*
 * Audio_driver.cpp
 *
 * Created: 15/07/2022 12:48:11
 *  Author: GuavTek
 */ 

#include "audio_driver.h"
#include "i2s.h"

/*
typedef struct i2s_config {
    uint32_t fs;
    uint32_t sck_mult;
    uint8_t  bit_depth;
    uint8_t  sck_pin;
    uint8_t  dout_pin;
    uint8_t  din_pin;
    uint8_t  clock_pin_base;
    bool     sck_enable;
} i2s_config;
typedef struct pio_i2s {
    PIO        pio;
    uint8_t    sm_mask;
    uint8_t    sm_sck;
    uint8_t    sm_dout;
    uint8_t    sm_din;
    uint       dma_ch_in_ctrl;
    uint       dma_ch_in_data;
    uint       dma_ch_out_ctrl;
    uint       dma_ch_out_data;
    int32_t*   in_ctrl_blocks[2];  // Control blocks MUST have 8-byte alignment.
    int32_t*   out_ctrl_blocks[2];
    int32_t    input_buffer[STEREO_BUFFER_SIZE * 2];
    int32_t    output_buffer[STEREO_BUFFER_SIZE * 2];
    i2s_config config;
} pio_i2s;
*/
static __attribute__((aligned(8))) pio_i2s i2s;

bool mic_active = false;
bool spk_active = false;

extern uint32_t blinkTime;
extern uint32_t blinkTime2;
extern volatile int64_t dropped_bytes;

// Supported sample rates
const uint32_t sample_rates[] = {44100, 48000, 88200, 96000};
uint32_t current_sample_rate  = 44100;
#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)
uint16_t byte_per_frame;

enum
{
	VOLUME_CTRL_0_DB = 0,
	VOLUME_CTRL_10_DB = 2560,
	VOLUME_CTRL_20_DB = 5120,
	VOLUME_CTRL_30_DB = 7680,
	VOLUME_CTRL_40_DB = 10240,
	VOLUME_CTRL_50_DB = 12800,
	VOLUME_CTRL_60_DB = 15360,
	VOLUME_CTRL_70_DB = 17920,
	VOLUME_CTRL_80_DB = 20480,
	VOLUME_CTRL_90_DB = 23040,
	VOLUME_CTRL_100_DB = 25600,
	VOLUME_CTRL_SILENCE = 0x8000,
};

// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];       // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];    // +1 for master channel 0

// TODO i2s library handles double buffering, do we need more buffering here?
// Buffer for microphone data
const uint16_t mic_buf_size = 32;
int32_t mic_buf[mic_buf_size];
// easier to read while debugging 16-bit mode
extern int16_t mic_buf16[mic_buf_size*2] __attribute__ ((alias ("mic_buf")));
uint32_t* const mic_buf_lo = (uint32_t*) &mic_buf[0];
uint32_t* const mic_buf_hi = (uint32_t*) &mic_buf[mic_buf_size/2];

// Buffer for speaker data
const uint16_t spk_buf_size = 32;
int32_t spk_buf[spk_buf_size];
// easier to read while debugging 16-bit mode
extern int16_t spk_buf16[spk_buf_size*2] __attribute__ ((alias ("spk_buf")));

uint32_t* const spk_buf_lo = (uint32_t*) &spk_buf[0];
uint32_t* const spk_buf_hi = (uint32_t*) &spk_buf[spk_buf_size/2];

// Speaker data size received in the last frame
int16_t spk_data_new;

// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution_out;
uint8_t current_resolution_in;

#if (CFG_TUD_AUDIO > 0)

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request){
	TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
	{
		if (request->bRequest == AUDIO_CS_REQ_CUR)
		{
			TU_LOG1("Clock get current freq %lu\r\n", current_sample_rate);

			audio_control_cur_4_t curf = { tu_htole32(current_sample_rate) };
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
		}
		else if (request->bRequest == AUDIO_CS_REQ_RANGE)
		{
			audio_control_range_4_n_t(N_SAMPLE_RATES) rangef =
			{
				.wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
			};
			TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
			for(uint8_t i = 0; i < N_SAMPLE_RATES; i++)
			{
				rangef.subrange[i].bMin = sample_rates[i];
				rangef.subrange[i].bMax = sample_rates[i];
				rangef.subrange[i].bRes = 0;
				TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
			}
			
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
		}
	}
	else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
	request->bRequest == AUDIO_CS_REQ_CUR)
	{
		audio_control_cur_1_t cur_valid = { .bCur = 1 };
		TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
		return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
	}
	TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n",
	request->bEntityID, request->bControlSelector, request->bRequest);
	return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf){
	(void)rhport;

	TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
	TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

		current_sample_rate = ((audio_control_cur_4_t const *)buf)->bCur;
		
		i2s_set_freq(current_sample_rate);	// TODO
		
		if (current_resolution_in == 24){
			byte_per_frame = current_sample_rate * 2 * 32 / 8000;
		} else {
			byte_per_frame = current_sample_rate * 2 * current_resolution_in / 8000;
		}
		
		TU_LOG1("Clock set current freq: %ld\r\n", current_sample_rate);

		return true;
	}
	else
	{
		TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
		request->bEntityID, request->bControlSelector, request->bRequest);
		return false;
	}
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request){
	TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

	if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
	{
		audio_control_cur_1_t mute1 = { .bCur = mute[request->bChannelNumber] };
		TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
		return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
	}
	else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
	{
		if (request->bRequest == AUDIO_CS_REQ_RANGE)
		{
			audio_control_range_2_n_t(1) range_vol = {
				tu_htole16(1),		// wNumSubRanges
				{ tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }		// subrange
			};
			TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
			range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
		}
		else if (request->bRequest == AUDIO_CS_REQ_CUR)
		{
			audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
			TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
		}
	}
	TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
	request->bEntityID, request->bControlSelector, request->bRequest);

	return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf){
	(void)rhport;

	TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
	TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

	if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

		mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;

		TU_LOG1("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

		return true;
	}
	else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

		volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;

		TU_LOG1("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

		return true;
	}
	else
	{
		TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
		request->bEntityID, request->bControlSelector, request->bRequest);
		return false;
	}
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request){
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_CLOCK)
	return tud_audio_clock_get_request(rhport, request);
	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
	return tud_audio_feature_unit_get_request(rhport, request);
	else
	{
		TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
		request->bEntityID, request->bControlSelector, request->bRequest);
	}
	return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf){
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
	return tud_audio_feature_unit_set_request(rhport, request, buf);
	if (request->bEntityID == UAC2_ENTITY_CLOCK)
	return tud_audio_clock_set_request(rhport, request, buf);
	TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
	request->bEntityID, request->bControlSelector, request->bRequest);

	return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request){
	(void)rhport;

	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0) {
		//current_resolution_out = 0;
		blinkTime = 100;
		// Detach DMA
		// TODO? Should dma stop?
		// dma_suspend(1);
		spk_active = false;
		spk_data_new = 0;
	}
	
	if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt == 0){
		//current_resolution_in = 0;
		blinkTime2 = 100;
		// Detach DMA
		// TODO? Should dma stop?
		//dma_suspend(0);
		mic_active = false;
	}
	
	return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request){
	(void)rhport;
	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));
		
	TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0) {
		current_resolution_out = resolutions_per_format[alt-1];
		blinkTime = 40;
		i2s_set_output_wordsize(current_resolution_out);	// TODO
		// Attach DMA
		//dma_resume(1);
		
		// Clear buffer when streaming format is changed
		spk_data_new = 0;
		i2s_tx_descriptor_a->btctrl.valid = 0;
		i2s_tx_descriptor_b->btctrl.valid = 0;
		spk_active = true;
	}
	
	if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt != 0) {
		current_resolution_in = resolutions_per_format[alt-1];
		blinkTime2 = 40;
		i2s_set_input_wordsize(current_resolution_in);	// TODO
		if (current_resolution_in == 24){
			byte_per_frame = current_sample_rate * 2 * 32 / 8000;
		} else {
			byte_per_frame = current_sample_rate * 2 * current_resolution_in / 8000;
		}
		// Attach DMA
		//dma_resume(0);
		
		//i2s_rx_descriptor_a->btctrl.valid = 0;
		//i2s_rx_descriptor_b->btctrl.valid = 0;
		mic_active = true;
	}

	return true;
}

const int32_t pid_Kp = 16;
const int32_t pid_Ki = 8;
const int32_t pid_Kd = 8;
const uint8_t pid_i_div = 1;

// Simple PID regulator
inline int32_t pid_step(int32_t delta) {
	static int32_t pid_integrate = 0;
	static int32_t delta_prev = 0;
	int32_t temp_pid;
	
	pid_integrate += delta * pid_Ki;
	int32_t pid_current = pid_Kp * delta + (pid_integrate >> pid_i_div);
	temp_pid = pid_current + pid_Kd * (delta - delta_prev);
	delta_prev = delta;
	return temp_pid;
}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting){
	(void)rhport;
	(void)func_id;
	(void)ep_out;
	(void)cur_alt_setting;
	
	const uint32_t midPoint = CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2;
	int32_t delta = spk_data_new - midPoint;
	
	i2s_adjust_freq(pid_step(delta));	// TODO
	
	spk_data_new += n_bytes_received;
	
	return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting){
	(void)rhport;
	(void)func_id;
	(void)ep_in;
	(void)cur_alt_setting;
	
	// Running average (current + last) >> 1;
	static int16_t avg_bytes_copied;
	avg_bytes_copied += n_bytes_copied;
	avg_bytes_copied >>= 1;
	
	if (!spk_active){
		int32_t delta = byte_per_frame - avg_bytes_copied;
				
		i2s_adjust_freq(pid_step(delta));	// TODO
	}
		
	return true;
}
#endif // (CFG_TUD_AUDIO > 0)

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

//uint32_t* const i2s_tx_reg = (uint32_t*) &I2S->DATA[1].reg;
//uint32_t* const i2s_rx_reg = (uint32_t*) &I2S->DATA[0].reg;

void i2s_callback(){

}

void audio_init(){
	i2s.config.bit_depth = 32;
	i2s.config.clock_pin_base = I2S_BCK;
	i2s.config.din_pin = I2S_DI;
	i2s.config.dout_pin = I2S_DO;
	i2s.config.sck_pin = I2S_MCK;
	i2s.config.fs = current_sample_rate;
	i2s.config.sck_enable = true;
	i2s.config.sck_mult = 256;
	// TODO: init audio?
    //i2s_program_start_synched(pio0, &i2s.config, i2s_callback, &i2s);
}

void audio_task(void){
	
	// Speaker data waiting?
	if (spk_data_new != 0){
		// Check if buffer is valid, and is not being processed
		
	}
	
	if (spk_active){
        // Load tx buffer
	}
	
	if (mic_active){
		// Read rx buffer if it is valid
    }
}
//*/

