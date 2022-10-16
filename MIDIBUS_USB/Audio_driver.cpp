/*
 * Audio_driver.cpp
 *
 * Created: 15/07/2022 12:48:11
 *  Author: GuavTek
 */ 

#include "Audio_driver.h"

bool mic_active = false;
bool spk_active = false;

extern uint32_t blinkTime;
extern uint32_t blinkTime2;
extern volatile int64_t dropped_bytes;

// Supported sample rates
const uint32_t sample_rates[] = {44100, 48000, 88200, 96000};
uint32_t current_sample_rate  = 44100;
#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

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

// Buffer for microphone data
const uint16_t mic_buf_size = 64;
int32_t mic_buf[mic_buf_size];
// easier to read while debugging 16-bit mode
extern int16_t mic_buf16[mic_buf_size*2] __attribute__ ((alias ("mic_buf")));
uint32_t* const mic_buf_lo = (uint32_t*) &mic_buf[0];
uint32_t* const mic_buf_hi = (uint32_t*) &mic_buf[mic_buf_size/2];

// Buffer for speaker data
const uint16_t spk_buf_size = 256;
int32_t spk_buf[spk_buf_size];
// easier to read while debugging 16-bit mode
extern int16_t spk_buf16[spk_buf_size*2] __attribute__ ((alias ("spk_buf")));

uint32_t* const spk_buf_lo = (uint32_t*) &spk_buf[0];
uint32_t* const spk_buf_hi = (uint32_t*) &spk_buf[spk_buf_size/2];

// Speaker data size received in the last frame
int16_t spk_data_new;

// Mic data waiting to be sent
int16_t mic_data_pend;

// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution_out;
uint8_t current_resolution_in;

const struct DMA_Channel_config dma_i2s_tx_conf = {
	.int_enable_complete = 0,
	.int_enable_suspend = 0,
	.int_enable_error = 0,
	.trigger_src = I2S_DMAC_ID_TX_1,
	.trigger_action = DMAC_CHCTRLB_TRIGACT_BEAT_Val,
	.arbitration_lvl = 2
};

const struct DMA_Channel_config dma_i2s_rx_conf = {
	.int_enable_complete = 0,
	.int_enable_suspend = 0,
	.int_enable_error = 0,
	.trigger_src = I2S_DMAC_ID_RX_0,
	.trigger_action = DMAC_CHCTRLB_TRIGACT_BEAT_Val,
	.arbitration_lvl = 2
};



// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
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
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
	(void)rhport;

	TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
	TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
	{
		TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

		current_sample_rate = ((audio_control_cur_4_t const *)buf)->bCur;
		
		i2s_set_freq(current_sample_rate);		
		
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
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
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
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
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
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
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
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
	return tud_audio_feature_unit_set_request(rhport, request, buf);
	if (request->bEntityID == UAC2_ENTITY_CLOCK)
	return tud_audio_clock_set_request(rhport, request, buf);
	TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
	request->bEntityID, request->bControlSelector, request->bRequest);

	return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void)rhport;

	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0) {
		//current_resolution_out = 0;
		blinkTime = 100;
		// Detach DMA
		dma_suspend(1);
		spk_active = false;
		// Clear DMA transactions?
	}
	
	if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt == 0){
		//current_resolution_in = 0;
		blinkTime2 = 100;
		// Detach DMA
		dma_suspend(0);
		mic_active = false;
		// Clear DMA transactions?
	}
	
	return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
	(void)rhport;
	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));
		
	TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0) {
		current_resolution_out = resolutions_per_format[alt-1];
		blinkTime = 40;
		i2s_set_output_wordsize(current_resolution_out);
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
		i2s_set_input_wordsize(current_resolution_in);
		// Attach DMA
		//dma_resume(0);
		
		mic_data_pend = 0;
		i2s_rx_descriptor_a->btctrl.valid = 0;
		i2s_rx_descriptor_b->btctrl.valid = 0;
		mic_active = true;
	}

	return true;
}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting){
	(void)rhport;
	(void)func_id;
	(void)ep_out;
	(void)cur_alt_setting;

	const uint32_t midPoint = (CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2) / spk_buf_size - 1;
	
	i2s_adjust_freq((spk_data_new /(spk_buf_size))-midPoint);
	
	spk_data_new += n_bytes_received;
	
	return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting){
	(void)rhport;
	(void)func_id;
	(void)ep_in;
	(void)cur_alt_setting;
	
	mic_data_pend -= n_bytes_copied;
	
	const uint32_t midPoint = (CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 2) / mic_buf_size - 1;

	if (!spk_active){
		i2s_adjust_freq(midPoint-(mic_data_pend /(mic_buf_size)));
	}
		
	return true;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

uint32_t* const i2s_tx_reg = (uint32_t*) &I2S->DATA[1].reg;
uint32_t* const i2s_rx_reg = (uint32_t*) &I2S->DATA[0].reg;

void audio_dma_init(){
	DMA_BTCTRL_t tempCtrl;
	tempCtrl.word = 0;
	tempCtrl.evosel = DMAC_BTCTRL_EVOSEL_BEAT_Val;
	tempCtrl.blockact = DMAC_BTCTRL_BLOCKACT_NOACT_Val;
	tempCtrl.beatsize = DMAC_BTCTRL_BEATSIZE_HWORD_Val;
	tempCtrl.stepsel = DMAC_BTCTRL_STEPSEL_SRC_Val;
	tempCtrl.dstinc = 1;
	tempCtrl.srcinc = 0;
	
	// RX
	dma_set_descriptor(i2s_rx_descriptor_a, mic_buf_size, i2s_rx_reg, mic_buf_lo, i2s_rx_descriptor_b, tempCtrl);
	dma_set_descriptor(i2s_rx_descriptor_b, mic_buf_size, i2s_rx_reg, mic_buf_hi, i2s_rx_descriptor_a, tempCtrl);
		
	tempCtrl.stepsel = DMAC_BTCTRL_STEPSEL_DST_Val;
	tempCtrl.srcinc = 1;
	tempCtrl.dstinc = 0;
	
	// TX
	dma_set_descriptor(i2s_tx_descriptor_a, spk_buf_size, spk_buf_lo, i2s_tx_reg, i2s_tx_descriptor_b, tempCtrl);
	dma_set_descriptor(i2s_tx_descriptor_b, spk_buf_size, spk_buf_hi, i2s_tx_reg, i2s_tx_descriptor_a, tempCtrl);
	
	dma_attach(0, dma_i2s_rx_conf);
	dma_suspend(0);
	dma_attach(1, dma_i2s_tx_conf);
	dma_suspend(1);
}

void audio_task(void)
{
	
	// Is DMAC processing descriptor?
	if (i2s_rx_descriptor_wb->next_descriptor == i2s_rx_descriptor_b){
		i2s_rx_descriptor_a->btctrl.valid = 0;
	} else if (i2s_rx_descriptor_wb->next_descriptor == i2s_rx_descriptor_a){
		i2s_rx_descriptor_b->btctrl.valid = 0;
	}
	
	if (i2s_tx_descriptor_wb->next_descriptor == i2s_tx_descriptor_b){
		i2s_tx_descriptor_a->btctrl.valid = 0;
	} else if (i2s_tx_descriptor_wb->next_descriptor == i2s_tx_descriptor_a){
		i2s_tx_descriptor_b->btctrl.valid = 0;
	}
	
	// Speaker data waiting?
	if (spk_data_new != 0){
		// Check if buffer is valid, and is not being processed
		if ( (!i2s_tx_descriptor_a->btctrl.valid) && (i2s_tx_descriptor_wb->next_descriptor != i2s_tx_descriptor_b) )	{
			uint8_t data_shift = (current_resolution_out + 8) >> 4;
			DMA_BTCTRL_t tempCtrl;
			tempCtrl.word = 0;
			tempCtrl.evosel = DMAC_BTCTRL_EVOSEL_BEAT_Val;
			tempCtrl.blockact = DMAC_BTCTRL_BLOCKACT_NOACT_Val;
			tempCtrl.stepsel = DMAC_BTCTRL_STEPSEL_DST_Val;
			tempCtrl.beatsize = data_shift;
			tempCtrl.srcinc = 1;
			tempCtrl.valid = 1;
	
			uint16_t new_data_size;
			new_data_size = tud_audio_read(spk_buf_lo, spk_buf_size * 2 );
			spk_data_new -= new_data_size;
			
 			uint16_t temp;
 			temp = new_data_size & (0xffff >> (16-data_shift-1));
			new_data_size >>= data_shift;
			new_data_size &= 0xfffe;
			
 			if (temp){
 				dropped_bytes += temp;
 			}
			
			if (new_data_size == 0){
				dropped_bytes += spk_data_new;
				spk_data_new = 0;
			} else {
				dma_set_descriptor(i2s_tx_descriptor_a, new_data_size, spk_buf_lo, i2s_tx_reg, i2s_tx_descriptor_b, tempCtrl);
			}
		} else if ( (!i2s_tx_descriptor_b->btctrl.valid) && (i2s_tx_descriptor_wb->next_descriptor != i2s_tx_descriptor_a) ){
			uint8_t data_shift = (current_resolution_out + 8) >> 4;
			DMA_BTCTRL_t tempCtrl;
			tempCtrl.word = 0;
			tempCtrl.evosel = DMAC_BTCTRL_EVOSEL_BEAT_Val;
			tempCtrl.blockact = DMAC_BTCTRL_BLOCKACT_NOACT_Val;
			tempCtrl.stepsel = DMAC_BTCTRL_STEPSEL_DST_Val;
			tempCtrl.beatsize = data_shift;
			tempCtrl.srcinc = 1;
			tempCtrl.valid = 1;
			
			uint16_t new_data_size;
			new_data_size = tud_audio_read(spk_buf_hi, spk_buf_size * 2 );
			spk_data_new -= new_data_size;
			
 			uint16_t temp;
 			temp = new_data_size & (0xffff >> (16-data_shift-1));
			new_data_size >>= data_shift;
			new_data_size &= 0xfffe;
			
 			if (temp){
 				dropped_bytes += temp;
 			}
			
			if (new_data_size == 0){
				dropped_bytes += spk_data_new;
				spk_data_new = 0;
			} else {
				dma_set_descriptor(i2s_tx_descriptor_b, new_data_size, spk_buf_hi, i2s_tx_reg, i2s_tx_descriptor_a, tempCtrl);
			}
		}
	}
	
	if (spk_active){
		// Restart DMA
		bool hasValid = i2s_tx_descriptor_b->btctrl.valid || i2s_tx_descriptor_a->btctrl.valid;
		bool fs_pin = PORT->Group[0].IN.reg & (1 << 11);
		static bool fs_prev;
		if(!DMAC->BUSYCH.bit.BUSYCH1 && (fs_prev != fs_pin) && (fs_pin == 1) && hasValid){
			DMAC->CHID.reg = 1;
			dma_resume(1);
			DMAC->CHINTENSET.bit.SUSP = 1;
		}
		fs_prev = fs_pin;
		
	}
	
	if (mic_active){
		if ( (!i2s_rx_descriptor_a->btctrl.valid) && (i2s_rx_descriptor_wb->next_descriptor != i2s_rx_descriptor_b) )	{
			uint8_t data_shift = (current_resolution_in + 8) >> 4;
			DMA_BTCTRL_t tempCtrl;
			tempCtrl.word = 0;
			tempCtrl.evosel = DMAC_BTCTRL_EVOSEL_BEAT_Val;
			tempCtrl.blockact = DMAC_BTCTRL_BLOCKACT_NOACT_Val;
			tempCtrl.beatsize = data_shift;
			tempCtrl.stepsel = DMAC_BTCTRL_STEPSEL_SRC_Val;
			tempCtrl.dstinc = 1;
			tempCtrl.valid = 1;
			
			uint16_t tempData;
			tempData = tud_audio_write(mic_buf_lo, mic_buf_size * 2 );
			mic_data_pend += tempData;

			//dma_set_descriptor(i2s_rx_descriptor_a, ((mic_buf_size*2) >> data_shift), i2s_rx_reg, mic_buf_lo, i2s_rx_descriptor_b, tempCtrl);
			dma_set_descriptor(i2s_rx_descriptor_a, ((mic_buf_size*2) >> data_shift), data_shift);
		} else if ( (!i2s_rx_descriptor_b->btctrl.valid) && (i2s_rx_descriptor_wb->next_descriptor != i2s_rx_descriptor_a) ){
			uint8_t data_shift = (current_resolution_in + 8) >> 4;
			DMA_BTCTRL_t tempCtrl;
			tempCtrl.word = 0;
			tempCtrl.evosel = DMAC_BTCTRL_EVOSEL_BEAT_Val;
			tempCtrl.blockact = DMAC_BTCTRL_BLOCKACT_NOACT_Val;
			tempCtrl.beatsize = data_shift;
			tempCtrl.stepsel = DMAC_BTCTRL_STEPSEL_SRC_Val;
			tempCtrl.dstinc = 1;
			tempCtrl.valid = 1;
			
			uint16_t tempData;
			tempData = tud_audio_write(mic_buf_hi, mic_buf_size * 2 );
			mic_data_pend += tempData;

			//dma_set_descriptor(i2s_rx_descriptor_b, ((mic_buf_size*2) >> data_shift), i2s_rx_reg, mic_buf_hi, i2s_rx_descriptor_a, tempCtrl);
			dma_set_descriptor(i2s_rx_descriptor_b, ((mic_buf_size*2) >> data_shift), data_shift);
		}
		
		bool fs_pin = PORT->Group[0].IN.reg & (1 << 11);
		static bool fs_prev;
		if (!DMAC->BUSYCH.bit.BUSYCH0 && (fs_prev != fs_pin) && (fs_pin == 1)){
			// Resume channel
			DMAC->CHID.reg = 0;
			dma_resume(0);
			DMAC->CHINTENSET.bit.SUSP = 1;
		}
		fs_prev = fs_pin;
		
	}
}
//*/
