/*
 * Audio_driver.cpp
 *
 * Created: 15/07/2022 12:48:11
 *  Author: GuavTek
 */

#include "audio_driver.h"
#include "i2s.h"
#include "hardware/dma.h"
#include "usb_descriptors.h"
#include "board_m2idi_usb.h"

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

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request){

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ){
		if (request->bRequest == AUDIO_CS_REQ_CUR){
			audio_control_cur_4_t curf = { tu_htole32(current_sample_rate) };
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
		} else if (request->bRequest == AUDIO_CS_REQ_RANGE){
			audio_control_range_4_n_t(N_SAMPLE_RATES) rangef = {
				.wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
			};
			for(uint8_t i = 0; i < N_SAMPLE_RATES; i++){
				rangef.subrange[i].bMin = sample_rates[i];
				rangef.subrange[i].bMax = sample_rates[i];
				rangef.subrange[i].bRes = 0;
			}

			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
		}
	}
	else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
	request->bRequest == AUDIO_CS_REQ_CUR){
		audio_control_cur_1_t cur_valid = { .bCur = 1 };
		return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
	}
	return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf){
	(void)rhport;

	if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ){
		current_sample_rate = ((audio_control_cur_4_t const *)buf)->bCur;

		i2s_set_samplerate(current_sample_rate);

		// TODO?
		if (current_resolution_in == 24){
			byte_per_frame = current_sample_rate * 2 * 32 / 8000;
		} else {
			byte_per_frame = current_sample_rate * 2 * current_resolution_in / 8000;
		}

		return true;
	}
	return false;
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request){
	if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR){
		audio_control_cur_1_t mute1 = { .bCur = mute[request->bChannelNumber] };
		return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
	} else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME){
		if (request->bRequest == AUDIO_CS_REQ_RANGE){
			audio_control_range_2_n_t(1) range_vol = {
				tu_htole16(1),		// wNumSubRanges
				{ tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }		// subrange
			};
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
		} else if (request->bRequest == AUDIO_CS_REQ_CUR){
			audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
		}
	}

	return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf){
	(void)rhport;

	if (request->bControlSelector == AUDIO_FU_CTRL_MUTE){
		mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;

		return true;
	} else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME){
		volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;

		return true;
	}
	return false;
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request){
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_CLOCK){
		return tud_audio_clock_get_request(rhport, request);
	}
	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT){
		return tud_audio_feature_unit_get_request(rhport, request);
	}
	return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf){
	audio_control_request_t const *request = (audio_control_request_t const *)p_request;

	if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT){
		return tud_audio_feature_unit_set_request(rhport, request, buf);
	}
	if (request->bEntityID == UAC2_ENTITY_CLOCK){
		return tud_audio_clock_set_request(rhport, request, buf);
	}

	return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request){
	(void)rhport;

	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0) {
		//current_resolution_out = 0;
		spk_active = false;
		spk_data_new = 0;
	}

	if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt == 0){
		//current_resolution_in = 0;
		mic_active = false;
	}

	if (!spk_active && !mic_active){
		// Disable I2S
		i2s_stop();
	}

	return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request){
	(void)rhport;
	uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
	uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

	if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0) {
		current_resolution_out = resolutions_per_format[alt-1];
		//i2s_set_output_wordsize(current_resolution_out);	// TODO

		// Clear buffer when streaming format is changed
		spk_data_new = 0;
		//i2s_tx_descriptor_a->btctrl.valid = 0;
		//i2s_tx_descriptor_b->btctrl.valid = 0;
		if (!spk_active && !mic_active){
			i2s_start();
		}
		spk_active = true;
	}

	if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt != 0) {
		current_resolution_in = resolutions_per_format[alt-1];
		//i2s_set_input_wordsize(current_resolution_in);	// TODO
		// TODO?
		if (current_resolution_in == 24){
			byte_per_frame = current_sample_rate * 2 * 32 / 8000;
		} else {
			byte_per_frame = current_sample_rate * 2 * current_resolution_in / 8000;
		}

		//i2s_rx_descriptor_a->btctrl.valid = 0;
		//i2s_rx_descriptor_b->btctrl.valid = 0;
		if (!spk_active && !mic_active){
			i2s_start();
		}
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

	//i2s_adjust_freq(pid_step(delta));	// TODO

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

		//i2s_adjust_freq(pid_step(delta));	// TODO
	}

	return true;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

bool i2s_buff_swapped;

static void i2s_callback(){
	i2s_buff_swapped = 1;
}

void audio_init(){
	i2s_config conf;
	conf.bit_depth = 32;
	conf.clock_pin_base = I2S_BCK; //	I2S_FS is BCK + 1
	conf.din_pin = I2S_DI;
	conf.dout_pin = I2S_DO;
	conf.mck_pin = I2S_MCK;
	conf.fs = current_sample_rate;
	conf.mck_enable = true;
	conf.mck_mult = 256;
    i2s_program_init(pio0, &conf, i2s_callback);
}

// TODO: samplerate mismatch
void audio_task(void){
	if (i2s_buff_swapped){
		i2s_buff_swapped = 0;
		uint32_t temp[STEREO_BUFFER_SIZE];
		// TODO: data format
		uint8_t wordsize = current_resolution_out == 16 ? 2 : 4;
		if (spk_data_new >= STEREO_BUFFER_SIZE*wordsize){
			uint8_t new_data_size;
			new_data_size = tud_audio_read(temp, STEREO_BUFFER_SIZE*wordsize);
			// TODO: 16-bit data must be split into 32-bit words
			i2s_write_buff(temp);
		} else {
			// TODO Write dummy data?
		}
		if (mic_active){
			wordsize = current_resolution_in == 16 ? 2 : 4;
			i2s_read_buff(temp);
			// TODO: 16-bit data must be compressed from 32-bit words
			tud_audio_write(temp, STEREO_BUFFER_SIZE*wordsize);
		}
	}//*/
}
