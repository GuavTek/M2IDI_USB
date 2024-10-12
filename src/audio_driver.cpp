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
uint32_t real_sample_rate = 44100;
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

// TODO: audio volume for speaker is not implemented
// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];       // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1];    // +1 for master channel 0

// Buffer for speaker data
const uint16_t SPK_BUF_SIZE = 96*4;
uint32_t spk_buf[SPK_BUF_SIZE];
// easier to read while debugging 16-bit mode
extern int16_t spk_buf16[SPK_BUF_SIZE*2] __attribute__ ((alias ("spk_buf")));
uint16_t spk_rd_idx;
uint16_t spk_wr_idx;
uint16_t spk_loaded;

// Speaker data waiting in USB section of RAM
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
			// Host requested current sample rate
			audio_control_cur_4_t curf = { tu_htole32(current_sample_rate) };
			return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
		} else if (request->bRequest == AUDIO_CS_REQ_RANGE){
			// Host requested all available sample rates
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
	} else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID && request->bRequest == AUDIO_CS_REQ_CUR){
		// Host requested which sample rates are valid with the current config
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
		real_sample_rate = current_sample_rate;

		i2s_set_samplerate(current_sample_rate);

		// TODO?
		uint8_t curr_res;
		if (spk_active){
			curr_res = current_resolution_out;
		} else {
			curr_res = current_resolution_in;
		}
		const uint32_t rate2byte = (2 << 16) / 8000; // 16.16 fixed point for (2 / 8000)
		if (curr_res == 24){
			uint32_t temp = current_sample_rate * 32 * rate2byte;
			byte_per_frame = temp >> 16;
		} else {
			uint32_t temp = current_sample_rate * curr_res * rate2byte;
			byte_per_frame = temp >> 16;
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

		// Clear buffer when streaming format is changed
		spk_data_new = 0;
		const uint32_t rate2byte = (2 << 16) / 8000; // 16.16 fixed point for (2 / 8000)
		if (current_resolution_out == 24){
			uint32_t temp = current_sample_rate * 32 * rate2byte;
			byte_per_frame = temp >> 16;
		} else {
			uint32_t temp = current_sample_rate * current_resolution_out * rate2byte;
			byte_per_frame = temp >> 16;
		}
		if (!spk_active && !mic_active){
			i2s_start();
		}
		spk_active = true;
	}

	if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt != 0) {
		current_resolution_in = resolutions_per_format[alt-1];
		// TODO?
		if (!spk_active){
			const uint32_t rate2byte = (2 << 16) / 8000; // 16.16 fixed point for (2 / 8000)
			if (current_resolution_in == 24){
				uint32_t temp = current_sample_rate * 32 * rate2byte;
				byte_per_frame = temp >> 16;
			} else {
				uint32_t temp = current_sample_rate * current_resolution_in * rate2byte;
				byte_per_frame = temp >> 16;
			}
		}

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

	int32_t delta = pid_step(spk_loaded + spk_data_new - byte_per_frame);
	if (delta != 0){
		real_sample_rate += delta;
		//i2s_set_samplerate(real_sample_rate);
	}

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
		int32_t delta = pid_step(byte_per_frame - avg_bytes_copied);
		if (delta != 0){
			real_sample_rate += delta;
			//i2s_set_samplerate(real_sample_rate);
		}
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

void load_audio_buff(){
	// Pull data from USB memory before it gets overwritten
	if (spk_data_new != 0){
		uint16_t buf_avail = SPK_BUF_SIZE - spk_loaded;
		uint16_t buf_end = SPK_BUF_SIZE - spk_wr_idx;
		if (buf_avail == 0){
			return;
		}
		if (buf_avail > buf_end){
			buf_avail = buf_end;
		}
		uint16_t loaded;
		if (current_resolution_out == 16) {
			uint16_t sample16_buff[SPK_BUF_SIZE/2];
			loaded = tud_audio_read(sample16_buff, buf_avail*2);
			spk_data_new -= loaded;
			loaded >>= 1;
			for (uint16_t i = 0; i < loaded; i++){
				spk_buf[spk_wr_idx] = sample16_buff[i];
				spk_buf[spk_wr_idx] <<= 16;
				spk_wr_idx++;
			}
		} else {
			loaded = tud_audio_read(&spk_buf[spk_wr_idx], buf_avail*4);
			spk_data_new -= loaded;
			loaded >>= 2;
			spk_wr_idx += loaded;
		}
		spk_loaded += loaded;
		if (loaded == 0){
			// We have lost data at some point
			spk_data_new = 0;
		}
		if (spk_wr_idx >= SPK_BUF_SIZE) {
			spk_wr_idx = 0;
		}
	}
}

// Last sample which was output to avoid pop if we run out of samples
uint32_t last_sampleL = 0x8000'0000;
uint32_t last_sampleR = 0x8000'0000;

void audio_task(void){
	load_audio_buff();
	if (i2s_buff_swapped){
		i2s_buff_swapped = 0;
		uint32_t temp[STEREO_BUFFER_SIZE];
		if (spk_loaded >= STEREO_BUFFER_SIZE){
			uint16_t load_end = SPK_BUF_SIZE - spk_rd_idx;
			if (load_end > STEREO_BUFFER_SIZE){
				i2s_write_buff(&spk_buf[spk_rd_idx]);
				spk_rd_idx += STEREO_BUFFER_SIZE;
				if (spk_rd_idx >= SPK_BUF_SIZE){
					spk_rd_idx = 0;
				}
			} else {
				// Load from buffer
				uint8_t i;
				for (i = 0; i < load_end; i++){
					temp[i] = spk_buf[spk_rd_idx++];
				}
				spk_rd_idx = 0;
				for (; i < STEREO_BUFFER_SIZE; i++){
					temp[i] = spk_buf[spk_rd_idx++];
				}
				i2s_write_buff(temp);
			}
			last_sampleL = temp[STEREO_BUFFER_SIZE-2];
			last_sampleR = temp[STEREO_BUFFER_SIZE-1];
			spk_loaded -= STEREO_BUFFER_SIZE;
		} else {
			// Write dummy data
			for (uint8_t i = 0; i < STEREO_BUFFER_SIZE; i += 2){
				temp[i] = last_sampleL;
				temp[i+1] = last_sampleR;
			}
			i2s_write_buff(temp);
		}
		if (mic_active){	// TODO: buffer it better
			i2s_read_buff(temp);
			if (current_resolution_in == 16){
				uint16_t buff16[STEREO_BUFFER_SIZE];
				for(uint16_t i = 0; i < STEREO_BUFFER_SIZE; i++){
					temp[i] >>= 16;
					buff16[i] = temp[i];
				}
				tud_audio_write(buff16, STEREO_BUFFER_SIZE * 2);
			} else {
				tud_audio_write(temp, STEREO_BUFFER_SIZE * 4);
			}
		}
	}
}
