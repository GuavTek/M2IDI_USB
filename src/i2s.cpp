
#include "i2s.h"
#include <math.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "i2s.pio.h"

pio_i2s i2s;
static __attribute__((aligned(8))) int32_t* ctrl_in[2];
static __attribute__((aligned(8))) int32_t* ctrl_out[2];
void dma_cb();
void (*i2s_handler)(void);
bool buff_num;

static float pio_div(float freq, uint16_t* div, uint8_t* frac) {
    float clk   = (float)clock_get_hz(clk_sys);
    float ratio = clk / freq;
    float d;
    float f = modff(ratio, &d);
    *div    = (uint16_t)d;
    *frac   = (uint8_t)(f * 256);

    // Use rounded values to calculate actual frequency
    float result = clk / ((float)*div + ((float)*frac / 256));

    return result;
}

static void set_clocks(const i2s_config* config) {
    uint16_t mck_d;
    uint8_t  mck_f;
    uint16_t bck_d;
    uint8_t  bck_f;
    float fs_attained;
    // Get MCK
    if (config->mck_enable){
        float mck_hz   = config->fs * config->mck_mult;
        float mck_attained  = pio_div(mck_hz * 2, &mck_d, &mck_f);
        // Use MCK rate to get actual Fs, so the state-machines stay in sync
        fs_attained = mck_attained / config->mck_mult / 2;
        pio_sm_set_clkdiv_int_frac(i2s.pio, i2s.sm_mck, mck_d, mck_f);
    } else {
        fs_attained = config->fs;
    }

    float bck_hz = fs_attained * config->bit_depth * 2;
    pio_div(bck_hz * 4, &bck_d, &bck_f);
    pio_sm_set_clkdiv_int_frac(i2s.pio, i2s.sm_dat, bck_d, bck_f);
}

// Change samplerate
void set_samplerate(uint32_t rate){
    // halt state-machines
    pio_set_sm_mask_enabled(i2s.pio, i2s.sm_mask, 0);
    // reset clock-dividers
    pio_clkdiv_restart_sm_mask(i2s.pio, i2s.sm_mask);
    i2s.config.fs = rate;
    set_clocks(&i2s.config);
    pio_enable_sm_mask_in_sync(i2s.pio, i2s.sm_mask);
}

static void dma_double_buffer_init() {
    i2s.dma_ch_in_data  = dma_claim_unused_channel(true);
    i2s.dma_ch_in_ctrl  = dma_claim_unused_channel(true);
    i2s.dma_ch_out_data = dma_claim_unused_channel(true);
    i2s.dma_ch_out_ctrl = dma_claim_unused_channel(true);

    // Control DMAs will alternate the address of the data DMAs for double buffering
    ctrl_in[0]  = i2s.input_buffer;
    ctrl_in[1]  = &i2s.input_buffer[STEREO_BUFFER_SIZE];
    ctrl_out[0] = i2s.output_buffer;
    ctrl_out[1] = &i2s.output_buffer[STEREO_BUFFER_SIZE];

    // Configure output control DMA
    dma_channel_config c = dma_channel_get_default_config(i2s.dma_ch_out_ctrl);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 3);  // Wrap address at 2 words
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_set_config(i2s.dma_ch_out_ctrl, &c, false);
    dma_channel_set_read_addr(i2s.dma_ch_out_ctrl, ctrl_out, false);
    dma_channel_set_write_addr(i2s.dma_ch_out_ctrl, &dma_hw->ch[i2s.dma_ch_out_data].al3_read_addr_trig, false);
    dma_channel_set_trans_count(i2s.dma_ch_out_ctrl, 1, false); // Transfer 1 word at a time

    // Configure output data DMA
    c = dma_channel_get_default_config(i2s.dma_ch_out_data);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_chain_to(&c, i2s.dma_ch_out_ctrl);
    channel_config_set_dreq(&c, pio_get_dreq(i2s.pio, i2s.sm_dat, true));
    dma_channel_set_config(i2s.dma_ch_out_data, &c, false);
    dma_channel_set_write_addr(i2s.dma_ch_out_data, &i2s.pio->txf[i2s.sm_dat], false);
    dma_channel_set_trans_count(i2s.dma_ch_out_data, STEREO_BUFFER_SIZE, false);

    // Configure input control DMA
    c = dma_channel_get_default_config(i2s.dma_ch_in_ctrl);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 3);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_set_config(i2s.dma_ch_in_ctrl, &c, false);
    dma_channel_set_read_addr(i2s.dma_ch_in_ctrl, ctrl_in, false);
    dma_channel_set_write_addr(i2s.dma_ch_in_ctrl, &dma_hw->ch[i2s.dma_ch_in_data].al2_write_addr_trig, false);
    dma_channel_set_trans_count(i2s.dma_ch_in_ctrl, 1, false);

    // Configure input data DMA
    c = dma_channel_get_default_config(i2s.dma_ch_in_data);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_chain_to(&c, i2s.dma_ch_in_ctrl);
    channel_config_set_dreq(&c, pio_get_dreq(i2s.pio, i2s.sm_dat, false));
    dma_channel_set_config(i2s.dma_ch_in_data, &c, false);
    dma_channel_set_read_addr(i2s.dma_ch_in_data, &i2s.pio->rxf[i2s.sm_dat], false);
    dma_channel_set_trans_count(i2s.dma_ch_in_data, STEREO_BUFFER_SIZE, false);

    // Input channel should complete after the output channel
    dma_channel_set_irq0_enabled(i2s.dma_ch_in_data, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_cb);
    irq_set_enabled(DMA_IRQ_0, true);

    // Enable all the dma channels
    dma_channel_start(i2s.dma_ch_out_ctrl);
    dma_channel_start(i2s.dma_ch_in_ctrl);
}

static void i2s_sync_program_init(PIO pio, const i2s_config* config) {
    uint32_t offset  = 0;
    i2s.pio     = pio;
    i2s.sm_mask = 0;
    i2s.config.mck_pin = config->mck_pin;
    i2s.config.dout_pin = config->dout_pin;
    i2s.config.din_pin = config->din_pin;
    i2s.config.clock_pin_base = config->clock_pin_base;
    i2s.config.mck_enable = config->mck_enable;
    i2s.config.fs = config->fs;
    i2s.config.bit_depth = config->bit_depth;
    i2s.config.mck_mult = config->mck_mult;

    if (config->mck_enable) {
        // mck state-machine
        i2s.sm_mck = pio_claim_unused_sm(pio, true);
        i2s.sm_mask |= (1u << i2s.sm_mck);
        offset = pio_add_program(pio, &i2s_mck_program);
        i2s_mck_program_init(pio, i2s.sm_mck, offset, config->mck_pin);
    }
    // Data state-machine
    i2s.sm_dat = pio_claim_unused_sm(pio, true);
    i2s.sm_mask |= (1u << i2s.sm_dat);
    offset = pio_add_program(pio, &i2s_bidir_master_program);
    i2s_bidir_master_program_init(pio, i2s.sm_dat, offset, config->bit_depth, config->dout_pin, config->din_pin, config->clock_pin_base);
    set_clocks(config);
}

void i2s_program_init(PIO pio, const i2s_config* config, void (*i2s_cb)(void)) {
    i2s_sync_program_init(pio, config);
    i2s_handler = i2s_cb;
    dma_double_buffer_init();
    pio_enable_sm_mask_in_sync(i2s.pio, i2s.sm_mask);
}

uint8_t i2s_get_active_buff(){
    return buff_num;
}

void i2s_read_buff(uint32_t* buff){
    uint8_t start_idx;
    if (i2s_get_active_buff()){
        start_idx = 0;
    } else {
        start_idx = STEREO_BUFFER_SIZE;
    }
   	for (size_t i = 0; i < STEREO_BUFFER_SIZE; i++) {
	    buff[i] = i2s.input_buffer[i+start_idx];
	}
}

void i2s_write_buff(uint32_t* buff){
    uint8_t start_idx;
    if (i2s_get_active_buff()){
        start_idx = 0;
    } else {
        start_idx = STEREO_BUFFER_SIZE;
    }
   	for (size_t i = 0; i < STEREO_BUFFER_SIZE; i++) {
	    i2s.output_buffer[i+start_idx] = buff[i];
	}
}

void i2s_stop(){
    pio_set_sm_mask_enabled(i2s.pio, i2s.sm_mask, 0);
    pio_restart_sm_mask(i2s.pio, i2s.sm_mask);
    dma_channel_abort(i2s.dma_ch_in_ctrl);
    dma_channel_abort(i2s.dma_ch_in_data);
    dma_channel_abort(i2s.dma_ch_out_ctrl);
    dma_channel_abort(i2s.dma_ch_out_data);
}

void i2s_start(){
    dma_channel_start(i2s.dma_ch_in_ctrl);
    dma_channel_start(i2s.dma_ch_out_ctrl);
    pio_enable_sm_mask_in_sync(i2s.pio, i2s.sm_mask);
}

void dma_cb(){
    // Clear irq
    dma_channel_acknowledge_irq0(i2s.dma_ch_in_data);
    buff_num = *(int32_t**)dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr == i2s.input_buffer;
    i2s_handler();
}
