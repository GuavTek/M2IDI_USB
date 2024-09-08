
#include <stdio.h>
#include "hardware/pio.h"

#ifndef I2S_TEST_I2S_H
#define I2S_TEST_I2S_H

#define AUDIO_BUFFER_FRAMES 96
#define STEREO_BUFFER_SIZE  AUDIO_BUFFER_FRAMES * 2  // roughly 1ms

typedef struct i2s_config {
    uint32_t fs;
    uint32_t mck_mult;
    uint8_t  bit_depth;
    uint8_t  mck_pin;
    uint8_t  dout_pin;
    uint8_t  din_pin;
    uint8_t  clock_pin_base;
    bool     mck_enable;
} i2s_config;

typedef struct pio_i2s {
    PIO        pio;
    uint8_t    sm_mask;
    uint8_t    sm_mck;
    uint8_t    sm_dat;
    uint8_t    dma_ch_in_data;
    uint8_t    dma_ch_in_ctrl;
    uint8_t    dma_ch_out_data;
    uint8_t    dma_ch_out_ctrl;
    int32_t    input_buffer[STEREO_BUFFER_SIZE * 2];
    int32_t    output_buffer[STEREO_BUFFER_SIZE * 2];
    i2s_config config;
} pio_i2s;

void i2s_program_init(PIO pio, const i2s_config* config, void (*i2s_cb)(void));
void i2s_stop();
void i2s_start();
void set_samplerate(uint32_t rate);
uint8_t i2s_get_active_buff();
void i2s_read_buff(uint32_t* buff);
void i2s_write_buff(uint32_t* buff);

#endif  // I2S_TEST_I2S_H
