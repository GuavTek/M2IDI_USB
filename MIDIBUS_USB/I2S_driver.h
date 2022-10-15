/*
 * I2S_driver.h
 *
 * Created: 01/07/2022 14:04:57
 *  Author: GuavTek
 */ 


#ifndef I2S_DRIVER_H_
#define I2S_DRIVER_H_

#include "sam.h"
#include "asf.h"
#include "DMA_driver.h"

enum {
	I2S_FS_LEFT_MAYBE,
	I2S_FS_RIGHT_MAYBE
} i2s_fs_state;


// DMA descriptors
extern DMA_Descriptor_t* i2s_rx_descriptor_a;
extern DMA_Descriptor_t* i2s_rx_descriptor_b;
extern DMA_Descriptor_t* i2s_rx_descriptor_wb;

extern DMA_Descriptor_t* i2s_tx_descriptor_a;
extern DMA_Descriptor_t* i2s_tx_descriptor_b;
extern DMA_Descriptor_t* i2s_tx_descriptor_wb;

void i2s_init(uint32_t samplerate);
void i2s_set_freq(uint32_t samplerate);
void i2s_set_output_wordsize(uint8_t size);
void i2s_set_input_wordsize(uint8_t size);

// Compensate for inaccurate reference clock
void i2s_adjust_freq(int32_t delta);

#endif /* I2S_DRIVER_H_ */