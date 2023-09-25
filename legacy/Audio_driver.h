/*
 * Audio_driver.h
 *
 * Created: 15/07/2022 12:48:29
 *  Author: GuavTek
 */ 


#ifndef AUDIO_DRIVER_H_
#define AUDIO_DRIVER_H_

#include "sam.h"
#include "I2S_driver.h"
#include "DMA_driver.h"
#include "tusb.h"

extern bool mic_active;
extern bool spk_active;

void audio_dma_init();
void audio_task(void);


#endif /* AUDIO_DRIVER_H_ */