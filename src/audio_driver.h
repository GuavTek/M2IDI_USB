/*
 * audio_driver.h
 *
 * Created: 15/07/2022
 *  Author: GuavTek
 */ 


#ifndef AUDIO_DRIVER_H_
#define AUDIO_DRIVER_H_

#include "i2s.h"
#include "tusb.h"

extern bool mic_active;
extern bool spk_active;

void audio_init();
void audio_task(void);


#endif /* AUDIO_DRIVER_H_ */