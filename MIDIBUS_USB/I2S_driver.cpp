/*
 * I2S_driver.cpp
 *
 * Created: 01/07/2022 14:04:33
 *  Author: GuavTek
 */ 

#include "I2S_driver.h"

DMA_Descriptor_t* i2s_rx_descriptor_a = &base_descriptor[0];
DMA_Descriptor_t* i2s_rx_descriptor_b = &transact_descriptor[0];
DMA_Descriptor_t* i2s_rx_descriptor_wb = &wrback_descriptor[0];

DMA_Descriptor_t* i2s_tx_descriptor_a = &base_descriptor[1];
DMA_Descriptor_t* i2s_tx_descriptor_b = &transact_descriptor[1];
DMA_Descriptor_t* i2s_tx_descriptor_wb = &wrback_descriptor[1];

int32_t compensateFreq = 0;
uint32_t fs_samplerate;

void i2s_init(const uint32_t samplerate){
	// Set up pin functions
	PORT->Group[0].DIRCLR.reg = 1 << 7;
	pin_set_peripheral_function(PINMUX_PA07G_I2S_SD0);
	
	PORT->Group[0].DIRSET.reg = 1 << 8;
	pin_set_peripheral_function(PINMUX_PA08G_I2S_SD1);
	
	PORT->Group[0].DIRSET.reg = 1 << 9;
	pin_set_peripheral_function(PINMUX_PA09G_I2S_MCK0);
	
	PORT->Group[0].DIRSET.reg = 1 << 10;
	pin_set_peripheral_function(PINMUX_PA10G_I2S_SCK0);
	
	PORT->Group[0].DIRSET.reg = 1 << 11;
	pin_set_peripheral_function(PINMUX_PA11G_I2S_FS0);
	
	// Enable reading of I2S FS pin
	PORT->Group[0].DIRCLR.reg = (1 << 11);
	PORT->Group[0].PINCFG[11].bit.INEN = 1;
	
	
	// Enable peripheral clocks
	PM->APBCMASK.reg |= PM_APBCMASK_I2S;
	
	// Set DPLL frequency
	i2s_set_freq(samplerate);
	
	// Select generic clock
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK7 | (GCLK_CLKCTRL_ID_I2S_0);
	
	// Configure clock unit
	// Use 384x fs
	I2S->CLKCTRL[0].reg =	I2S_CLKCTRL_SLOTSIZE_32 |
							I2S_CLKCTRL_NBSLOTS(1) |
							I2S_CLKCTRL_BITDELAY_I2S |
							I2S_CLKCTRL_FSWIDTH_SLOT |
							I2S_CLKCTRL_MCKSEL_GCLK | 
							I2S_CLKCTRL_MCKOUTDIV(0) |
							I2S_CLKCTRL_SCKSEL_MCKDIV | 
							I2S_CLKCTRL_MCKDIV((384 / (2*32))-1) |
							I2S_CLKCTRL_FSSEL_SCKDIV |
							I2S_CLKCTRL_MCKEN;
	
	// Configure Serializers
	I2S->SERCTRL[0].reg =	I2S_SERCTRL_CLKSEL_CLK0 |
							I2S_SERCTRL_MONO_STEREO |
							I2S_SERCTRL_BITREV_MSBIT |
							I2S_SERCTRL_DATASIZE_32 |
							I2S_SERCTRL_EXTEND_ZERO	|
							I2S_SERCTRL_SLOTADJ_LEFT |
							I2S_SERCTRL_WORDADJ_RIGHT |
							I2S_SERCTRL_SERMODE_RX;
							
	I2S->SERCTRL[1].reg =	I2S_SERCTRL_CLKSEL_CLK0 |
							I2S_SERCTRL_MONO_STEREO |
							I2S_SERCTRL_BITREV_MSBIT |
							I2S_SERCTRL_DATASIZE_32 |
							I2S_SERCTRL_EXTEND_ZERO |
							I2S_SERCTRL_SLOTADJ_LEFT |
							I2S_SERCTRL_WORDADJ_RIGHT |
							I2S_SERCTRL_TXSAME_SAME |
							I2S_SERCTRL_SERMODE_TX;
	
	// Set interrupts, Nope use DMA instead
	I2S->INTENSET.bit.RXRDY0 = 0;// 1;
	I2S->INTENSET.bit.TXRDY1 = 0;// 1;
	
	i2s_fs_state = I2S_FS_LEFT_MAYBE;
	
	// Enable clock unit and serializers
	while(I2S->SYNCBUSY.reg & I2S_SYNCBUSY_ENABLE);
	I2S->CTRLA.reg |=
		I2S_CTRLA_CKEN0 |
		I2S_CTRLA_SEREN0 |
		I2S_CTRLA_SEREN1 |
		I2S_CTRLA_ENABLE;
	
	// Enable interrupt
	//NVIC_EnableIRQ(I2S_IRQn);
	
	// Write dummy data
	while(I2S->SYNCBUSY.bit.SEREN1);
	I2S->DATA[1].reg = 0;

}

void i2s_set_freq(){
	uint32_t tmpldr;
	uint8_t  tmpldrfrac;
	uint32_t refclk = 32768;
	
	/* Calculate LDRFRAC and LDR */
	uint64_t output_frequency = fs_samplerate * 384 - 100000;// + compensateFreq;
	tmpldr = (output_frequency << 4) / refclk;
	tmpldrfrac = tmpldr & 0x0f;
	tmpldr = (tmpldr >> 4) - 1;
	
	SYSCTRL->DPLLRATIO.reg =
	SYSCTRL_DPLLRATIO_LDRFRAC(tmpldrfrac) |
	SYSCTRL_DPLLRATIO_LDR(tmpldr);
}

void i2s_set_freq(uint32_t samplerate) {
	fs_samplerate = samplerate;
	i2s_set_freq();
}

// Compensate for inaccurate reference clock
void i2s_adjust_freq(int32_t delta){
	compensateFreq += delta;
	//i2s_set_freq();
}

void i2s_set_output_wordsize(uint8_t size){
	uint8_t tempSize;
	if ((size == 24) || (size == 32)){
		tempSize = I2S_SERCTRL_DATASIZE_32_Val;
	} else if (size == 16){
		tempSize = I2S_SERCTRL_DATASIZE_16_Val;
	} else if (size == 8){
		tempSize = I2S_SERCTRL_DATASIZE_8_Val;
	} else {
		// Invalid size
		return;
	}
	
	// Is changed?
	if(I2S->SERCTRL[1].bit.DATASIZE == tempSize){
		// No change
		return;
	}
	
	// Disable serializer
	I2S->CTRLA.bit.SEREN1 = 0;
	while(I2S->SYNCBUSY.bit.SEREN1);
	
	// Set data size
	I2S->SERCTRL[1].bit.DATASIZE = tempSize;
	
	// Enable serializer
	I2S->CTRLA.bit.SEREN1 = 1;
	while(I2S->SYNCBUSY.bit.SEREN1);
	//I2S->DATA[1].reg = 0;	// Dummy data
	
}

void i2s_set_input_wordsize(uint8_t size){
	uint8_t tempSize;
	if ((size == 24) || (size == 32)){
		tempSize = I2S_SERCTRL_DATASIZE_32_Val;
	} else if (size == 16){
		tempSize = I2S_SERCTRL_DATASIZE_16_Val;
	} else if (size == 8){
		tempSize = I2S_SERCTRL_DATASIZE_8_Val;
	} else {
	// Invalid size
	return;
}
	
	// Is changed?
	if(I2S->SERCTRL[0].bit.DATASIZE == tempSize){
		// No change
		return;
	}
	
	// Disable serializer
	I2S->CTRLA.bit.SEREN0 = 0;
	while(I2S->SYNCBUSY.bit.SEREN0);
	
	// Set data size
	I2S->SERCTRL[0].bit.DATASIZE = tempSize;
	
	// Enable serializer
	I2S->CTRLA.bit.SEREN0 = 1;
	while(I2S->SYNCBUSY.bit.SEREN0);
	
}

