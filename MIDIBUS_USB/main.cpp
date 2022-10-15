/*
 * MIDIBUS_USB.cpp
 *
 * Created: 26/06/2021 19:48:09
 * Author : GuavTek
 */ 

#include "MIDI_Config.h"
#include "sam.h"
#include "asf.h"
#include "MCP2517.h"
#include "SPI.h"
#include "MIDI_Driver.h"
#include "DMA_driver.h"
#include "I2S_driver.h"
#include "Audio_driver.h"
#include "tusb.h"


MCP2517_C CAN(SERCOM5);

void CAN_Receive(CAN_Rx_msg_t* data);
void USB_Init();
void audio_task(void);
void midi_task(void);
void audio_dma_init();

void MIDI_CAN_UMP_handler(struct MIDI_UMP_t* msg);
void MIDI_USB_UMP_handler(struct MIDI_UMP_t* msg);

MIDI_C MIDI_USB(1);
MIDI_C MIDI_CAN(2);

uint32_t midiID = 239;
uint32_t blinkTime = 100;
uint32_t blinkTime2 = 100;
volatile uint32_t system_ticks = 0;

// For debugging
volatile Dmac* tempDMA = DMAC;
volatile int64_t dropped_bytes = 0;
volatile I2s* tempI2S = I2S;
volatile uint64_t txufl_count = 0;
volatile uint64_t rxofl_count = 0;
volatile uint64_t txufr_count = 0;
volatile uint64_t rxofr_count = 0;
volatile uint64_t terr_count = 0;

inline void Debug_func() {
	if ((I2S->INTFLAG.bit.RXOR0) && mic_active && (i2s_rx_descriptor_wb->btctrl.valid == 0)){
		rxofl_count++;
	}
	if ((I2S->INTFLAG.bit.RXOR1) && spk_active && (i2s_tx_descriptor_wb->btctrl.valid == 0)){
		txufl_count++;
	}
	if (DMAC->INTSTATUS.bit.CHINT2){
		terr_count++;
	}
};

int main(void)
{
	system_init();
	CAN.Init(CAN_CONF, SPI_CONF);
	CAN.Set_Rx_Callback(CAN_Receive);
	i2s_init(44100);
	
	dma_init(base_descriptor, wrback_descriptor);
	audio_dma_init();
	
	PORT->Group[0].DIRSET.reg = (1 << 16) | (1 << 17);
	PORT->Group[0].OUTCLR.reg = 1 << 16;

	
	USB_Init();
	tusb_init();

	MIDI_CAN.Set_handler(MIDI_CAN_UMP_handler);
	MIDI_USB.Set_handler(MIDI_USB_UMP_handler);
	
	NVIC_EnableIRQ(SERCOM5_IRQn);
	system_interrupt_enable_global();
	
	char temp[4];
	CAN_Tx_msg_t message;
	message.payload = temp;
	message.canFDFrame = true;
	message.dataLengthCode = CAN.Get_DLC(4);
	message.id = 69;
	message.extendedID = false;
	temp[2] = 70;
	//CAN.Transmit_Message(&message, 2);
	
    while (1) 
    {
		CAN.State_Machine();
		
		// USB tasks
		tud_task();
		//tuh_task();
		
		Debug_func();
		
		audio_task();
		midi_task();
		
		
		if (PORT->Group[0].IN.reg & (1 << 11)){
			PORT->Group[0].OUTCLR.reg = 1 << 16;
		} else {
			PORT->Group[0].OUTSET.reg = 1 << 16;
		}
			
		static uint32_t timrr = 0;
		if (timrr < system_ticks/**/)	{
			timrr = system_ticks + blinkTime;
			PORT->Group[0].OUTTGL.reg = 1 << 17;
		}
		
    }
}

// Initialize clocks and pins for the USB port
void USB_Init(){
	
	NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS(2);
	
	SysTick_Config(F_CPU/1000);
	
	/* USB Clock init
	 * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
	 * for low speed and full speed operation. */
	
	// Enable USB clock
	PM->APBCMASK.reg |= PM_APBBMASK_USB;
	PM->AHBMASK.reg |= PM_AHBMASK_USB;
	
	// Select generic clock
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | (GCLK_CLKCTRL_ID_USB);

	// USB Pin Init
	PORT->Group[0].DIRSET.reg = (1 << 24) | (1 << 25);
	PORT->Group[0].OUTCLR.reg = (1 << 24) | (1 << 25);
	PORT->Group[0].PINCFG[24].bit.PULLEN = 0;
	PORT->Group[0].PINCFG[25].bit.PULLEN = 0;
	
	pin_set_peripheral_function(PINMUX_PA24G_USB_DM);
	pin_set_peripheral_function(PINMUX_PA25G_USB_DP);
	
	// Enable USB ID pin
	PORT->Group[0].DIRCLR.reg = (1 << 27);
	PORT->Group[0].PINCFG[27].bit.INEN = 1;
}

// Handle MIDI CAN data
void CAN_Receive(CAN_Rx_msg_t* data){
	MIDI_CAN.Decode(data->payload, CAN.Get_Data_Length(data->dataLengthCode) );
}

void MIDI_CAN_UMP_handler(struct MIDI_UMP_t* msg){
	char tempData[16];
	uint8_t length;
	length = MIDI_USB.Encode(tempData, msg, MIDI_USB.Get_Version());
	
	if (length > 0){
		tud_midi_stream_write(0, (uint8_t*)(tempData), length);
	}
}

// Handle USB midi data
void MIDI_USB_UMP_handler(struct MIDI_UMP_t* msg){
	char tempData[16];
	uint8_t length;
	msg->com.group = 1;
	length = MIDI_CAN.Encode(tempData, msg, 2);
	
	CAN_Tx_msg_t txMsg;
	txMsg.dataLengthCode = CAN.Get_DLC(length);
	txMsg.payload = tempData;
	txMsg.id = midiID;
	CAN.Transmit_Message(&txMsg, 2);
}

void midi_task(void)
{
	uint8_t packet[16];
	uint8_t length;
	while ( tud_midi_available() ) {
		length = tud_midi_stream_read(packet, 16);
		MIDI_USB.Decode((char*)(packet), length);
	}
}
/**/

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	blinkTime = 2000;
	blinkTime2 = 1998;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	blinkTime = 100;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	blinkTime = 1000;
}

void SysTick_Handler (void)
{
	system_ticks++;
}

uint32_t board_millis(void)
{
	return system_ticks;
}

void DMAC_Handler(){
	uint32_t tempFlag = DMAC->INTPEND.bit.ID;
	DMAC->CHID.reg = tempFlag;
	
	dma_resume(tempFlag);
	DMAC->CHINTENCLR.bit.SUSP = 1;
}

void USB_Handler(void){
	tud_int_handler(0);
	
	//tuh_int_handler(0);
}

// For some reason the SERCOM5 interrupt leads to the SERCOM3 handler
// That was a painful debugging session
void SERCOM5_Handler(void){
	CAN.Handler();
}

