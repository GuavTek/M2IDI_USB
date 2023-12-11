#include <stdio.h>
#include "pico/stdlib.h"
#include "usb_descriptors.h"
#include "MIDI_Config.h"
#include "SPI_RP2040.h"
#include "MCP2517.cpp"
#include "MIDI_Driver.cpp"
//#include "DMA_driver.h"
//#include "I2S_driver.h"
//#include "Audio_driver.h"
#include <tusb.h>
#include "RingBuffer.h"

SPI_RP2040_C SPI = SPI_RP2040_C();
MCP2517_C CAN = MCP2517_C(&SPI);
RingBuffer<8, MIDI_UMP_t> canBuffer;

void CAN_Receive_Header(CAN_Rx_msg_t* data);
void CAN_Receive_Data(char* data, uint8_t length);
void USB_Init();
void audio_task(void);
void midi_task(void);
void audio_dma_init();
void check_can_int();

void MIDI_CAN_UMP_handler(struct MIDI_UMP_t* msg);
void MIDI_USB_UMP_handler(struct MIDI_UMP_t* msg);

MIDI_C MIDI_USB(1);
MIDI_C MIDI_CAN(2);

uint32_t midiID = 239;
uint32_t blinkTime = 100;
uint32_t blinkTime2 = 100;
volatile uint32_t system_ticks = 0;

int main(void)
{
	//system_init();
	SPI.Init(SPI_CONF);
	//i2s_init(44100);
	
	//dma_init(base_descriptor, wrback_descriptor);
	//audio_dma_init();
	
	//PORT->Group[0].DIRSET.reg = (1 << 16) | (1 << 17);
	//PORT->Group[0].OUTCLR.reg = 1 << 16;

	// Configure the CAN_INT pin
	//struct port_config intCon = {
	//	.direction = PORT_PIN_DIR_INPUT,
	//	.input_pull = PORT_PIN_PULL_NONE,
	//	.powersave = false
	//};
	//port_pin_set_config(PIN_PA00, &intCon);
	
	USB_Init();
	tusb_init();

	MIDI_CAN.Set_handler(MIDI_CAN_UMP_handler);
	MIDI_USB.Set_handler(MIDI_USB_UMP_handler);
	
	//NVIC_EnableIRQ(SERCOM5_IRQn);
	//system_interrupt_enable_global();
	
	CAN.Init(CAN_CONF);
	CAN.Set_Rx_Header_Callback(CAN_Receive_Header);
	CAN.Set_Rx_Data_Callback(CAN_Receive_Data);
	
	char temp[4];
	CAN_Tx_msg_t message;
	message.payload = temp;
	message.canFDFrame = true;
	message.dataLengthCode = CAN.Get_DLC(4);
	message.id = 69;
	message.extendedID = false;
	temp[2] = 70;
	//CAN.Transmit_Message(&message, 2);
	
    while (true) 
    {
		// USB tasks
		tud_task();
		//tuh_task();
		
		//Debug_func();
		
		//audio_task();
		midi_task();
		
		//if (PORT->Group[0].IN.reg & (1 << 11)){
		//	PORT->Group[0].OUTCLR.reg = 1 << 16;
		//} else {
		//	PORT->Group[0].OUTSET.reg = 1 << 16;
		//}
		
		if (CAN.Ready()){
			check_can_int();
		}
		if (canBuffer.Count() && CAN.Ready()){
			char tempData[16];
			MIDI_UMP_t msg;
			uint8_t length;
			
			canBuffer.Read(&msg);
			length = MIDI_CAN.Encode(tempData, &msg, 2);

			CAN_Tx_msg_t txMsg;
			txMsg.dataLengthCode = CAN.Get_DLC(length);
			txMsg.payload = tempData;
			txMsg.id = midiID;
			//CAN.Transmit_Message(&txMsg, 2);
		}
		
		static uint32_t timrr = 0;
		if (timrr < system_ticks/**/)	{
			timrr = system_ticks + blinkTime;
			//PORT->Group[0].OUTTGL.reg = 1 << 17;
		}
		
    }
}

// Initialize clocks and pins for the USB port
void USB_Init(){
	
	//NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS(2);
	
	//SysTick_Config(F_CPU/1000);
	
	/* USB Clock init
	 * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
	 * for low speed and full speed operation. */
	
	// Enable USB clock
	//PM->APBCMASK.reg |= PM_APBBMASK_USB;
	//PM->AHBMASK.reg |= PM_AHBMASK_USB;
	
	// Select generic clock
	//GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | (GCLK_CLKCTRL_ID_USB);

	// USB Pin Init
	//PORT->Group[0].DIRSET.reg = (1 << 24) | (1 << 25);
	//PORT->Group[0].OUTCLR.reg = (1 << 24) | (1 << 25);
	//PORT->Group[0].PINCFG[24].bit.PULLEN = 0;
	//PORT->Group[0].PINCFG[25].bit.PULLEN = 0;
	
	//pin_set_peripheral_function(PINMUX_PA24G_USB_DM);
	//pin_set_peripheral_function(PINMUX_PA25G_USB_DP);
	
	// Enable USB ID pin
	//PORT->Group[0].DIRCLR.reg = (1 << 27);
	//PORT->Group[0].PINCFG[27].bit.INEN = 1;
}

// Handle MIDI CAN data
void CAN_Receive_Header(CAN_Rx_msg_t* data){
	//MIDI_CAN.Decode(data->payload, CAN.Get_Data_Length(data->dataLengthCode) );
}

// Handle MIDI CAN data
void CAN_Receive_Data(char* data, uint8_t length){
	//MIDI_CAN.Decode(data->payload, CAN.Get_Data_Length(data->dataLengthCode) );
}

void check_can_int(){
	//if (!port_pin_get_input_level(PIN_PA00)){
	//	CAN.Check_Rx();
	//}
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
	msg->com.group = 1;
	canBuffer.Write(msg);
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

void USB_Handler(void){
	tud_int_handler(0);
	
	//tuh_int_handler(0);
}

// For some reason the SERCOM5 interrupt leads to the SERCOM3 handler
// That was a painful debugging session
//void SERCOM5_Handler(void){
//	SPI.Handler();
//}
