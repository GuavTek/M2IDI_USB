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
#include "usb.h"

SPI_RP2040_C SPI = SPI_RP2040_C();
MCP2517_C CAN = MCP2517_C(&SPI);
RingBuffer<8, MIDI_UMP_t> canBuffer;

void CAN_Receive_Header(CAN_Rx_msg_t* data);
void CAN_Receive_Data(char* data, uint8_t length);
void audio_task(void);
void midi_task(void);
void audio_dma_init();
void check_can_int();

void MIDI_CAN_UMP_handler(struct MIDI_UMP_t* msg);
void MIDI_USB_UMP_handler(struct MIDI_UMP_t* msg);

MIDI_C MIDI_USB(1);
MIDI_C MIDI_CAN(2);

uint32_t midiID = 239;

int main(void){
	gpio_init(LEDH);
	gpio_init(LEDD);
	gpio_set_dir(LEDH, GPIO_OUT);
	gpio_set_dir(LEDD, GPIO_OUT);
	//system_init();
	//SPI.Init(SPI_CONF);
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

	MIDI_CAN.Set_handler(MIDI_CAN_UMP_handler);
	MIDI_USB.Set_handler(MIDI_USB_UMP_handler);
	
	//NVIC_EnableIRQ(SERCOM5_IRQn);
	//system_interrupt_enable_global();
	
	//CAN.Init(CAN_CONF);
	//CAN.Set_Rx_Header_Callback(CAN_Receive_Header);
	//CAN.Set_Rx_Data_Callback(CAN_Receive_Data);
	
    while (true){
		// USB tasks
		//tud_task();
		tuh_task();
		
		//Debug_func();
		
		//audio_task();
		midi_task();
		
		//if (PORT->Group[0].IN.reg & (1 << 11)){
		//	PORT->Group[0].OUTCLR.reg = 1 << 16;
		//} else {
		//	PORT->Group[0].OUTSET.reg = 1 << 16;
		//}
		
		//if (CAN.Ready()){
		//	check_can_int();
		//}
		//if (canBuffer.Count() && CAN.Ready()){
		//	char tempData[16];
		//	MIDI_UMP_t msg;
		//	uint8_t length;
		//	
		//	canBuffer.Read(&msg);
		//	length = MIDI_CAN.Encode(tempData, &msg, 2);
		//	CAN_Tx_msg_t txMsg;
		//	txMsg.dataLengthCode = CAN.Get_DLC(length);
		//	txMsg.payload = tempData;
		//	txMsg.id = midiID;
		//	//CAN.Transmit_Message(&txMsg, 2);
		//}
		
		static uint32_t timrr = 0;
		if (timrr + blinkTime < system_ticks/**/)	{
			timrr = system_ticks;
			static bool hstate = 0;
			gpio_put(LEDH, hstate);
			hstate = !hstate;
		}
		system_ticks++;
    }
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
		//tud_midi_stream_write(0, (uint8_t*)(tempData), length);
	}
}

// Handle USB midi data
void MIDI_USB_UMP_handler(struct MIDI_UMP_t* msg){
	msg->com.group = 1;
	canBuffer.Write(msg);
}

void midi_task(void){
	//uint8_t packet[16];
	//uint8_t length;
	//while ( tud_midi_available() ) {
	//	length = tud_midi_stream_read(packet, 16);
	//	MIDI_USB.Decode((char*)(packet), length);
	//}
}
/**/

// For some reason the SERCOM5 interrupt leads to the SERCOM3 handler
// That was a painful debugging session
//void SERCOM5_Handler(void){
//	SPI.Handler();
//}
