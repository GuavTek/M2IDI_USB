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
	//SPI.Init(SPI_CONF);
	//i2s_init(44100);
	
	//dma_init(base_descriptor, wrback_descriptor);
	//audio_dma_init();
	
	// Configure button pin with interrupt
	// TODO

	USB_Init();

	MIDI_CAN.Set_handler(MIDI_CAN_UMP_handler);
	MIDI_USB.Set_handler(MIDI_USB_UMP_handler);
	
	// Enable SPI interrupt
	// TODO

	//CAN.Init(CAN_CONF);
	//CAN.Set_Rx_Header_Callback(CAN_Receive_Header);
	//CAN.Set_Rx_Data_Callback(CAN_Receive_Data);
	
    while (true){
		// USB tasks
		//tud_task();
		tuh_task();
		
		//audio_task();
		midi_task();
		
		// Detect usb status
		//if (gpio_get(USB_ID_PIN)){
		//	gpio_set(LEDD, 1);
		//} else {
		//	gpio_set(LEDD, 0);
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
		if (timrr + blinkTime < time_us_32())	{
			timrr = time_us_32();
			static bool hstate = 0;
			gpio_put(LEDH, hstate);
			hstate = !hstate;
		}
    }
}

// Handle MIDI CAN data
void CAN_Receive_Header(CAN_Rx_msg_t* data){
	// Detect CAN id, and MIDI muid collisions
	// TODO
	//MIDI_CAN.Decode(data->payload, CAN.Get_Data_Length(data->dataLengthCode) );
}

// Handle MIDI CAN data
void CAN_Receive_Data(char* data, uint8_t length){
	// Receive MIDI payload from CAN
	//MIDI_CAN.Decode(data->payload, CAN.Get_Data_Length(data->dataLengthCode) );
}

void check_can_int(){
	// Check interrupt pin to start reading from CAN controller
	//if (!port_pin_get_input_level(PIN_PA00)){
	//	CAN.Check_Rx();
	//}
}

void MIDI_CAN_UMP_handler(struct MIDI_UMP_t* msg){
	char tempData[16];
	uint8_t length;
	length = MIDI_USB.Encode(tempData, msg, MIDI_USB.Get_Version());
	
	if (length > 0){
		// TODO: select host or device when appropriate
		//tud_midi_stream_write(0, (uint8_t*)(tempData), length);
	}
}

// Handle USB midi data
void MIDI_USB_UMP_handler(struct MIDI_UMP_t* msg){
	msg->com.group = 1;
	canBuffer.Write(msg);
}

void midi_task(void){
	// Send MIDI data over USB
	// TODO

	//uint8_t packet[16];
	//uint8_t length;
	//while ( tud_midi_available() ) {
	//	length = tud_midi_stream_read(packet, 16);
	//	MIDI_USB.Decode((char*)(packet), length);
	//}
}
