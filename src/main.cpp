#include <stdio.h>
#include "pico/stdlib.h"
#include <hardware/spi.h>
#include <hardware/irq.h>
#include <hardware/timer.h>
#include "usb_descriptors.h"
#include "MIDI_Config.h"
#include "SPI_RP2040.h"
#include "MCP2517.h"
#include "MIDI_Driver.h"
#include "audio_driver.h"
#include "usb.h"
#include <tusb.h>
#include "RingBuffer.h"
#include "usb_midi_host.h"

SPI_RP2040_C SPI_CAN = SPI_RP2040_C(spi1, 1);
MCP2517_C CAN = MCP2517_C(&SPI_CAN, 0);
RingBuffer<8, MIDI_UMP_t> canBuffer;

void dma1_irq_handler();
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
MIDI_UMP_t lostMsg;
bool hasLost;

int main(void){
	// Board init
	set_sys_clock_khz(132000, true);

	gpio_init(LEDH);
	gpio_init(LEDD);
	gpio_set_dir(LEDH, GPIO_OUT);
	gpio_set_dir(LEDD, GPIO_OUT);
	gpio_init(M2IDI_CAN_INT_PIN);
	gpio_set_dir(M2IDI_CAN_INT_PIN, GPIO_IN);

	SPI_CAN.Init(SPI_CAN_CONF);

	// Configure button pin with interrupt
	// TODO

	USB_Init();
	audio_init();

	MIDI_CAN.Set_handler(MIDI_CAN_UMP_handler);
	MIDI_USB.Set_handler(MIDI_USB_UMP_handler);

	// Enable SPI interrupt
	irq_set_exclusive_handler(DMA_IRQ_1, dma1_irq_handler);
	irq_set_enabled(DMA_IRQ_1, true);

	CAN.Init(CAN_CONF);
	CAN.Set_Rx_Header_Callback(CAN_Receive_Header);
	CAN.Set_Rx_Data_Callback(CAN_Receive_Data);

    while (true){
		audio_task();
		midi_task();
		USB_Service();

		if (CAN.Ready()){
			check_can_int();
		}
		if (canBuffer.Count() && CAN.Ready()){
			static int8_t numBytes = 0;
			char buff[16];
			MIDI_UMP_t msg;
			uint8_t length;

			if (hasLost) {
				// Recover lost message
				hasLost = false;
				msg = lostMsg;
			} else {
				canBuffer.Read(&msg);
			}
			length = MIDI_CAN.Encode(buff, &msg, 2);

			// TODO: this will probably break with multiple devices
			if (msg.type == MIDI_MT_E::Data128) {
				if (numBytes == 0){
					// Send first UMP
					CAN_Tx_msg_t outMsg;
					outMsg.dataLengthCode = CAN.Get_DLC(length);
					outMsg.id = (midiID & 0x7F)|(int(MIDI_MT_E::Data128) << 7);
					outMsg.payload = buff;
					outMsg.bitrateSwitch = true;
					CAN.Write_Message(&outMsg, 2);
				} else {
					CAN.Append_Payload(buff, length);
				}
				numBytes += length;
				if (msg.data128.status == MIDI2_DATA128_E::Single){
					CAN.Send_Message();
					numBytes = 0;
				} else if (msg.data128.status == MIDI2_DATA128_E::End){
					CAN.Send_Message();
					numBytes = 0;
				} else if (numBytes > 56){
					CAN.Send_Message();
					numBytes = 0;
				}
			} else if (msg.type == MIDI_MT_E::Data64){
				if (numBytes == 0){
					// Send first UMP
					CAN_Tx_msg_t outMsg;
					outMsg.dataLengthCode = CAN.Get_DLC(length);
					outMsg.id = (midiID & 0x7F)|(int(MIDI_MT_E::Data64) << 7);
					outMsg.payload = buff;
					outMsg.bitrateSwitch = true;
					CAN.Write_Message(&outMsg, 2);
				} else {
					CAN.Append_Payload(buff, length);
				}
				numBytes += length;
				if (msg.data64.status == MIDI2_DATA64_E::Single){
					CAN.Send_Message();
					numBytes = 0;
				} else if (msg.data64.status == MIDI2_DATA64_E::End){
					CAN.Send_Message();
					numBytes = 0;
				} else if (numBytes > 56){
					CAN.Send_Message();
					numBytes = 0;
				}
			} else if (msg.type == MIDI_MT_E::RealTime){
				if (numBytes != 0){
					// In the middle of sysex
					CAN.Send_Message();
					numBytes = 0;
					lostMsg = msg;
					hasLost = true;
				} else {
					// Send CAN message
					CAN_Tx_msg_t outMsg;
					outMsg.dataLengthCode = CAN.Get_DLC(length);
					outMsg.id = (midiID & 0x7F)|(int(MIDI_MT_E::RealTime) << 7);
					outMsg.payload = buff;
					CAN.Write_Message(&outMsg, 2);
					CAN.Send_Message();
				}
			} else {
				// Send CAN message
				CAN_Tx_msg_t outMsg;
				outMsg.dataLengthCode = CAN.Get_DLC(length);
				outMsg.id = (midiID & 0x7F)|(int(msg.type) << 7);
				outMsg.payload = buff;
				CAN.Write_Message(&outMsg, 2);
				CAN.Send_Message();
			}
		}

		static uint32_t timrr = 0;
		volatile uint32_t us_time = time_us_32();
		if (timrr <= time_us_32())	{
			timrr = time_us_32() + blinkTime;
			static bool hstate = 0;
			if (host_active){
				gpio_put(LEDH, hstate);
				gpio_put(LEDD, 0);
			} else {
				gpio_put(LEDH, 0);
				gpio_put(LEDD, hstate);
			}
			hstate = !hstate;
		}
    }
}

// Handle MIDI CAN data
void CAN_Receive_Header(CAN_Rx_msg_t* data){
	// Detect CAN id, and MIDI muid collisions
	// TODO

}

// Handle MIDI CAN data
void CAN_Receive_Data(char* data, uint8_t length){
	// Receive MIDI payload from CAN
	MIDI_CAN.Decode(data, length);
}

void check_can_int(){
	// Check interrupt pin to start reading from CAN controller
	if (!gpio_get(M2IDI_CAN_INT_PIN)){
		CAN.Check_Rx();
	}
}

void MIDI_CAN_UMP_handler(struct MIDI_UMP_t* msg){
	char tempData[16];
	uint8_t length;
	length = MIDI_USB.Encode(tempData, msg, MIDI_USB.Get_Version());

	if (length > 0){
		usb_midi_tx(tempData, length);
	}
}

// Handle USB midi data
void MIDI_USB_UMP_handler(struct MIDI_UMP_t* msg){
	msg->com.group = 1;
	canBuffer.Write(msg);
}

void midi_task(void){
	// Send MIDI data over USB
	if (host_active){
		// Host mode
		for (uint8_t i = 0; i < devNum; i++){
			if (devPend & (1 << i)) {
    			uint8_t cableNum;
    			uint8_t buffer[48];
    			uint32_t bytesRead = 69;
				while(bytesRead){
					bytesRead = tuh_midi_stream_read(devAddr[i], &cableNum, buffer, sizeof(buffer));
					MIDI_USB.Decode((char*) buffer, bytesRead);
					// TODO: send data to other devices
				}
				devPend &= ~(1 << i);
			}
		}
	} else {
		// Device mode
		uint8_t packet[16];
		uint8_t length;
		while ( tud_midi_available() ) {
			length = tud_midi_stream_read(packet, 16);
			MIDI_USB.Decode((char*)(packet), length);
		}
	}
}

void dma1_irq_handler (){
	SPI_CAN.Handler();
}
