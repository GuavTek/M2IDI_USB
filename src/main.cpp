#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include <hardware/clocks.h>
#include <hardware/spi.h>
#include <hardware/irq.h>
#include <hardware/timer.h>
#include "usb_descriptors.h"
#include "MIDI_Config.h"
#include "SPI_RP2040.h"
#include "MCP2517.h"
#include "umpProcessor.h"
#include "bytestreamToUMP.h"
#include "umpToBytestream.h"
#include "audio_driver.h"
#include "usb.h"
#include <tusb.h>
#include "RingBuffer.h"
#include "usb_midi_host.h"

SPI_RP2040_C SPI_CAN = SPI_RP2040_C(spi1, 1);
MCP2517_C CAN = MCP2517_C(&SPI_CAN, 0);
RingBuffer<32, uint32_t> canBuffer[17];
RingBuffer<128, char> usbBuffer;

void dma1_irq_handler();
void CAN_Receive_Header(CAN_Rx_msg_t* data);
void CAN_Receive_Data(char* data, uint8_t length);
void audio_task(void);
void midi_task(void);
void audio_dma_init();
void check_can_int();

void midi_stream_discovery(uint8_t majVer, uint8_t minVer, uint8_t filter);

umpProcessor MIDI_PROC;
bytestreamToUMP MIDI_BS2UMP;
umpToBytestream MIDI_UMP2BS;

uint32_t midiID = 239;

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

	MIDI_PROC.setMidiEndpoint(midi_stream_discovery);

	// Enable SPI interrupt
	irq_set_exclusive_handler(DMA_IRQ_1, dma1_irq_handler);
	irq_set_enabled(DMA_IRQ_1, true);

	CAN.Init(CAN_CONF);
	CAN.Set_Rx_Header_Callback(CAN_Receive_Header);
	CAN.Set_Rx_Data_Callback(CAN_Receive_Data);

    while (true){
		static uint8_t currentBuffer = 0;
		audio_task();
		midi_task();
		USB_Service();

		if (CAN.Ready()){
			check_can_int();
		}

		if (CAN.Ready() && canBuffer[currentBuffer].Count()){
			int8_t numBytes = 0;
			int8_t sendingGroup;
			char buff[64];

			memset(buff, 0, 64);

			// Load data in CAN
			// TODO: Check if CAN controller memory is full
			while (canBuffer[currentBuffer].Count()){
				// Peek in buffer to check length
				uint8_t msgLen;
				uint8_t wordLen;
				uint32_t msgWord;
				canBuffer[currentBuffer].Peek(&msgWord);
				msgLen = get_ump_size(get_ump_type(msgWord));
				wordLen = msgLen >> 2;
				if (canBuffer[currentBuffer].Count() < wordLen){
					// Message incomplete
					break;
				} else if ((msgLen+numBytes) > 64){
					// Message does not fit in buffer
					break;
				}
				// Copy bytes
				for (uint8_t i = 0; i < wordLen; i++){
					canBuffer[currentBuffer].Read(&msgWord);
					buff[numBytes++] = (msgWord >> 24) & 0xff;
					buff[numBytes++] = (msgWord >> 16) & 0xff;
					buff[numBytes++] = (msgWord >>  8) & 0xff;
					buff[numBytes++] = msgWord & 0xff;
				}
			}

			if (numBytes > 0){
				CAN_Tx_msg_t outMsg;
				outMsg.dataLengthCode = CAN.Get_DLC(numBytes);
				outMsg.id = (midiID & 0x7F)|((currentBuffer & 0xf) << 7);
				outMsg.extendedID = (currentBuffer == 16);
				outMsg.payload = buff;
				outMsg.bitrateSwitch = false; // TODO: fix datarate
				CAN.Write_Message(&outMsg, 2);
				CAN.Send_Message();
			}
		}
		if (currentBuffer == 16){
			currentBuffer = 0;
		} else {
			currentBuffer++;
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
	for (uint8_t i = 0; i < length; i++) {
		usbBuffer.Write(&data[i]);
	}
}

void check_can_int(){
	// Check interrupt pin to start reading from CAN controller
	if (!gpio_get(M2IDI_CAN_INT_PIN)){
		CAN.Check_Rx();
	}
}

void midi_stream_discovery(uint8_t majVer, uint8_t minVer, uint8_t filter){
	//Upon Recieving the filter it is important to return the information requested
	if(filter & 0x1){ //Endpoint Info Notification
		//std::array<uint32_t, 4> ump = UMPMessage::mtFMidiEndpointInfoNotify(1, true, true, false, false);
		//sendUMP(ump.data(),4);
	}

	if(filter & 0x2) {
		//std::array<uint32_t, 4> ump = UMPMessage::mtFMidiEndpointDeviceInfoNotify(
		//{MIDI_MFRID & 0xff, (MIDI_MFRID >> 8) & 0xff, (MIDI_MFRID >> 16) & 0xff},
		//{MIDI_FAMID & 0xff, (MIDI_FAMID >> 8) & 0xff},
		//{DEVICE_MODELID & 0xff, (DEVICE_MODELID >> 8) & 0xff},
		//{DEVICE_VERSIONID & 0xff, (DEVICE_VERSIONID >> 8) & 0xff, (DEVICE_VERSIONID >> 16) & 0xff, (DEVICE_VERSIONID >> 24) & 0xff});
		//sendUMP( ump.data(), 4);
	}

	if(filter & 0x4) {
		//uint8_t friendlyNameLength = sizeof(DEVICE_NAME);
		//for(uint8_t offset=0; offset<friendlyNameLength; offset+=14) {
		//	std::array<uint32_t, 4> ump = UMPMessage::mtFMidiEndpointTextNotify(MIDIENDPOINT_NAME_NOTIFICATION, offset, (uint8_t *) DEVICE_NAME,friendlyNameLength);
			//sendUMP(ump.data(),4);
		//}
	}

	if(filter & 0x8) {
		// TODO: read MCU unique ID
		//int8_t piiLength = sizeof(PRODUCT_INSTANCE_ID);
		//for(uint8_t offset=0; offset<piiLength; offset+=14) {
		//	std::array<uint32_t, 4> ump = UMPMessage::mtFMidiEndpointTextNotify(PRODUCT_INSTANCE_ID, offset, (uint8_t *) buff,piiLength);
		//	//sendUMP(ump.data(),4);
		//}
	}

	if(filter & 0x10){
		//std::array<uint32_t, 4> ump = UMPMessage::mtFNotifyProtocol(0x2,false,false);
		//sendUMP(ump.data(),4);
	}
}

void midi_task(void){
	static int8_t umpLen = 0;
	static uint8_t umpGroup;
	if (!host_active && !tud_mounted()){
		umpLen = 0;
	}
	MIDI_BS2UMP.defaultGroup = 1;
	// Read MIDI data from USB
	if (host_active){
		// Host mode
		for (uint8_t i = 0; i < devNum; i++){
			if (devPend & (1 << i)) {
    			uint8_t cableNum;
    			uint8_t buffer[48];
    			uint32_t bytesRead = 69;
				while(bytesRead){
					bytesRead = tuh_midi_stream_read(devAddr[i], &cableNum, buffer, sizeof(buffer));
					for (uint8_t j = 0; j < bytesRead; j++){
						MIDI_BS2UMP.bytestreamParse(buffer[j]);
						while(MIDI_BS2UMP.availableUMP()){
							uint32_t temp = MIDI_BS2UMP.readUMP();
							if (umpLen <= 0){
								uint8_t tp = get_ump_type(temp);
								umpLen = get_ump_size(tp);
								if ((tp == UMP_UTILITY)||(tp == UMP_MIDI_ENDPOINT)){
									// Groupless messages
									umpGroup = 16;
								} else {
									umpGroup = get_ump_group(temp);
								}
							}
							umpLen -= 4;
							canBuffer[umpGroup].Write(&temp);
						}
					}
					// TODO: send data to other devices
				}
				devPend &= ~(1 << i);
			}
		}
	} else {
		// Device mode
		uint8_t buffer[16];
		uint8_t bytesRead;
		while ( tud_midi_available() ) {
			bytesRead = tud_midi_stream_read(buffer, 16);
			for (uint8_t j = 0; j < bytesRead; j++){
				MIDI_BS2UMP.bytestreamParse(buffer[j]);
				while(MIDI_BS2UMP.availableUMP()){
					uint32_t temp = MIDI_BS2UMP.readUMP();
					if (umpLen <= 0){
						uint8_t tp = get_ump_type(temp);
						umpLen = get_ump_size(tp);
						if ((tp == UMP_UTILITY)||(tp == UMP_MIDI_ENDPOINT)){
							// Groupless messages
							umpGroup = 16;
						} else {
							umpGroup = get_ump_group(temp);
						}
					}
					umpLen -= 4;
					canBuffer[umpGroup].Write(&temp);
				}
			}
		}
	}
	// Write MIDI data to USB
	uint16_t buffLen = usbBuffer.Count();
	if (buffLen > 64){
		buffLen = 64;
	}
	// TODO: this may send incomplete messages
	if (buffLen){
		char buff[64];
		char umpBuff[64];
		for (uint8_t i = 0; i < buffLen; i++){
			usbBuffer.Read(&umpBuff[i]);
		}
		buffLen >>= 2;
		uint32_t length = 0;
		for (uint8_t i = 0; i < buffLen; i++){
			uint32_t temp;
			uint8_t ii = i*4;
			temp = (umpBuff[ii] << 24)|(umpBuff[ii+1] << 16)|(umpBuff[ii+2] << 8)|umpBuff[ii+3];
			MIDI_UMP2BS.UMPStreamParse(temp);
			while (MIDI_UMP2BS.availableBS()){
				buff[length++] = MIDI_UMP2BS.readBS();
			}
		}

		// TODO: What to do if data is not accepted by USB driver
		length = usb_midi_tx(buff, length);
	}
}

void dma1_irq_handler (){
	SPI_CAN.Handler();
}
