#include <usb.h>
#include <tusb.h>
#include "usb_midi_host.h"
#include "pico/stdlib.h"

uint32_t blinkTime = 0;
uint32_t blinkTime2 = 0;
uint8_t devAddr[CFG_TUH_DEVICE_MAX];
int8_t devNum = 0;
uint16_t devPend = 0;

// Initialize clocks and pins for the USB port
void USB_Init(){
	
	// Enable USB ID pin?

	
	blinkTime = 500000;
	tusb_init();
}

void USB_Handler(void){
	//tud_int_handler(0);
	
	tuh_int_handler(0);
}

uint32_t usb_midi_tx(char data[], uint32_t length){
	// TODO: device mode
	//tud_midi_stream_write(0, (uint8_t*)(tempData), length);
	
	// Host mode
	if (devNum <= 0){
		return length;
	}
	
	// TODO: will this work with multiple devices?
	for (int8_t i = 0; i < devNum; i++){
		length = tuh_midi_stream_write(devAddr[i], /* TODO cable num */ 0, (uint8_t*) data, length);
		tuh_midi_stream_flush(devAddr[i]);
	}
	return length;
}

//--------------------------------------------------------------------+
// Host callbacks
//--------------------------------------------------------------------+

void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx){
    (void ) in_ep;
    (void ) out_ep;
    (void ) num_cables_rx;
    (void ) num_cables_tx;

    TU_LOG1("MIDI device address = %u, IN endpoint %u has %u cables, OUT endpoint %u has %u cables\r\n",
           dev_addr, in_ep & 0xf, num_cables_rx, out_ep & 0xf, num_cables_tx);

	if (devNum < CFG_TUH_DEVICE_MAX) {
    	devAddr[devNum++] = dev_addr;
	}

	blinkTime = 100000;
}

// Invoked when device with midi interface is un-mounted
void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance){
    (void ) dev_addr;
    (void ) instance;

    TU_LOG1("MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
    uint8_t i = 0;
	if (devNum <= 0) return;
	// Find address
	for(; i < devNum; i++){
		if (devAddr[i] == dev_addr){
			devNum--;
			break;
		}
	}
	// Remove address
	for (; i <= devNum; ++i){
		devAddr[i] = devAddr[i-1];
	}

	blinkTime = 500000;
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets){
	int devIndex = -1;
	for (int8_t i = 0; i < devNum; i++){
		if (dev_addr == devAddr[i]) {
			devIndex = i;
		}
	}
	
    if (devIndex < 0){
        return;
    }

    if(num_packets == 0){
        return;
    }

	devPend |= 1 << devIndex;
}

void tuh_midi_tx_cb(uint8_t dev_addr){
    (void ) dev_addr;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void){
	blinkTime = 100000;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en){
	(void) remote_wakeup_en;
	blinkTime = 200000;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void){
	blinkTime = 100000;
}
