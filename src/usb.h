#ifndef _USB_H_
#define _USB_H_

#include <stdio.h>
#include "tusb_config.h"

#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t blinkTime;
extern uint32_t blinkTime2;
extern uint8_t devAddr[CFG_TUH_DEVICE_MAX];
extern int8_t devNum;
extern volatile uint32_t system_ticks;

// Initialize clocks and pins for the USB port
void USB_Init();
	
void USB_Handler(void);

//--------------------------------------------------------------------+
// Host callbacks
//--------------------------------------------------------------------+

void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx);

// Invoked when device with midi interface is un-mounted
void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance);

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets);

void tuh_midi_tx_cb(uint8_t dev_addr);

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void);

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en);

// Invoked when usb bus is resumed
void tud_resume_cb(void);

#ifdef __cplusplus
}
#endif

#endif
