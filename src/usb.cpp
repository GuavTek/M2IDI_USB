#include <usb.h>
#include <tusb.h>
#include "usb_midi_host.h"
#include "pico/stdlib.h"

uint32_t blinkTime = 0;
uint32_t blinkTime2 = 0;
uint8_t devAddr[CFG_TUH_DEVICE_MAX];
int8_t devNum = 0;
volatile uint32_t system_ticks = 0;

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
	blinkTime = 500000;
	tusb_init();
}

void SysTick_Handler (void){
	system_ticks++;
}

uint32_t board_millis(void){
	return system_ticks;
}

void USB_Handler(void){
	//tud_int_handler(0);
	
	tuh_int_handler(0);
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

	blinkTime = 50000;
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
	for (size_t i = 0; i < devNum; i++){
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

    uint8_t cable_num;
    uint8_t buffer[48];
    uint32_t bytes_read = tuh_midi_stream_read(dev_addr, &cable_num, buffer, sizeof(buffer));
    (void ) bytes_read;

	static bool dstate = 0;
	gpio_put(LEDD, dstate);
	dstate = !dstate;

    TU_LOG1("Read bytes %lu cable %u", bytes_read, cable_num);
    TU_LOG1_MEM(buffer, bytes_read, 2);
}

void tuh_midi_tx_cb(uint8_t dev_addr){
    (void ) dev_addr;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void){
	blinkTime = 2000;
	blinkTime2 = 1998;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en){
	(void) remote_wakeup_en;
	blinkTime = 100;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void){
	blinkTime = 1000;
}
