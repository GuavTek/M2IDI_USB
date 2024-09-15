/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jerzy Kasenberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "usb_descriptors.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]     AUDIO | MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
    _PID_MAP(MIDI, 3) | _PID_MAP(AUDIO, 4) | _PID_MAP(VENDOR, 5) )

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,		// 0x00 for each interface defines its own
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
#define CONFIG_TOTAL_LEN    	(TUD_CONFIG_DESC_LEN + TUD_AUDIO_HEADSET_STEREO_DESC_LEN + TUD_MIDI_DESC_LEN)

#define EPNUM_AUDIO_IN    0x01
#define EPNUM_AUDIO_OUT   0x01
#define EPNUM_MIDI_OUT   0x03
#define EPNUM_MIDI_IN   0x03

uint8_t const desc_configuration[] =
{
    // Interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, 0),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(2, EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN | 0x80),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, ITF_NUM_TOTAL, EPNUM_MIDI_OUT, (0x80 | EPNUM_MIDI_IN), 64)
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void)index; // for multiple configurations
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] =
{
	(const char[]) { 0x09, 0x04 },  // 0: is supported language is English (0x0409)
	"GuavTek",                      // 1: Manufacturer
	"M2IDI Eurorack interface",              // 2: Product
	NULL,                       // 3: Serials, will use unique ID
	"Eurorack Input",             // 4: Audio Interface
	"Eurorack Output",           // 5: Audio Interface
	"Eurorack MIDI",				// 6: MIDI Interface
};

static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void)langid;

  uint8_t chr_count;

  if (index == 0){
    // Language ID
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else if (index == 3){
    pico_unique_board_id_t pico_id;
    pico_get_unique_board_id(&pico_id);
    for (uint8_t i = 0; i < 8; i++){
      for (uint8_t j = 0; j < 2; j++){
        uint8_t jj = (i << 1) + j;
        const char nibble2hex[16] = {
            '0', '1', '2', '3', '4', '5', '6', '7',
            '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
        };
        uint8_t nibble = (pico_id.id[i] >> (4*j)) & 0xf;
        _desc_str[1+jj] = nibble2hex[nibble];
      }
    }
    chr_count = 16;
  } else {
    // Convert ASCII string into UTF-16

    if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0]))) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
    if ( chr_count > max_count ) chr_count = max_count;

    for (uint8_t i = 0; i < chr_count; i++){
      _desc_str[1 + i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2 * chr_count + 2);

  return _desc_str;
}
