#ifndef _BOARD_M2IDI_USB_H
#define _BOARD_M2IDI_USB_H

// For board detection
#define BOARD_M2IDI_USB

#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

// --- LED ---
#ifndef LEDH
#define LEDH 27
#endif
#ifndef LEDD
#define LEDD 28
#endif

// no PICO_DEFAULT_WS2812_PIN

// --- BUTTON ---
#ifndef BUTTIO
#define BUTTIO 26
#endif

// --- USB ID ---
#ifndef USB_ID_PIN
#define USB_ID_PIN 29
#endif

// --- I2S ---
#ifndef I2S_FS
#define I2S_FS 5
#endif
#ifndef I2S_BCK
#define I2S_BCK 4
#endif
#ifndef I2S_MCK
#define I2S_MCK 3
#endif
#ifndef I2S_DO
#define I2S_DO 2
#endif
#ifndef I2S_DI
#define I2S_DI 21
#endif

// --- SPI ---
#ifndef M2IDI_CAN_SPI
#define M2IDI_CAN_SPI 1
#endif
#ifndef M2IDI_CAN_SPI_SCK_PIN
#define M2IDI_CAN_SPI_SCK_PIN 10
#endif
#ifndef M2IDI_CAN_SPI_TX_PIN
#define M2IDI_CAN_SPI_TX_PIN 11
#endif
#ifndef M2IDI_CAN_SPI_RX_PIN
#define M2IDI_CAN_SPI_RX_PIN 8
#endif
#ifndef M2IDI_CAN_SPI_CSN_PIN
#define M2IDI_CAN_SPI_CSN_PIN 9
#endif
#ifndef M2IDI_CAN_INT_PIN
#define M2IDI_CAN_INT_PIN 12
#endif

#ifndef M2IDI_MEM_SPI
#define M2IDI_MEM_SPI 0
#endif
#ifndef M2IDI_MEM_SPI_SCK_PIN
#define M2IDI_MEM_SPI_SCK_PIN 18
#endif
#ifndef M2IDI_MEM_SPI_TX_PIN
#define M2IDI_MEM_SPI_TX_PIN 19
#endif
#ifndef M2IDI_MEM_SPI_RX_PIN
#define M2IDI_MEM_SPI_RX_PIN 16
#endif
#ifndef M2IDI_MEM_SPI_CSN_RAM_PIN
#define M2IDI_MEM_SPI_CSN_RAM_PIN 20
#endif
#ifndef M2IDI_MEM_SPI_CSN_EEPROM_PIN
#define M2IDI_MEM_SPI_CSN_EEPROM_PIN 17
#endif

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_IS25LP080 1
#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 0

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
#endif

#endif
