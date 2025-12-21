#pragma once

// SPI wiring for SD card (SDSPI mode)
#define WIMILL_PIN_SD_CS   10
#define WIMILL_PIN_SD_MOSI 11
#define WIMILL_PIN_SD_SCK  12
#define WIMILL_PIN_SD_MISO 13

// Default UART console speed
#define WIMILL_UART_BAUD 115200

// Mount point for the FATFS
#define WIMILL_SD_MOUNT_POINT "/sdcard"

// SD SPI frequency control (kHz)
#define WIMILL_SD_FREQ_KHZ_DEFAULT 10000
#define WIMILL_SD_FREQ_KHZ_4MHZ     4000
#define WIMILL_SD_FREQ_KHZ_1MHZ     1000
