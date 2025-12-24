#pragma once

// SPI wiring for SD card (SDSPI mode)
#define WIMILL_PIN_SD_CS   13
#define WIMILL_PIN_SD_MOSI 16
#define WIMILL_PIN_SD_SCK  15
#define WIMILL_PIN_SD_MISO 17

// Default UART console speed
#define WIMILL_UART_BAUD 115200

// Setup mode button (active-low)
#define WIMILL_PIN_SETUP_BTN 21

// Mount point for the FATFS
#define WIMILL_SD_MOUNT_POINT "/sdcard"

// SD SPI frequency control (kHz)
#define WIMILL_SD_FREQ_KHZ_DEFAULT 20000
#define WIMILL_SD_FREQ_KHZ_20MHZ    20000
#define WIMILL_SD_FREQ_KHZ_26MHZ    26000

// SD self-test buffer size (bytes)
#define WIMILL_SDTEST_BUF_SZ 16384

// RGB LED (WS2812/NeoPixel) on GPIO48 (optional, fail-safe if absent)
#define WIMILL_RGB_GPIO 48
#define WIMILL_RGB_COUNT 1
