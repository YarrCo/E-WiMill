// TinyUSB configuration for ESP32-S3 MSC device
#pragma once

#include "sdkconfig.h"
#include "tusb_option.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_TUSB_MCU OPT_MCU_ESP32S3
#define CFG_TUSB_OS OPT_OS_FREERTOS
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 1
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_ECM_RNDIS 0

#define CFG_TUD_MSC_EP_BUFSIZE 64
#define CFG_TUD_MSC_MAXLUN 1
#define CFG_TUD_MSC_BUFSIZE CONFIG_TINYUSB_MSC_BUFSIZE

#ifdef __cplusplus
}
#endif
