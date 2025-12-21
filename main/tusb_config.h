// Minimal TinyUSB configuration for ESP32-S3 device MSC
#pragma once

#include "sdkconfig.h"
#include "tusb_option.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_ESP32S3
#endif

#define CFG_TUSB_OS                 OPT_OS_FREERTOS
#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_MEM_SECTION        TU_ATTR_ALIGNED(4)
#define CFG_TUSB_MEM_ALIGN          TU_ATTR_ALIGNED(4)

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG              CONFIG_TINYUSB_DEBUG_LEVEL
#endif

#define CFG_TUD_ENDPOINT0_SIZE      64

#define CFG_TUD_CDC                 0
#define CFG_TUD_MSC                 1
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0
#define CFG_TUD_NET                 0
#define CFG_TUD_BTH                 0
#define CFG_TUD_DFU_RT              0

#define CFG_TUD_MSC_EP_BUFSIZE      512
#define CFG_TUD_MSC_BUFSIZE         CONFIG_TINYUSB_MSC_BUFSIZE
#define CFG_TUD_MSC_MAXLUN          1

#ifdef __cplusplus
}
#endif
