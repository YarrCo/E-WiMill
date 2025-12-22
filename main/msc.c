#include "msc.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tusb.h"             // Main TinyUSB header
#include "tusb_msc_storage.h" // Helpers for MSC

#include "led_status.h"
#include "sdcard.h"
#include "wimill_pins.h" // Убедитесь, что этот файл существует и доступен

#define TAG "MSC"
#define MSC_SECTOR_SIZE 512
#define MSC_DETACH_DELAY_MS 500

// --- USB DESCRIPTORS (MANUAL) ---
#define USB_VID 0x303A
#define USB_PID 0x4002
#define EPNUM_MSC_OUT 0x01
#define EPNUM_MSC_IN 0x81
#define TUD_MSC_DESC_LEN (9 + 9 + 7 + 7)

// Device Descriptor
static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01};

// Configuration Descriptor
static const uint8_t desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_MSC_DESC_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(0, 0, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

// String Descriptors
static const char *desc_strings[] = {
    (const char[]){0x09, 0x04}, // 0: English (0x0409)
    "Espressif",                // 1: Manufacturer
    "WiMill Disk",              // 2: Product
    "123456",                   // 3: Serial
};
// --- END DESCRIPTORS ---

typedef struct
{
    bool valid;
    bool dirty;
    uint32_t lba;
    uint8_t data[MSC_SECTOR_SIZE] __attribute__((aligned(4)));
} sector_cache_t;

static sdmmc_card_t *s_card = NULL;
static uint32_t s_block_size = MSC_SECTOR_SIZE;
static uint32_t s_block_count = 0;
static bool s_usb_enabled = false;
static msc_state_t s_state = MSC_STATE_USB_DETACHED;
static sector_cache_t s_cache = {0};

static inline void lock_io(void)
{
    sdcard_lock();
}

static inline void unlock_io(void)
{
    sdcard_unlock();
}

static void set_state(msc_state_t st)
{
    if (s_state == st)
        return;
    s_state = st;
    switch (s_state)
    {
    case MSC_STATE_USB_ATTACHED:
        led_status_set(LED_STATE_USB_ATTACHED);
        break;
    case MSC_STATE_USB_DETACHED:
        led_status_set(LED_STATE_USB_DETACHED);
        break;
    default:
        led_status_set(LED_STATE_ERROR);
        break;
    }
}

static esp_err_t flush_cache_locked(void)
{
    if (!s_cache.valid || !s_cache.dirty)
        return ESP_OK;
    esp_err_t ret = sdmmc_write_sectors(s_card, s_cache.data, s_cache.lba, 1);
    if (ret == ESP_OK)
        s_cache.dirty = false;
    return ret;
}

static esp_err_t load_cache_locked(uint32_t lba)
{
    if (s_cache.valid && s_cache.lba == lba)
        return ESP_OK;
    ESP_RETURN_ON_ERROR(flush_cache_locked(), TAG, "flush failed");
    esp_err_t ret = sdmmc_read_sectors(s_card, s_cache.data, lba, 1);
    if (ret != ESP_OK)
        return ret;
    s_cache.valid = true;
    s_cache.dirty = false;
    s_cache.lba = lba;
    return ESP_OK;
}

static esp_err_t msc_read_partial(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    if (!s_card)
        return ESP_ERR_INVALID_STATE;
    if ((offset + bufsize) > s_block_size)
        return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(load_cache_locked(lba), TAG, "load failed");
    memcpy(buffer, s_cache.data + offset, bufsize);
    return ESP_OK;
}

static esp_err_t msc_write_partial(uint32_t lba, uint32_t offset, const uint8_t *buffer, uint32_t bufsize)
{
    if (!s_card)
        return ESP_ERR_INVALID_STATE;
    if ((offset + bufsize) > s_block_size)
        return ESP_ERR_INVALID_ARG;
    if (s_cache.valid && s_cache.dirty && s_cache.lba != lba)
    {
        ESP_RETURN_ON_ERROR(flush_cache_locked(), TAG, "flush failed");
    }
    ESP_RETURN_ON_ERROR(load_cache_locked(lba), TAG, "load failed");
    memcpy(s_cache.data + offset, buffer, bufsize);
    s_cache.dirty = true;
    return ESP_OK;
}

static esp_err_t msc_enable(void)
{
    if (s_usb_enabled)
        return ESP_OK;

    // Инициализация карты
    ESP_RETURN_ON_ERROR(sdcard_init_raw(&s_card), TAG, "sd init failed");

    // !!! ИСПРАВЛЕНИЕ 1: Корректный расчет размера !!!
    s_block_size = s_card->csd.sector_size;
    s_block_count = s_card->csd.capacity; // Без умножения!

    memset(&s_cache, 0, sizeof(s_cache));

    // !!! ИСПРАВЛЕНИЕ 2: Ручные дескрипторы !!!
    tinyusb_config_t tusb_cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = {
            .skip_setup = false,
            .self_powered = false,
            .vbus_monitor_io = -1,
        },
        .task = {
            .size = 8192,
            .priority = 5,
#if CONFIG_FREERTOS_UNICORE
            .xCoreID = 0,
#else
            .xCoreID = 1,
#endif
        },
        .descriptor = {
            .device = &desc_device,
            .qualifier = NULL,
            .string = desc_strings,
            .string_count = sizeof(desc_strings) / sizeof(desc_strings[0]),
            .full_speed_config = desc_configuration,
            .high_speed_config = NULL,
        },
        .event_cb = NULL,
        .event_arg = NULL,
    };

    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "tinyusb init failed");

    tud_connect();
    s_usb_enabled = true;
    return ESP_OK;
}

static void msc_disable(void)
{
    if (!s_usb_enabled)
        return;
    lock_io();
    flush_cache_locked();
    s_cache.valid = false;
    unlock_io();

    tud_disconnect();
    // Даем время хосту понять отключение
    vTaskDelay(pdMS_TO_TICKS(100));
    tinyusb_driver_uninstall();
    s_usb_enabled = false;
    s_card = NULL;
}

esp_err_t msc_init(void)
{
    sdcard_set_mode(SDCARD_MODE_USB);
    esp_err_t ret = msc_enable();
    if (ret != ESP_OK)
    {
        set_state(MSC_STATE_ERROR);
        return ret;
    }
    set_state(MSC_STATE_USB_ATTACHED);
    ESP_LOGI(TAG, "MSC initialized. Blocks: %lu", (unsigned long)s_block_count);
    return ESP_OK;
}

msc_state_t msc_get_state(void) { return s_state; }

esp_err_t msc_attach(void)
{
    if (s_state == MSC_STATE_USB_ATTACHED)
        return ESP_OK;

    // Сначала отмонтируем VFS, если занято
    if (sdcard_is_mounted())
    {
        if (sdcard_unmount() != ESP_OK)
            return ESP_FAIL;
    }

    sdcard_set_mode(SDCARD_MODE_USB);
    if (msc_enable() != ESP_OK)
        return ESP_FAIL;

    set_state(MSC_STATE_USB_ATTACHED);
    return ESP_OK;
}

esp_err_t msc_detach(void)
{
    if (s_state == MSC_STATE_USB_DETACHED)
        return ESP_OK;

    msc_disable();
    sdcard_set_mode(SDCARD_MODE_APP);
    vTaskDelay(pdMS_TO_TICKS(MSC_DETACH_DELAY_MS));

    if (sdcard_mount() != ESP_OK)
    {
        set_state(MSC_STATE_ERROR);
        return ESP_FAIL;
    }
    set_state(MSC_STATE_USB_DETACHED);
    return ESP_OK;
}

// --- TinyUSB Callbacks ---

uint8_t tud_msc_get_maxlun_cb(void) { return 0; }

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    memcpy(vendor_id, "ESP32   ", 8);
    memcpy(product_id, "WiMill Disk     ", 16);
    memcpy(product_rev, "1.0 ", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    return (s_card != NULL);
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = s_block_count; // Берем исправленное значение
    *block_size = (uint16_t)s_block_size;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;
    if (!s_card)
        return -1;
    lock_io();
    esp_err_t ret = ESP_OK;

    // Fast Path (Optimized)
    if (offset == 0 && (bufsize % s_block_size) == 0)
    {
        if (s_cache.valid && s_cache.dirty)
            flush_cache_locked();
        uint32_t sectors = bufsize / s_block_size;
        ret = sdmmc_read_sectors(s_card, buffer, lba, sectors);
    }
    else
    {
        ret = msc_read_partial(lba, offset, buffer, bufsize);
    }
    unlock_io();

    if (ret != ESP_OK)
    {
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
        return -1;
    }
    return bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;
    if (!s_card)
        return -1;
    lock_io();
    esp_err_t ret = ESP_OK;

    // Fast Path (Optimized)
    if (offset == 0 && (bufsize % s_block_size) == 0)
    {
        if (s_cache.valid && s_cache.dirty)
            flush_cache_locked();
        uint32_t sectors = bufsize / s_block_size;
        // Инвалидация кэша, если пишем поверх него
        if (s_cache.valid && s_cache.lba >= lba && s_cache.lba < (lba + sectors))
        {
            s_cache.valid = false;
        }
        ret = sdmmc_write_sectors(s_card, buffer, lba, sectors);
    }
    else
    {
        ret = msc_write_partial(lba, offset, buffer, bufsize);
    }
    unlock_io();

    if (ret != ESP_OK)
    {
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
        return -1;
    }
    return bufsize;
}

void tud_msc_write10_complete_cb(uint8_t lun) { (void)lun; }

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void)lun;
    (void)power_condition;
    (void)load_eject;
    return start;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    (void)buffer;
    (void)bufsize;
    switch (scsi_cmd[0])
    {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
    case 0x35: // SYNCHRONIZE_CACHE_10
        lock_io();
        flush_cache_locked();
        unlock_io();
        return 0;
    case SCSI_CMD_TEST_UNIT_READY:
        return s_card ? 0 : -1;
    case SCSI_CMD_START_STOP_UNIT:
        return 0;
    default:
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
        return -1;
    }
}

bool tud_msc_flush_cb(uint8_t lun)
{
    (void)lun;
    lock_io();
    flush_cache_locked();
    unlock_io();
    return true;
}
