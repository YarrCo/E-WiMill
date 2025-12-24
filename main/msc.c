#include "msc.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tusb.h" // Main TinyUSB header

#include "led_status.h"
#include "sdcard.h"
#include "wimill_pins.h" // Убедитесь, что этот файл существует и доступен

#define TAG "MSC"
#define MSC_SECTOR_SIZE 512
#define MSC_DETACH_DELAY_MS 500
#define MSC_READAHEAD_SECTORS 8
#ifndef MSC_USE_RAMDISK
#define MSC_USE_RAMDISK 1
#endif
#define MSC_RAMDISK_SIZE (1024 * 1024)

// --- USB DESCRIPTORS (MANUAL) ---
#define USB_VID 0x303A
#define USB_PID 0x4002
#define EPNUM_MSC_OUT 0x01
#define EPNUM_MSC_IN 0x81
// #define TUD_MSC_DESC_LEN (9 + 9 + 7 + 7)
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)
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
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(0, 0, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

// String Descriptors
static const char *desc_strings[] = {
    (const char[]){0x09, 0x04}, // 0: English (0x0409)
    "Espressif",                // 1: Manufacturer
    "WiMill Disk",              // 2: Product
    "RAMDISK001",               // 3: Serial
};
// --- END DESCRIPTORS ---

typedef struct
{
    bool valid;
    bool dirty;
    uint32_t lba;
    uint8_t data[MSC_SECTOR_SIZE] __attribute__((aligned(4)));
} sector_cache_t;

typedef struct
{
    bool valid;
    uint32_t lba;
    uint32_t count;
    uint8_t data[MSC_READAHEAD_SECTORS * MSC_SECTOR_SIZE] __attribute__((aligned(4)));
} read_ahead_cache_t;

static sdmmc_card_t *s_card = NULL;
static uint32_t s_block_size = MSC_SECTOR_SIZE;
static uint32_t s_block_count = 0;
static uint8_t *s_ramdisk = NULL;
static size_t s_ramdisk_size = MSC_RAMDISK_SIZE;
static bool s_ramdisk_ready = false;
static bool s_usb_installed = false;
static bool s_usb_connected = false;
static bool s_media_present = false;
static bool s_unit_attention = false;
static msc_state_t s_state = MSC_STATE_USB_DETACHED;
static sector_cache_t s_cache = {0};
static read_ahead_cache_t s_read_ahead = {0};

static inline void lock_io(void)
{
    sdcard_lock();
}

static inline void unlock_io(void)
{
    sdcard_unlock();
}

static void write_le16(uint8_t *buf, size_t off, uint16_t val)
{
    buf[off] = (uint8_t)(val & 0xFF);
    buf[off + 1] = (uint8_t)((val >> 8) & 0xFF);
}

static void write_le32(uint8_t *buf, size_t off, uint32_t val)
{
    buf[off] = (uint8_t)(val & 0xFF);
    buf[off + 1] = (uint8_t)((val >> 8) & 0xFF);
    buf[off + 2] = (uint8_t)((val >> 16) & 0xFF);
    buf[off + 3] = (uint8_t)((val >> 24) & 0xFF);
}

static void write_be32(uint8_t *buf, size_t off, uint32_t val)
{
    buf[off] = (uint8_t)((val >> 24) & 0xFF);
    buf[off + 1] = (uint8_t)((val >> 16) & 0xFF);
    buf[off + 2] = (uint8_t)((val >> 8) & 0xFF);
    buf[off + 3] = (uint8_t)(val & 0xFF);
}

static void write_be24(uint8_t *buf, size_t off, uint32_t val)
{
    buf[off] = (uint8_t)((val >> 16) & 0xFF);
    buf[off + 1] = (uint8_t)((val >> 8) & 0xFF);
    buf[off + 2] = (uint8_t)(val & 0xFF);
}

static void ramdisk_format(void)
{
    memset(s_ramdisk, 0, s_ramdisk_size);
    uint8_t *bs = s_ramdisk;
    bs[0] = 0xEB;
    bs[1] = 0x3C;
    bs[2] = 0x90;
    memcpy(&bs[3], "MSDOS5.0", 8);
    write_le16(bs, 0x0B, 512);
    bs[0x0D] = 1;
    write_le16(bs, 0x0E, 1);
    bs[0x10] = 2;
    write_le16(bs, 0x11, 128);
    write_le16(bs, 0x13, (uint16_t)(s_ramdisk_size / 512));
    bs[0x15] = 0xF8;
    write_le16(bs, 0x16, 6);
    write_le16(bs, 0x18, 32);
    write_le16(bs, 0x1A, 64);
    write_le32(bs, 0x1C, 0);
    write_le32(bs, 0x20, 0);
    bs[0x24] = 0x80;
    bs[0x25] = 0x00;
    bs[0x26] = 0x29;
    write_le32(bs, 0x27, 0x1234ABCD);
    memcpy(&bs[0x2B], "RAMDISK   ", 11);
    memcpy(&bs[0x36], "FAT12   ", 8);
    bs[0x1FE] = 0x55;
    bs[0x1FF] = 0xAA;

    size_t fat1_off = 1 * 512;
    size_t fat2_off = (1 + 6) * 512;
    s_ramdisk[fat1_off + 0] = 0xF8;
    s_ramdisk[fat1_off + 1] = 0xFF;
    s_ramdisk[fat1_off + 2] = 0xFF;
    s_ramdisk[fat2_off + 0] = 0xF8;
    s_ramdisk[fat2_off + 1] = 0xFF;
    s_ramdisk[fat2_off + 2] = 0xFF;

    size_t root_off = (1 + 6 + 6) * 512;
    memcpy(&s_ramdisk[root_off], "RAMDISK   ", 11);
    s_ramdisk[root_off + 11] = 0x08;
}

static esp_err_t ramdisk_init(void)
{
    if (s_ramdisk_ready && s_ramdisk)
    {
        return ESP_OK;
    }
    if (!s_ramdisk)
    {
        uint8_t *buf = heap_caps_malloc(s_ramdisk_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!buf)
        {
            buf = heap_caps_malloc(s_ramdisk_size, MALLOC_CAP_DEFAULT);
        }
        if (!buf)
        {
            return ESP_ERR_NO_MEM;
        }
        s_ramdisk = buf;
    }
    ramdisk_format();
    s_ramdisk_ready = true;
    return ESP_OK;
}

static bool storage_ready(void)
{
#if MSC_USE_RAMDISK
    return s_ramdisk_ready && s_ramdisk;
#else
    return s_card != NULL;
#endif
}

static bool media_ready(void)
{
    return s_media_present && storage_ready();
}

static esp_err_t msc_read_sectors(uint32_t lba, void *buffer, uint32_t count)
{
#if MSC_USE_RAMDISK
    if (!s_ramdisk || !buffer)
    {
        return ESP_ERR_INVALID_STATE;
    }
    size_t offset = (size_t)lba * s_block_size;
    size_t len = (size_t)count * s_block_size;
    if (offset + len > s_ramdisk_size)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(buffer, s_ramdisk + offset, len);
    return ESP_OK;
#else
    return sdmmc_read_sectors(s_card, buffer, lba, count);
#endif
}

static esp_err_t msc_write_sectors(uint32_t lba, const void *buffer, uint32_t count)
{
#if MSC_USE_RAMDISK
    if (!s_ramdisk || !buffer)
    {
        return ESP_ERR_INVALID_STATE;
    }
    size_t offset = (size_t)lba * s_block_size;
    size_t len = (size_t)count * s_block_size;
    if (offset + len > s_ramdisk_size)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(s_ramdisk + offset, buffer, len);
    return ESP_OK;
#else
    return sdmmc_write_sectors(s_card, buffer, lba, count);
#endif
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
    esp_err_t ret = msc_write_sectors(s_cache.lba, s_cache.data, 1);
    if (ret == ESP_OK)
    {
        s_cache.dirty = false;
        if (s_read_ahead.valid)
        {
            uint32_t ra_end = s_read_ahead.lba + s_read_ahead.count;
            if (s_cache.lba >= s_read_ahead.lba && s_cache.lba < ra_end)
                s_read_ahead.valid = false;
        }
    }
    return ret;
}

static esp_err_t load_cache_locked(uint32_t lba)
{
    if (s_cache.valid && s_cache.lba == lba)
        return ESP_OK;
    ESP_RETURN_ON_ERROR(flush_cache_locked(), TAG, "flush failed");
    esp_err_t ret = msc_read_sectors(lba, s_cache.data, 1);
    if (ret != ESP_OK)
        return ret;
    s_cache.valid = true;
    s_cache.dirty = false;
    s_cache.lba = lba;
    return ESP_OK;
}

static esp_err_t msc_read_partial(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    if (!storage_ready())
        return ESP_ERR_INVALID_STATE;
    if ((offset + bufsize) > s_block_size)
        return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(load_cache_locked(lba), TAG, "load failed");
    memcpy(buffer, s_cache.data + offset, bufsize);
    return ESP_OK;
}

static esp_err_t msc_write_partial(uint32_t lba, uint32_t offset, const uint8_t *buffer, uint32_t bufsize)
{
    if (!storage_ready())
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
    if (s_read_ahead.valid)
    {
        uint32_t ra_end = s_read_ahead.lba + s_read_ahead.count;
        if (lba >= s_read_ahead.lba && lba < ra_end)
            s_read_ahead.valid = false;
    }
    return ESP_OK;
}

static esp_err_t msc_enable(void)
{
// Инициализация карты
#if MSC_USE_RAMDISK
    ESP_RETURN_ON_ERROR(ramdisk_init(), TAG, "ramdisk init failed");
    s_block_size = MSC_SECTOR_SIZE;
    s_block_count = (uint32_t)(s_ramdisk_size / s_block_size);
    s_card = NULL;
#else
    ESP_RETURN_ON_ERROR(sdcard_init_raw(&s_card), TAG, "sd init failed");

    // !!! ИСПРАВЛЕНИЕ 1: Корректный расчет размера !!!
    s_block_size = s_card->csd.sector_size;
    s_block_count = s_card->csd.capacity;
#endif

    memset(&s_cache, 0, sizeof(s_cache));
    memset(&s_read_ahead, 0, sizeof(s_read_ahead));

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

    if (!s_usb_installed)
    {
        ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "tinyusb init failed");
        s_usb_installed = true;
    }

    if (!s_usb_connected)
    {
        tud_connect();
        s_usb_connected = true;
    }
#if MSC_USE_RAMDISK
    ESP_LOGI(TAG, "MSC RAM disk ready: %u KB", (unsigned)(s_ramdisk_size / 1024));
#endif
    return ESP_OK;
}

static void msc_disable(void)
{
    if (!s_usb_installed)
        return;
    lock_io();
    flush_cache_locked();
    s_cache.valid = false;
    s_read_ahead.valid = false;
    unlock_io();
    // Keep USB connected, only hide media.
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
    s_media_present = true;
    s_unit_attention = false;
    set_state(MSC_STATE_USB_ATTACHED);
    ESP_LOGI(TAG, "MSC initialized. Blocks: %lu", (unsigned long)s_block_count);
    return ESP_OK;
}

msc_state_t msc_get_state(void) { return s_state; }

esp_err_t msc_attach(void)
{
    if (s_state == MSC_STATE_USB_ATTACHED)
        return ESP_OK;

    ESP_LOGI(TAG, "usb attach");

    // Сначала отмонтируем VFS, если занято
    if (sdcard_is_mounted())
    {
        if (sdcard_unmount() != ESP_OK)
            return ESP_FAIL;
    }

    sdcard_set_mode(SDCARD_MODE_USB);
    if (msc_enable() != ESP_OK)
        return ESP_FAIL;

    s_media_present = true;
    s_unit_attention = true;
    set_state(MSC_STATE_USB_ATTACHED);
    return ESP_OK;
}

esp_err_t msc_detach(void)
{
    if (s_state == MSC_STATE_USB_DETACHED)
        return ESP_OK;

    ESP_LOGI(TAG, "usb detach");

    msc_disable();
    s_media_present = false;
    s_unit_attention = false;
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
    if (!s_media_present) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return false;
    }
    if (s_unit_attention) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
        s_unit_attention = false;
        return false;
    }
    return storage_ready();
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    if (!media_ready()) {
        *block_count = 0;
        *block_size = MSC_SECTOR_SIZE;
        return;
    }
    *block_count = s_block_count; // Берем исправленное значение
    *block_size = (uint16_t)s_block_size;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;
    if (!s_media_present) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return -1;
    }
    if (s_unit_attention) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
        s_unit_attention = false;
        return -1;
    }
    if (!storage_ready())
        return -1;
    lock_io();
    esp_err_t ret = ESP_OK;

    // Fast Path (Optimized)
    if (offset == 0 && (bufsize % s_block_size) == 0)
    {
        if (s_cache.valid && s_cache.dirty)
            flush_cache_locked();
        uint32_t sectors = bufsize / s_block_size;
        if (s_block_count > 0 && (lba + sectors) > s_block_count)
        {
            ret = ESP_ERR_INVALID_SIZE;
        }
        else
        {
            bool served = false;
            if (s_read_ahead.valid)
            {
                uint32_t ra_end = s_read_ahead.lba + s_read_ahead.count;
                if (lba >= s_read_ahead.lba && (lba + sectors) <= ra_end)
                {
                    uint32_t offset_sectors = lba - s_read_ahead.lba;
                    memcpy(buffer, s_read_ahead.data + (offset_sectors * s_block_size), bufsize);
                    served = true;
                    ret = ESP_OK;
                }
            }

            if (!served)
            {
                if (sectors <= MSC_READAHEAD_SECTORS)
                {
                    uint32_t ra_sectors = MSC_READAHEAD_SECTORS;
                    if (s_block_count > 0)
                    {
                        uint32_t remaining = s_block_count - lba;
                        if (remaining < ra_sectors)
                            ra_sectors = remaining;
                    }
                    if (ra_sectors == 0 || ra_sectors < sectors)
                    {
                        ret = ESP_ERR_INVALID_SIZE;
                    }
                    else
                    {
                        ret = msc_read_sectors(lba, s_read_ahead.data, ra_sectors);
                        if (ret == ESP_OK)
                        {
                            s_read_ahead.valid = true;
                            s_read_ahead.lba = lba;
                            s_read_ahead.count = ra_sectors;
                            memcpy(buffer, s_read_ahead.data, bufsize);
                        }
                    }
                }
                else
                {
                    ret = msc_read_sectors(lba, buffer, sectors);
                }
            }
        }
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
    if (!s_media_present) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return -1;
    }
    if (s_unit_attention) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
        s_unit_attention = false;
        return -1;
    }
    if (!storage_ready())
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
        if (s_read_ahead.valid)
        {
            uint32_t ra_end = s_read_ahead.lba + s_read_ahead.count;
            uint32_t write_end = lba + sectors;
            if (lba < ra_end && write_end > s_read_ahead.lba)
                s_read_ahead.valid = false;
        }
        ret = msc_write_sectors(lba, buffer, sectors);
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
    if (!s_media_present) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return -1;
    }
    if (s_unit_attention) {
        tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
        s_unit_attention = false;
        return -1;
    }
    switch (scsi_cmd[0])
    {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
    case 0x35: // SYNCHRONIZE_CACHE_10
        lock_io();
        flush_cache_locked();
        unlock_io();
        return 0;
    case SCSI_CMD_READ_FORMAT_CAPACITIES:
        if (!media_ready()) {
            tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
            return -1;
        }
        if (bufsize < 12) {
            return -1;
        }
        memset(buffer, 0, 12);
        // Capacity list length (8 bytes)
        write_be32((uint8_t *)buffer, 0, 8);
        // Number of blocks
        write_be32((uint8_t *)buffer, 4, s_block_count);
        // Descriptor type: formatted media
        ((uint8_t *)buffer)[8] = 0x02;
        // Block length (3 bytes)
        write_be24((uint8_t *)buffer, 9, s_block_size);
        return 12;
    case SCSI_CMD_MODE_SENSE_6:
        if (!media_ready()) {
            tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
            return -1;
        }
        if (bufsize < 4) {
            return -1;
        }
        memset(buffer, 0, 4);
        ((uint8_t *)buffer)[0] = 3; // Mode data length
        return 4;
    case SCSI_CMD_MODE_SENSE_10:
        if (!media_ready()) {
            tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
            return -1;
        }
        if (bufsize < 8) {
            return -1;
        }
        memset(buffer, 0, 8);
        ((uint8_t *)buffer)[1] = 6; // Mode data length
        return 8;
    case SCSI_CMD_TEST_UNIT_READY:
        return media_ready() ? 0 : -1;
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
