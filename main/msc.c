#include "msc.h"

#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

#include "led_status.h"
#include "sdcard.h"

#define TAG "MSC"
#define MSC_SECTOR_SIZE 512
#define MSC_DETACH_DELAY_MS 300

typedef struct {
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
    if (s_state == st) {
        return;
    }
    s_state = st;
    switch (s_state) {
    case MSC_STATE_USB_ATTACHED:
        led_status_set(LED_STATE_USB_ATTACHED);
        break;
    case MSC_STATE_USB_DETACHED:
        led_status_set(LED_STATE_USB_DETACHED);
        break;
    case MSC_STATE_ERROR:
    default:
        led_status_set(LED_STATE_ERROR);
        break;
    }
}

static esp_err_t flush_cache_locked(void)
{
    if (!s_cache.valid || !s_cache.dirty) {
        return ESP_OK;
    }
    esp_err_t ret = sdmmc_write_sectors(s_card, s_cache.data, s_cache.lba, 1);
    if (ret == ESP_OK) {
        s_cache.dirty = false;
    }
    return ret;
}

static esp_err_t load_cache_locked(uint32_t lba)
{
    if (s_cache.valid && s_cache.lba == lba) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(flush_cache_locked(), TAG, "flush failed before load");

    esp_err_t ret = sdmmc_read_sectors(s_card, s_cache.data, lba, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    s_cache.valid = true;
    s_cache.dirty = false;
    s_cache.lba = lba;
    return ESP_OK;
}

static esp_err_t msc_read_partial(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    if (!s_card) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((offset + bufsize) > s_block_size) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(load_cache_locked(lba), TAG, "load cache failed");
    memcpy(buffer, s_cache.data + offset, bufsize);
    return ESP_OK;
}

static esp_err_t msc_write_partial(uint32_t lba, uint32_t offset, const uint8_t *buffer, uint32_t bufsize)
{
    if (!s_card) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((offset + bufsize) > s_block_size) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_cache.valid && s_cache.dirty && s_cache.lba != lba) {
        ESP_RETURN_ON_ERROR(flush_cache_locked(), TAG, "flush before switching LBA failed");
    }
    ESP_RETURN_ON_ERROR(load_cache_locked(lba), TAG, "load cache failed");
    memcpy(s_cache.data + offset, buffer, bufsize);
    s_cache.dirty = true;
    return ESP_OK;
}

static esp_err_t msc_enable(void)
{
    if (s_usb_enabled) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(sdcard_init_raw(&s_card), TAG, "sd init failed");
    s_block_size = s_card->csd.sector_size ? s_card->csd.sector_size : MSC_SECTOR_SIZE;
    s_block_count = (s_card->csd.capacity * s_card->csd.sector_size) / s_block_size;
    memset(&s_cache, 0, sizeof(s_cache));

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "tinyusb init failed");

    tud_connect();
    s_usb_enabled = true;
    return ESP_OK;
}

static void msc_disable(void)
{
    if (!s_usb_enabled) {
        return;
    }
    lock_io();
    flush_cache_locked();
    s_cache.valid = false;
    s_cache.dirty = false;
    unlock_io();

    tud_disconnect();
    esp_err_t ret = tinyusb_driver_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "tinyusb uninstall failed: %s", esp_err_to_name(ret));
    }
    s_usb_enabled = false;
    s_card = NULL;
}

esp_err_t msc_init(void)
{
    sdcard_set_mode(SDCARD_MODE_USB);
    esp_err_t ret = msc_enable();
    if (ret != ESP_OK) {
        set_state(MSC_STATE_ERROR);
        return ret;
    }
    set_state(MSC_STATE_USB_ATTACHED);
    ESP_LOGI(TAG, "MSC ready: blocks=%u block_size=%u", (unsigned)s_block_count, (unsigned)s_block_size);
    return ESP_OK;
}

msc_state_t msc_get_state(void)
{
    return s_state;
}

esp_err_t msc_attach(void)
{
    if (s_state == MSC_STATE_USB_ATTACHED) {
        return ESP_OK;
    }

    if (sdcard_is_mounted()) {
        esp_err_t ret = sdcard_unmount();
        if (ret != ESP_OK) {
            set_state(MSC_STATE_ERROR);
            return ret;
        }
    }

    sdcard_set_mode(SDCARD_MODE_USB);
    esp_err_t ret = msc_enable();
    if (ret != ESP_OK) {
        set_state(MSC_STATE_ERROR);
        return ret;
    }
    set_state(MSC_STATE_USB_ATTACHED);
    return ESP_OK;
}

esp_err_t msc_detach(void)
{
    if (s_state == MSC_STATE_USB_DETACHED) {
        return ESP_OK;
    }

    msc_disable();
    sdcard_set_mode(SDCARD_MODE_APP);
    vTaskDelay(pdMS_TO_TICKS(MSC_DETACH_DELAY_MS));

    esp_err_t ret = sdcard_mount();
    if (ret != ESP_OK) {
        set_state(MSC_STATE_ERROR);
        return ret;
    }
    set_state(MSC_STATE_USB_DETACHED);
    return ESP_OK;
}

// ---------------- TinyUSB MSC callbacks ----------------

uint8_t tud_msc_get_maxlun_cb(void)
{
    return 1;
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    const char vid[] = "WIMILL";
    const char pid[] = "SDSPI MSC";
    const char rev[] = "0.1";
    memcpy(vendor_id, vid, sizeof(vid) - 1);
    memcpy(product_id, pid, sizeof(pid) - 1);
    memcpy(product_rev, rev, sizeof(rev) - 1);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    if (!s_card) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return false;
    }
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = 0;
    *block_size = (uint16_t)s_block_size;
    if (s_card) {
        *block_count = (s_card->csd.capacity * s_card->csd.sector_size) / s_block_size;
    }
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;
    if (!s_card) {
        return -1;
    }

    lock_io();
    esp_err_t ret;

    if (offset == 0 && (bufsize % s_block_size) == 0) {
        if (s_cache.valid && s_cache.dirty) {
            ret = flush_cache_locked();
            if (ret != ESP_OK) {
                goto read_err;
            }
        }
        uint32_t sectors = bufsize / s_block_size;
        ret = sdmmc_read_sectors(s_card, buffer, lba, sectors);
    } else {
        ret = msc_read_partial(lba, offset, (uint8_t *)buffer, bufsize);
    }

    unlock_io();
    if (ret != ESP_OK) {
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
        return -1;
    }
    return (int32_t)bufsize;

read_err:
    unlock_io();
    tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
    return -1;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;
    if (!s_card) {
        return -1;
    }

    lock_io();
    esp_err_t ret;

    if (offset == 0 && (bufsize % s_block_size) == 0) {
        if (s_cache.valid && s_cache.dirty) {
            ret = flush_cache_locked();
            if (ret != ESP_OK) {
                goto write_err;
            }
        }
        uint32_t sectors = bufsize / s_block_size;
        ret = sdmmc_write_sectors(s_card, buffer, lba, sectors);
        if (s_cache.valid && (s_cache.lba >= lba && s_cache.lba < (lba + sectors))) {
            s_cache.valid = false;
            s_cache.dirty = false;
        }
    } else {
        ret = msc_write_partial(lba, offset, buffer, bufsize);
    }

    unlock_io();
    if (ret != ESP_OK) {
        tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
        return -1;
    }
    return (int32_t)bufsize;

write_err:
    unlock_io();
    tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x03, 0x00);
    return -1;
}

void tud_msc_write10_complete_cb(uint8_t lun)
{
    (void)lun;
}

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

    switch (scsi_cmd[0]) {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
    case 0x35: // SYNCHRONIZE_CACHE_10
        lock_io();
        flush_cache_locked();
        unlock_io();
        return 0;
    case SCSI_CMD_TEST_UNIT_READY:
        if (s_card) {
            return 0;
        }
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return -1;
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
    esp_err_t ret = flush_cache_locked();
    unlock_io();
    return ret == ESP_OK;
}
