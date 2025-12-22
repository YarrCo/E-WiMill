#include "msc.h"

#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

#include "led_status.h"
#include "sdcard.h"

#define TAG "MSC"
#define MSC_SECTOR_SIZE 512

typedef struct {
    bool valid;
    bool dirty;
    uint32_t lba;
    uint8_t data[MSC_SECTOR_SIZE] __attribute__((aligned(4)));
} sector_cache_t;

static sdmmc_card_t *s_card = NULL;
static uint32_t s_block_size = MSC_SECTOR_SIZE;
static uint32_t s_block_count = 0;
static sector_cache_t s_cache = {0};
static SemaphoreHandle_t s_io_mutex = NULL;

static inline void lock_io(void)
{
    if (s_io_mutex) {
        xSemaphoreTake(s_io_mutex, portMAX_DELAY);
    }
}

static inline void unlock_io(void)
{
    if (s_io_mutex) {
        xSemaphoreGive(s_io_mutex);
    }
}

static esp_err_t flush_cache(void)
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

static esp_err_t load_cache(uint32_t lba)
{
    if (s_cache.valid && s_cache.lba == lba) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(flush_cache(), TAG, "flush failed before load");

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
    ESP_RETURN_ON_ERROR(load_cache(lba), TAG, "load cache failed");
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
        ESP_RETURN_ON_ERROR(flush_cache(), TAG, "flush before switching LBA failed");
    }
    ESP_RETURN_ON_ERROR(load_cache(lba), TAG, "load cache failed");
    memcpy(s_cache.data + offset, buffer, bufsize);
    s_cache.dirty = true;
    return ESP_OK;
}

esp_err_t msc_init(void)
{
    s_io_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_io_mutex != NULL, ESP_ERR_NO_MEM, TAG, "no mutex");

    ESP_RETURN_ON_ERROR(sdcard_init_raw(&s_card), TAG, "sd init failed");
    s_block_size = s_card->csd.sector_size ? s_card->csd.sector_size : MSC_SECTOR_SIZE;
    s_block_count = s_card->csd.capacity;
    memset(&s_cache, 0, sizeof(s_cache));

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "tinyusb init failed");

    led_status_set(LED_STATE_USB_ATTACHED);
    ESP_LOGI(TAG, "MSC ready: blocks=%u block_size=%u", (unsigned)s_block_count, (unsigned)s_block_size);
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
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00); // medium not present
        return false;
    }
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = s_block_count;
    *block_size = (uint16_t)s_block_size;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;
    lock_io();
    esp_err_t ret;

    if (offset == 0 && (bufsize % s_block_size) == 0) {
        // Flush dirty cache to keep coherence before direct read
        if (s_cache.valid && s_cache.dirty) {
            ret = flush_cache();
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
        ESP_LOGE(TAG, "READ10 failed lba=%lu off=%lu size=%lu err=%s",
                 (unsigned long)lba, (unsigned long)offset, (unsigned long)bufsize, esp_err_to_name(ret));
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x03, 0x00);
        return -1;
    }
    return (int32_t)bufsize;

read_err:
    unlock_io();
    ESP_LOGE(TAG, "READ10 failed lba=%lu off=%lu size=%lu err=%s",
             (unsigned long)lba, (unsigned long)offset, (unsigned long)bufsize, esp_err_to_name(ret));
    tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x03, 0x00);
    return -1;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;
    lock_io();
    esp_err_t ret;

    if (offset == 0 && (bufsize % s_block_size) == 0) {
        if (s_cache.valid && s_cache.dirty) {
            ret = flush_cache();
            if (ret != ESP_OK) {
                goto write_err;
            }
        }
        uint32_t sectors = bufsize / s_block_size;
        ret = sdmmc_write_sectors(s_card, buffer, lba, sectors);
        // Invalidate cache so subsequent partial accesses reload fresh data
        s_cache.valid = false;
        s_cache.dirty = false;
    } else {
        ret = msc_write_partial(lba, offset, buffer, bufsize);
    }

    unlock_io();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WRITE10 failed lba=%lu off=%lu size=%lu err=%s",
                 (unsigned long)lba, (unsigned long)offset, (unsigned long)bufsize, esp_err_to_name(ret));
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x03, 0x00);
        return -1;
    }
    return (int32_t)bufsize;

write_err:
    unlock_io();
    ESP_LOGE(TAG, "WRITE10 failed lba=%lu off=%lu size=%lu err=%s",
             (unsigned long)lba, (unsigned long)offset, (unsigned long)bufsize, esp_err_to_name(ret));
    tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x03, 0x00);
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
    (void)lun;
    (void)buffer;
    (void)bufsize;

    if (scsi_cmd[0] == SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL) {
        return 0;
    }

    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

bool tud_msc_flush_cb(uint8_t lun)
{
    (void)lun;
    lock_io();
    esp_err_t ret = flush_cache();
    unlock_io();
    return ret == ESP_OK;
}
