#include "msc.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"
#include "tusb.h"

#include "led_status.h"
#include "sdcard.h"
#include "wimill_pins.h"

#define TAG "MSC"

#define MSC_QUEUE_LEN 4
#define MSC_TASK_STACK 4096
#define MSC_TASK_PRIO 5
#define MSC_ALLOC_UNIT (32 * 1024)

static msc_state_t s_state = MSC_STATE_USB_ATTACHED;
static uint64_t s_last_activity_us = 0;
static uint32_t s_idle_timeout_ms = 15000;
static QueueHandle_t s_op_queue = NULL;
static TaskHandle_t s_msc_task = NULL;
static sdmmc_card_t *s_card = NULL;
static tinyusb_msc_storage_handle_t s_storage = NULL;

static void set_state(msc_state_t st)
{
    if (s_state != st) {
        ESP_LOGI(TAG, "State change: %d -> %d", s_state, st);
        s_state = st;
        switch (s_state) {
        case MSC_STATE_USB_ATTACHED:
            led_status_set(LED_STATE_USB_ATTACHED);
            break;
        case MSC_STATE_USB_DETACHED:
            led_status_set(LED_STATE_USB_DETACHED);
            break;
        case MSC_STATE_ATTACHING:
        case MSC_STATE_DETACHING:
            led_status_set(LED_STATE_QUEUE_WAIT);
            break;
        case MSC_STATE_ERROR:
            led_status_set(LED_STATE_ERROR);
            break;
        default:
            break;
        }
    }
}

static bool usb_idle_elapsed(void)
{
    uint64_t now = esp_timer_get_time();
    uint64_t diff_ms = (now - s_last_activity_us) / 1000ULL;
    return diff_ms >= s_idle_timeout_ms;
}

static esp_err_t set_mount_point(tinyusb_msc_mount_point_t target)
{
    if (!s_storage) {
        return ESP_ERR_INVALID_STATE;
    }
    tinyusb_msc_mount_point_t current;
    ESP_RETURN_ON_ERROR(tinyusb_msc_get_storage_mount_point(s_storage, &current), TAG, "get mount point failed");
    if (current == target) {
        return ESP_OK;
    }
    esp_err_t ret = tinyusb_msc_set_storage_mount_point(s_storage, target);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch mount point to %d: %s", target, esp_err_to_name(ret));
    }
    return ret;
}

static void update_activity(void)
{
    s_last_activity_us = esp_timer_get_time();
}

void msc_on_usb_activity(void)
{
    update_activity();
}

void msc_on_usb_rw(bool write, uint32_t lba, uint32_t bytes)
{
    (void)write;
    (void)lba;
    (void)bytes;
    update_activity();
}

static void storage_event_cb(tinyusb_msc_storage_handle_t handle, tinyusb_msc_event_t *event, void *arg)
{
    (void)handle;
    (void)arg;
    ESP_LOGI(TAG, "MSC event id=%d mount_point=%d", event->id, event->mount_point);
    if (event->id == TINYUSB_MSC_EVENT_MOUNT_COMPLETE) {
        if (event->mount_point == TINYUSB_MSC_STORAGE_MOUNT_USB) {
            set_state(MSC_STATE_USB_ATTACHED);
            sdcard_set_mounted(false);
        } else if (event->mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP) {
            set_state(MSC_STATE_USB_DETACHED);
            sdcard_set_mounted(true);
        }
    }
}

static void msc_task(void *arg)
{
    (void)arg;
    for (;;) {
        msc_op_t op;
        if (xQueueReceive(s_op_queue, &op, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (!usb_idle_elapsed()) {
                ESP_LOGW(TAG, "USB busy, requeue op");
                xQueueSendToBack(s_op_queue, &op, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            // detach (switch to APP mount)
            set_state(MSC_STATE_DETACHING);
            if (set_mount_point(TINYUSB_MSC_STORAGE_MOUNT_APP) != ESP_OK) {
                set_state(MSC_STATE_ERROR);
                continue;
            }

            // perform op (copytest only for now)
            if (op.type == MSC_OP_COPYTEST) {
                const char *tmp_path = WIMILL_SD_MOUNT_POINT "/copytest.part";
                const char *final_path = WIMILL_SD_MOUNT_POINT "/copytest.bin";
                ESP_LOGI(TAG, "COPYTEST %zu MB", op.size_mb);
                sdcard_lock();
                FILE *f = fopen(tmp_path, "wb");
                if (f) {
                    size_t total = op.size_mb * 1024 * 1024;
                    size_t chunk = 4096;
                    uint8_t *buf = malloc(chunk);
                    if (buf) {
                        memset(buf, 0xAA, chunk);
                        size_t written = 0;
                        while (written < total) {
                            size_t to_write = (total - written) > chunk ? chunk : (total - written);
                            if (fwrite(buf, 1, to_write, f) != to_write) {
                                ESP_LOGE(TAG, "copytest write failed at %zu", written);
                                break;
                            }
                            written += to_write;
                        }
                        free(buf);
                        fflush(f);
                        fsync(fileno(f));
                        fclose(f);
                        rename(tmp_path, final_path);
                        ESP_LOGI(TAG, "copytest done: %s (%zu bytes)", final_path, total);
                    } else {
                        ESP_LOGE(TAG, "copytest malloc failed");
                        fclose(f);
                    }
                } else {
                    ESP_LOGE(TAG, "copytest open failed");
                }
                sdcard_unlock();
            }

            // attach back to USB
            set_state(MSC_STATE_ATTACHING);
            set_mount_point(TINYUSB_MSC_STORAGE_MOUNT_USB);
            set_state(MSC_STATE_USB_ATTACHED);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void msc_init(void)
{
    s_op_queue = xQueueCreate(MSC_QUEUE_LEN, sizeof(msc_op_t));
    s_last_activity_us = esp_timer_get_time();

    // Init raw card (SDSPI)
    ESP_ERROR_CHECK(sdcard_init_raw(&s_card));

    // TinyUSB driver install (using tusb_config.h descriptors)
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_msc_storage_config_t storage_cfg = {
        .medium.card = s_card,
        .fat_fs = {
            .base_path = WIMILL_SD_MOUNT_POINT,
            .config = {
                .format_if_mount_failed = false,
                .max_files = 5,
                .allocation_unit_size = MSC_ALLOC_UNIT,
                .disk_status_check_enable = true,
            },
            .do_not_format = true,
            .format_flags = FM_ANY,
        },
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB,
    };
    ESP_ERROR_CHECK(tinyusb_msc_new_storage_sdmmc(&storage_cfg, &s_storage));
    tinyusb_msc_set_storage_callback(storage_event_cb, NULL);

    xTaskCreatePinnedToCore(msc_task, "msc_task", MSC_TASK_STACK, NULL, MSC_TASK_PRIO, &s_msc_task, tskNO_AFFINITY);
    set_state(MSC_STATE_USB_ATTACHED);
    ESP_LOGI(TAG, "MSC init done, state USB_ATTACHED");
}

msc_state_t msc_get_state(void)
{
    return s_state;
}

uint64_t msc_last_activity_ms(void)
{
    return s_last_activity_us / 1000ULL;
}

esp_err_t msc_force_attach(void)
{
    esp_err_t ret = set_mount_point(TINYUSB_MSC_STORAGE_MOUNT_USB);
    if (ret == ESP_OK) {
        set_state(MSC_STATE_USB_ATTACHED);
    }
    return ret;
}

esp_err_t msc_force_detach(void)
{
    if (!usb_idle_elapsed()) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = set_mount_point(TINYUSB_MSC_STORAGE_MOUNT_APP);
    if (ret == ESP_OK) {
        set_state(MSC_STATE_USB_DETACHED);
    }
    return ret;
}

esp_err_t msc_set_idle_timeout(uint32_t ms)
{
    s_idle_timeout_ms = ms;
    return ESP_OK;
}

uint32_t msc_get_idle_timeout(void)
{
    return s_idle_timeout_ms;
}

esp_err_t msc_enqueue_op(const msc_op_t *op)
{
    if (!op) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xQueueSendToBack(s_op_queue, op, 0) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void msc_queue_dump(void)
{
    UBaseType_t pending = uxQueueMessagesWaiting(s_op_queue);
    ESP_LOGI(TAG, "Queue pending ops: %lu", (unsigned long)pending);
}
