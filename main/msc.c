#include "msc.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"

#include "sdcard.h"
#include "wimill_pins.h"

#define TAG "MSC"

#define MSC_QUEUE_LEN 4
#define MSC_TASK_STACK 4096
#define MSC_TASK_PRIO 5

static msc_state_t s_state = MSC_STATE_USB_ATTACHED;
static uint64_t s_last_activity_us = 0;
static uint32_t s_idle_timeout_ms = 15000;
static SemaphoreHandle_t s_state_mutex = NULL;
static QueueHandle_t s_op_queue = NULL;
static TaskHandle_t s_msc_task = NULL;

static void set_state(msc_state_t st)
{
    if (s_state != st) {
        ESP_LOGI(TAG, "State change: %d -> %d", s_state, st);
        s_state = st;
    }
}

static bool usb_idle_elapsed(void)
{
    uint64_t now = esp_timer_get_time();
    uint64_t diff_ms = (now - s_last_activity_us) / 1000ULL;
    return diff_ms >= s_idle_timeout_ms;
}

static void update_activity(void)
{
    s_last_activity_us = esp_timer_get_time();
}

static void msc_mount_sdmmc(void)
{
    // SD is mounted by our sdcard.c; tinyusb_msc_storage_init_sdmmc expects sdmmc_card_t*
    // But we are using SDSPI so we can reuse sdmmc_card_t pointer from sdcard.c by adding accessor if needed.
    // For now, we assume sdcard_mount already done and we cannot hand over card directly.
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
            // detach
            set_state(MSC_STATE_DETACHING);
            tinyusb_msc_storage_unmount(); // stop exposing
            set_state(MSC_STATE_USB_DETACHED);

            // perform op (placeholder copytest)
            if (op.type == MSC_OP_COPYTEST) {
                ESP_LOGI(TAG, "COPYTEST %zu MB (placeholder)", op.size_mb);
                // TODO: implement real copytest via sdcard API with mount
            }

            // attach back
            set_state(MSC_STATE_ATTACHING);
            tinyusb_msc_storage_mount("/sdcard");
            set_state(MSC_STATE_USB_ATTACHED);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void msc_init(void)
{
    s_state_mutex = xSemaphoreCreateMutex();
    s_op_queue = xQueueCreate(MSC_QUEUE_LEN, sizeof(msc_op_t));
    s_last_activity_us = esp_timer_get_time();

    // TinyUSB driver install (simple config; descriptors default from esp_tinyusb)
    tinyusb_config_t tusb_cfg = {
        .external_phy = false,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // Init storage using SDMMC/SD over SDSPI already initialised
    // Note: For SDSPI, esp_tinyusb doesn't have helper; we will use sdmmc_card_t* from sdcard.c if exposed.

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
    set_state(MSC_STATE_ATTACHING);
    ESP_RETURN_ON_ERROR(tinyusb_msc_storage_mount("/sdcard"), TAG, "attach failed");
    set_state(MSC_STATE_USB_ATTACHED);
    return ESP_OK;
}

esp_err_t msc_force_detach(void)
{
    set_state(MSC_STATE_DETACHING);
    ESP_RETURN_ON_ERROR(tinyusb_msc_storage_unmount(), TAG, "detach failed");
    set_state(MSC_STATE_USB_DETACHED);
    return ESP_OK;
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
