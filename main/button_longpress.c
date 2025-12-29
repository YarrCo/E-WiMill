#include "button_longpress.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BTN"

typedef struct {
    gpio_num_t gpio;
    uint32_t longpress_ms;
    uint32_t debounce_ms;
    button_longpress_cb_t cb;
    void *ctx;
    TaskHandle_t task;
    TaskHandle_t cb_task;
} button_ctx_t;

static button_ctx_t s_btn = {0};

static void button_cb_task(void *arg)
{
    (void)arg;
    if (s_btn.cb) {
        s_btn.cb(s_btn.ctx);
    }
    s_btn.cb_task = NULL;
    vTaskDelete(NULL);
}

static void button_task(void *arg)
{
    (void)arg;
    int last_raw = 1;
    int stable_level = 1;
    int64_t last_change_us = 0;
    int64_t press_start_us = 0;
    bool triggered = false;

    for (;;) {
        int raw = gpio_get_level(s_btn.gpio);
        int64_t now_us = esp_timer_get_time();

        if (raw != last_raw) {
            last_raw = raw;
            last_change_us = now_us;
        } else if ((now_us - last_change_us) >= (int64_t)s_btn.debounce_ms * 1000) {
            stable_level = raw;
        }

        bool pressed = (stable_level == 0);
        if (pressed) {
            if (press_start_us == 0) {
                press_start_us = now_us;
                triggered = false;
            } else if (!triggered &&
                       (now_us - press_start_us) >= (int64_t)s_btn.longpress_ms * 1000) {
                triggered = true;
                if (!s_btn.cb_task && s_btn.cb) {
                    if (xTaskCreate(button_cb_task, "button_cb", 4096, NULL, 6,
                                    &s_btn.cb_task) != pdPASS) {
                        s_btn.cb_task = NULL;
                    }
                }
            }
        } else {
            press_start_us = 0;
            triggered = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t button_longpress_init(gpio_num_t gpio,
                                uint32_t longpress_ms,
                                uint32_t debounce_ms,
                                button_longpress_cb_t cb,
                                void *ctx)
{
    if (s_btn.task) {
        return ESP_OK;
    }

    s_btn.gpio = gpio;
    s_btn.longpress_ms = longpress_ms;
    s_btn.debounce_ms = debounce_ms;
    s_btn.cb = cb;
    s_btn.ctx = ctx;

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO init failed: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t ok = xTaskCreate(button_task, "button_longpress", 2048, NULL, 5, &s_btn.task);
    if (ok != pdPASS) {
        s_btn.task = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}
