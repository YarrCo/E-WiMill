#include "led_status.h"

#include "driver/rmt_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#include "wimill_pins.h"

#define TAG "LED"

static led_strip_handle_t s_strip = NULL;
static led_state_t s_state = LED_STATE_BOOT;
static TaskHandle_t s_led_task = NULL;
static bool s_setup_active = false;
static bool s_setup_entry_pending = false;
static bool s_wifi_disconnected = false;

static void set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_strip) {
        return;
    }
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

static void blink_pattern(uint8_t r, uint8_t g, uint8_t b, uint32_t on_ms, uint32_t off_ms)
{
    set_color(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    set_color(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(off_ms));
}

static void setup_entry_pattern(void)
{
    for (int i = 0; i < 3; ++i) {
        blink_pattern(0, 0, 64, 120, 120);
    }
}

static void setup_breathe(void)
{
    for (int level = 0; level <= 64; level += 2) {
        if (!s_setup_active) {
            return;
        }
        set_color((uint8_t)(level / 4), 0, (uint8_t)level);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    for (int level = 64; level >= 0; level -= 2) {
        if (!s_setup_active) {
            return;
        }
        set_color((uint8_t)(level / 4), 0, (uint8_t)level);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    for (int level = 0; level <= 64; level += 2) {
        if (!s_setup_active) {
            return;
        }
        set_color(0, (uint8_t)(level / 2), (uint8_t)level);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    for (int level = 64; level >= 0; level -= 2) {
        if (!s_setup_active) {
            return;
        }
        set_color(0, (uint8_t)(level / 2), (uint8_t)level);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static void normal_breathe(uint8_t r, uint8_t g, uint8_t b, uint32_t step_ms)
{
    for (int level = 0; level <= 64; level += 2) {
        if (s_setup_active) {
            return;
        }
        set_color((uint8_t)((r * level) / 64),
                  (uint8_t)((g * level) / 64),
                  (uint8_t)((b * level) / 64));
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
    for (int level = 64; level >= 0; level -= 2) {
        if (s_setup_active) {
            return;
        }
        set_color((uint8_t)((r * level) / 64),
                  (uint8_t)((g * level) / 64),
                  (uint8_t)((b * level) / 64));
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
}

static void led_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_setup_active) {
            if (s_setup_entry_pending) {
                setup_entry_pattern();
                s_setup_entry_pending = false;
            }
            setup_breathe();
            continue;
        }
        if (s_wifi_disconnected) {
            blink_pattern(96, 0, 24, 200, 800);
            continue;
        }
        switch (s_state) {
        case LED_STATE_BOOT:
            blink_pattern(0, 64, 0, 50, 500);
            s_state = LED_STATE_USB_ATTACHED;
            break;
        case LED_STATE_USB_ATTACHED:
            normal_breathe(6, 87, 33, 40);
            break;
        case LED_STATE_USB_DETACHED:
            normal_breathe(6, 87, 33, 40);
            break;
        case LED_STATE_ERROR:
            blink_pattern(64, 0, 0, 100, 200);
            blink_pattern(64, 0, 0, 100, 800);
            break;
        case LED_STATE_QUEUE_WAIT:
            blink_pattern(48, 0, 48, 150, 500);
            break;
        case LED_STATE_WIFI_DISCONNECTED:
            blink_pattern(96, 0, 24, 200, 800);
            break;
        default:
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        }
    }
}

void led_status_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WIMILL_RGB_GPIO,
        .max_leds = WIMILL_RGB_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed (gpio=%d): %s", WIMILL_RGB_GPIO, esp_err_to_name(err));
        s_strip = NULL;
        return;
    }

    set_color(0, 0, 0);
    xTaskCreatePinnedToCore(led_task, "led_status", 2048, NULL, 3, &s_led_task, tskNO_AFFINITY);
}

void led_status_set(led_state_t state)
{
    s_state = state;
}

void led_status_set_setup(bool active)
{
    if (active && !s_setup_active) {
        s_setup_active = true;
        s_setup_entry_pending = true;
    } else if (!active && s_setup_active) {
        s_setup_active = false;
        s_setup_entry_pending = false;
        set_color(0, 0, 0);
    }
}

void led_status_set_wifi(bool connected)
{
    s_wifi_disconnected = !connected;
}
