#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "button_longpress.h"
#include "cli.h"
#include "led_status.h"
#include "msc.h"
#include "setup_mode.h"
#include "wimill_pins.h"

#define TAG "APP"

static void setup_button_cb(void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "Setup button long-press detected");
    setup_mode_start();
}

void app_main(void)
{
    // Fix for slow USB init: silence the noisy drivers!
    esp_log_level_set("sdspi_transaction", ESP_LOG_ERROR);
    esp_log_level_set("sdspi_host", ESP_LOG_ERROR);
    esp_log_level_set("sdmmc_req", ESP_LOG_ERROR);
    esp_log_level_set("sdmmc_cmd", ESP_LOG_ERROR);
    esp_log_level_set("sdmmc_init", ESP_LOG_ERROR);

    ESP_LOGI(TAG, "E-WiMill MSC debug build: raw SDSPI + manual MSC callbacks");
    ESP_LOGI(TAG, "SPI pins - CS:%d SCK:%d MOSI:%d MISO:%d", WIMILL_PIN_SD_CS, WIMILL_PIN_SD_SCK,
             WIMILL_PIN_SD_MOSI, WIMILL_PIN_SD_MISO);
    ESP_LOGI(TAG, "SD freq: %u kHz", WIMILL_SD_FREQ_KHZ_DEFAULT);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
    }

    led_status_init();
    led_status_set(LED_STATE_BOOT);

    err = setup_mode_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Setup mode init failed: %s", esp_err_to_name(err));
    }

    err = button_longpress_init(WIMILL_PIN_SETUP_BTN, 5000, 40, setup_button_cb, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Button init failed: %s", esp_err_to_name(err));
    }

    err = msc_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MSC init failed: %s", esp_err_to_name(err));
        led_status_set(LED_STATE_ERROR);
        return;
    }

    err = cli_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "CLI start failed: %s", esp_err_to_name(err));
    }

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
