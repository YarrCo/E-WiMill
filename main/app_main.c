#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cli.h"
#include "led_status.h"
#include "msc.h"
#include "wimill_pins.h"

#define TAG "APP"

void app_main(void)
{
    ESP_LOGI(TAG, "E-WiMill MSC debug build: raw SDSPI + manual MSC callbacks");
    ESP_LOGI(TAG, "SPI pins - CS:%d SCK:%d MOSI:%d MISO:%d", WIMILL_PIN_SD_CS, WIMILL_PIN_SD_SCK,
             WIMILL_PIN_SD_MOSI, WIMILL_PIN_SD_MISO);
    ESP_LOGI(TAG, "SD freq: %u kHz", WIMILL_SD_FREQ_KHZ_DEFAULT);

    led_status_init();
    led_status_set(LED_STATE_BOOT);

    esp_err_t err = msc_init();
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
