#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cli.h"
#include "sdcard.h"
#include "wimill_pins.h"

#define TAG "APP"

static void log_space(void)
{
    sd_space_info_t info = {0};
    if (sdcard_get_space(&info) == ESP_OK) {
        double total_mb = (double)info.total_bytes / (1024.0 * 1024.0);
        double free_mb = (double)info.free_bytes / (1024.0 * 1024.0);
        ESP_LOGI(TAG, "Space: total=%.2f MB, free=%.2f MB", total_mb, free_mb);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "E-WiMill MVP-01: SD Mount + File Manager (Serial)");
    ESP_LOGI(TAG, "SPI pins - CS:%d SCK:%d MOSI:%d MISO:%d", WIMILL_PIN_SD_CS, WIMILL_PIN_SD_SCK,
             WIMILL_PIN_SD_MOSI, WIMILL_PIN_SD_MISO);
    ESP_LOGI(TAG, "Default SD freq: %u kHz", WIMILL_SD_FREQ_KHZ_DEFAULT);

    esp_err_t err = sdcard_mount();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SD mount OK");
        log_space();
        sdcard_list_root();
    } else {
        ESP_LOGW(TAG, "Mount failed. Fix wiring/format and run 'mount' in CLI after reboot or wiring fix.");
    }

    if (cli_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start CLI");
    } else {
        cli_print_help();
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
