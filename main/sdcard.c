#include "sdcard.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"

#include "wimill_pins.h"

#define TAG "SDCARD"

static uint32_t s_current_freq_khz = WIMILL_SD_FREQ_KHZ_DEFAULT;

static bool is_supported_freq(uint32_t khz)
{
    return khz == WIMILL_SD_FREQ_KHZ_DEFAULT ||
           khz == WIMILL_SD_FREQ_KHZ_20MHZ ||
           khz == WIMILL_SD_FREQ_KHZ_26MHZ;
}

esp_err_t sdcard_init_raw(sdmmc_card_t **out_card)
{
    if (!out_card)
    {
        return ESP_ERR_INVALID_ARG;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = s_current_freq_khz;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = WIMILL_PIN_SD_MOSI,
        .miso_io_num = WIMILL_PIN_SD_MISO,
        .sclk_io_num = WIMILL_PIN_SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8192,
    };

    gpio_set_pull_mode(WIMILL_PIN_SD_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(WIMILL_PIN_SD_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(WIMILL_PIN_SD_SCK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(WIMILL_PIN_SD_CS, GPIO_PULLUP_ONLY);

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = WIMILL_PIN_SD_CS;
    slot_config.host_id = host.slot;

    sdspi_dev_handle_t dev_handle = 0;
    ret = sdspi_host_init_device(&slot_config, &dev_handle);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        return ret;
    }

    sdmmc_card_t *card = calloc(1, sizeof(sdmmc_card_t));
    if (!card)
    {
        return ESP_ERR_NO_MEM;
    }

    ret = sdmmc_card_init(&host, card);
    if (ret != ESP_OK)
    {
        free(card);
        return ret;
    }

    char name[8] = {0};
    memcpy(name, card->cid.name, sizeof(card->cid.name));
    double size_mb = ((double)card->csd.capacity) * card->csd.sector_size / (1024.0 * 1024.0);
    ESP_LOGI(TAG, "SD raw init OK: %s size=%.2f MB freq=%u kHz", name, size_mb, s_current_freq_khz);

    *out_card = card;
    return ESP_OK;
}

uint32_t sdcard_get_current_freq_khz(void)
{
    return s_current_freq_khz;
}

esp_err_t sdcard_set_frequency(uint32_t freq_khz)
{
    if (!is_supported_freq(freq_khz))
    {
        return ESP_ERR_INVALID_ARG;
    }
    s_current_freq_khz = freq_khz;
    return ESP_OK;
}
