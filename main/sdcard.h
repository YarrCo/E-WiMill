#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdmmc_cmd.h"

esp_err_t sdcard_init_raw(sdmmc_card_t **out_card);
uint32_t sdcard_get_current_freq_khz(void);
esp_err_t sdcard_set_frequency(uint32_t freq_khz);
