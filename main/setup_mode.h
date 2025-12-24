#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t setup_mode_init(void);
esp_err_t setup_mode_start(void);
esp_err_t setup_mode_autostart(void);
bool setup_mode_is_active(void);
