#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef void (*button_longpress_cb_t)(void *ctx);

esp_err_t button_longpress_init(gpio_num_t gpio,
                                uint32_t longpress_ms,
                                uint32_t debounce_ms,
                                button_longpress_cb_t cb,
                                void *ctx);
