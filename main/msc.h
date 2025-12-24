#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    MSC_STATE_USB_ATTACHED,
    MSC_STATE_USB_DETACHED,
    MSC_STATE_ERROR,
} msc_state_t;

esp_err_t msc_init(void);
msc_state_t msc_get_state(void);
esp_err_t msc_attach(void);
esp_err_t msc_detach(void);
bool msc_is_host_connected(void);
