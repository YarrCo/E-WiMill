#pragma once

#include <stdbool.h>

typedef enum {
    LED_STATE_BOOT,
    LED_STATE_USB_ATTACHED,
    LED_STATE_USB_DETACHED,
    LED_STATE_ERROR,
} led_state_t;

void led_status_init(void);
void led_status_set(led_state_t state);
