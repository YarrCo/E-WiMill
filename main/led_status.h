#pragma once

#include <stdbool.h>

typedef enum {
    LED_STATE_BOOT,
    LED_STATE_USB_ATTACHED,
    LED_STATE_USB_DETACHED,
    LED_STATE_ERROR,
    LED_STATE_QUEUE_WAIT,
    LED_STATE_WIFI_DISCONNECTED,
} led_state_t;

void led_status_init(void);
void led_status_set(led_state_t state);
void led_status_set_setup(bool active);
void led_status_set_wifi(bool connected);
