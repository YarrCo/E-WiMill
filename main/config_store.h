#pragma once

#include <stdint.h>

#include "esp_err.h"

#define CONFIG_DEV_NAME_LEN 32
#define CONFIG_STA_SSID_LEN 33
#define CONFIG_STA_PSK_LEN 65
#define CONFIG_LAST_IP_LEN 16

typedef enum {
    WIFI_BOOT_AP = 0,
    WIFI_BOOT_STA = 1,
} wifi_boot_mode_t;

typedef struct {
    char dev_name[CONFIG_DEV_NAME_LEN];
    char sta_ssid[CONFIG_STA_SSID_LEN];
    char sta_psk[CONFIG_STA_PSK_LEN];
    uint16_t web_port;
    char last_sta_ip[CONFIG_LAST_IP_LEN];
    uint8_t wifi_boot_mode;
} wimill_config_t;

void config_load_defaults(wimill_config_t *cfg);
esp_err_t config_load(wimill_config_t *cfg);
esp_err_t config_save(const wimill_config_t *cfg);
