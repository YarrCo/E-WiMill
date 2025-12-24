#include "config_store.h"

#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

static const char *k_namespace = "wimill";

static void copy_str(char *dst, size_t dst_len, const char *src)
{
    if (!dst || dst_len == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_len);
    dst[dst_len - 1] = '\0';
}

void config_load_defaults(wimill_config_t *cfg)
{
    if (!cfg) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    copy_str(cfg->dev_name, sizeof(cfg->dev_name), "E-WiMill");
    copy_str(cfg->sta_ssid, sizeof(cfg->sta_ssid), "");
    copy_str(cfg->sta_psk, sizeof(cfg->sta_psk), "");
    copy_str(cfg->last_sta_ip, sizeof(cfg->last_sta_ip), "0.0.0.0");
    cfg->web_port = 8080;
    cfg->wifi_boot_mode = WIFI_BOOT_AP;
}

static esp_err_t read_str(nvs_handle_t nvs, const char *key, char *out, size_t out_len)
{
    size_t required = out_len;
    esp_err_t err = nvs_get_str(nvs, key, out, &required);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err == ESP_OK) {
        out[out_len - 1] = '\0';
    }
    return err;
}

esp_err_t config_load(wimill_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    config_load_defaults(cfg);

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(k_namespace, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = read_str(nvs, "dev_name", cfg->dev_name, sizeof(cfg->dev_name));
    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }
    err = read_str(nvs, "sta_ssid", cfg->sta_ssid, sizeof(cfg->sta_ssid));
    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }
    err = read_str(nvs, "sta_psk", cfg->sta_psk, sizeof(cfg->sta_psk));
    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }
    err = read_str(nvs, "last_sta_ip", cfg->last_sta_ip, sizeof(cfg->last_sta_ip));
    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }

    uint8_t boot = cfg->wifi_boot_mode;
    err = nvs_get_u8(nvs, "wifi_boot", &boot);
    if (err == ESP_OK) {
        if (boot > WIFI_BOOT_STA) {
            boot = WIFI_BOOT_AP;
        }
        cfg->wifi_boot_mode = boot;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return err;
    }

    uint16_t port = 0;
    err = nvs_get_u16(nvs, "web_port", &port);
    if (err == ESP_OK) {
        cfg->web_port = port;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return err;
    }

    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t config_save(const wimill_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(k_namespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(nvs, "dev_name", cfg->dev_name);
    if (err == ESP_OK) err = nvs_set_str(nvs, "sta_ssid", cfg->sta_ssid);
    if (err == ESP_OK) err = nvs_set_str(nvs, "sta_psk", cfg->sta_psk);
    if (err == ESP_OK) err = nvs_set_str(nvs, "last_sta_ip", cfg->last_sta_ip);
    if (err == ESP_OK) err = nvs_set_u8(nvs, "wifi_boot", cfg->wifi_boot_mode);
    if (err == ESP_OK) err = nvs_set_u16(nvs, "web_port", cfg->web_port);
    if (err == ESP_OK) err = nvs_commit(nvs);

    nvs_close(nvs);
    return err;
}
