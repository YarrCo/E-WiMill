#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool mounted;
    uint32_t current_freq_khz;
    uint32_t default_freq_khz;
    char card_name[8];
    uint64_t total_bytes;
    uint64_t free_bytes;
} sdcard_status_t;

typedef struct {
    uint64_t total_bytes;
    uint64_t free_bytes;
} sd_space_info_t;

esp_err_t sdcard_mount(void);
esp_err_t sdcard_unmount(void);
bool sdcard_is_mounted(void);
const char *sdcard_mount_point(void);
uint32_t sdcard_get_current_freq_khz(void);
uint32_t sdcard_get_default_freq_khz(void);
esp_err_t sdcard_set_frequency(uint32_t freq_khz, bool remount);
esp_err_t sdcard_get_status(sdcard_status_t *out_status);
esp_err_t sdcard_get_space(sd_space_info_t *info);
esp_err_t sdcard_list_root(void);
esp_err_t sdcard_remove(const char *name);
esp_err_t sdcard_mkdir(const char *name);
esp_err_t sdcard_cat(const char *name, size_t max_bytes);
esp_err_t sdcard_touch(const char *name, size_t size_bytes);
esp_err_t sdcard_self_test(size_t size_mb, uint32_t freq_khz);
