#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdmmc_cmd.h"

typedef struct {
    uint64_t total_bytes;
    uint64_t free_bytes;
} sd_space_info_t;

typedef struct {
    bool mounted;
    uint32_t current_freq_khz;
    uint32_t default_freq_khz;
    uint32_t allocation_unit;
    uint32_t sdtest_buf_bytes;
    char card_name[8];
    uint64_t total_bytes;
    uint64_t free_bytes;
} sdcard_status_t;

typedef enum {
    SDCARD_MODE_USB,
    SDCARD_MODE_APP,
} sdcard_mode_t;

void sdcard_set_mode(sdcard_mode_t mode);
sdcard_mode_t sdcard_get_mode(void);
bool sdcard_is_vfs_allowed(void);

void sdcard_lock(void);
void sdcard_unlock(void);

esp_err_t sdcard_init_raw(sdmmc_card_t **out_card);
esp_err_t sdcard_mount(void);
esp_err_t sdcard_unmount(void);
bool sdcard_is_mounted(void);
const char *sdcard_mount_point(void);

uint32_t sdcard_get_current_freq_khz(void);
uint32_t sdcard_get_default_freq_khz(void);
esp_err_t sdcard_set_frequency(uint32_t freq_khz, bool remount);
bool sdcard_get_disk_status_check(void);
esp_err_t sdcard_set_disk_status_check(bool enable, bool remount);

esp_err_t sdcard_get_status(sdcard_status_t *out_status);
esp_err_t sdcard_get_space(sd_space_info_t *info);
esp_err_t sdcard_list(const char *path);
esp_err_t sdcard_remove(const char *path);
esp_err_t sdcard_mkdir(const char *path);
esp_err_t sdcard_cat(const char *path, size_t max_bytes);
esp_err_t sdcard_touch(const char *path, size_t size_bytes);
esp_err_t sdcard_self_test(size_t size_mb, uint32_t freq_khz, size_t buf_bytes);
esp_err_t sdcard_bench(size_t size_mb, size_t buf_bytes);
