#include "sdcard.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "wimill_pins.h"

#define TAG "SDCARD"
#define DEFAULT_MAX_FILES 10
#define DEFAULT_ALLOC_UNIT (16 * 1024)
#define SDTEST_FILE_PATH WIMILL_SD_MOUNT_POINT "/.wimill_sdtest.bin"
#define SDTEST_BLOCK_SIZE 4096

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;
static spi_host_device_t s_host_id = SPI2_HOST;
static uint32_t s_current_freq_khz = WIMILL_SD_FREQ_KHZ_DEFAULT;

static bool is_supported_freq(uint32_t khz)
{
    return khz == WIMILL_SD_FREQ_KHZ_DEFAULT ||
           khz == WIMILL_SD_FREQ_KHZ_4MHZ ||
           khz == WIMILL_SD_FREQ_KHZ_1MHZ;
}

static void log_mount_hint(esp_err_t err)
{
    ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(err));
    ESP_LOGW(TAG, "Check wiring (3.3V, CS/SCK/MOSI/MISO), card formatted FAT32, and try again.");
}

esp_err_t sdcard_unmount(void)
{
    if (!s_mounted) {
        return ESP_OK;
    }
    esp_err_t err = esp_vfs_fat_sdcard_unmount(WIMILL_SD_MOUNT_POINT, s_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(err));
        return err;
    }
    s_card = NULL;
    s_mounted = false;
    spi_bus_free(s_host_id);
    ESP_LOGI(TAG, "SD unmounted");
    return ESP_OK;
}

static esp_err_t mount_filesystem(void)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = s_host_id;
    host.max_freq_khz = s_current_freq_khz; // adjustable for stability

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = WIMILL_PIN_SD_MOSI,
        .miso_io_num = WIMILL_PIN_SD_MISO,
        .sclk_io_num = WIMILL_PIN_SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    gpio_set_pull_mode(WIMILL_PIN_SD_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(WIMILL_PIN_SD_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(WIMILL_PIN_SD_SCK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(WIMILL_PIN_SD_CS, GPIO_PULLUP_ONLY);

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to init SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = WIMILL_PIN_SD_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = DEFAULT_MAX_FILES,
        .allocation_unit_size = DEFAULT_ALLOC_UNIT,
        .disk_status_check_enable = true,
    };

    ESP_LOGI(TAG, "Mounting SD at %s (freq %u kHz)", WIMILL_SD_MOUNT_POINT, host.max_freq_khz);
    ret = esp_vfs_fat_sdspi_mount(WIMILL_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        log_mount_hint(ret);
        if (ret != ESP_ERR_INVALID_STATE) {
            spi_bus_free(host.slot);
        }
        return ret;
    }

    s_mounted = true;
    ESP_LOGI(TAG, "Mounted at %s", WIMILL_SD_MOUNT_POINT);

    if (s_card) {
        char name[8] = {0};
        memcpy(name, s_card->cid.name, sizeof(s_card->cid.name));
        const double size_mb = ((double)s_card->csd.capacity) * s_card->csd.sector_size / (1024.0 * 1024.0);
        ESP_LOGI(TAG, "Card name: %s, size: %.2f MB", name, size_mb);
    }

    return ESP_OK;
}

esp_err_t sdcard_mount(void)
{
    if (s_mounted) {
        return ESP_OK;
    }
    return mount_filesystem();
}

bool sdcard_is_mounted(void)
{
    return s_mounted;
}

const char *sdcard_mount_point(void)
{
    return WIMILL_SD_MOUNT_POINT;
}

uint32_t sdcard_get_current_freq_khz(void)
{
    return s_current_freq_khz;
}

uint32_t sdcard_get_default_freq_khz(void)
{
    return WIMILL_SD_FREQ_KHZ_DEFAULT;
}

esp_err_t sdcard_set_frequency(uint32_t freq_khz, bool remount)
{
    if (!is_supported_freq(freq_khz)) {
        ESP_LOGW(TAG, "Unsupported frequency: %u kHz", freq_khz);
        return ESP_ERR_INVALID_ARG;
    }

    if (s_current_freq_khz == freq_khz && (!remount || !s_mounted)) {
        ESP_LOGI(TAG, "SD frequency already set to %u kHz", freq_khz);
        return ESP_OK;
    }

    s_current_freq_khz = freq_khz;
    ESP_LOGI(TAG, "SD frequency set to %u kHz", freq_khz);

    if (remount && s_mounted) {
        ESP_LOGI(TAG, "Remounting SD to apply new frequency...");
        ESP_RETURN_ON_ERROR(sdcard_unmount(), TAG, "Unmount failed");
        return sdcard_mount();
    }

    return ESP_OK;
}

esp_err_t sdcard_get_space(sd_space_info_t *info)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_mounted) {
        ESP_LOGE(TAG, "SD card is not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    FATFS *fs = NULL;
    DWORD free_clusters = 0;
    FRESULT res = f_getfree(WIMILL_SD_MOUNT_POINT, &free_clusters, &fs);
    if (res != FR_OK || !fs) {
        ESP_LOGE(TAG, "f_getfree failed: %d", res);
        return ESP_FAIL;
    }

    uint64_t cluster_size = ((uint64_t)fs->csize) * 512; // bytes per cluster
    info->total_bytes = ((uint64_t)(fs->n_fatent - 2)) * cluster_size;
    info->free_bytes = ((uint64_t)free_clusters) * cluster_size;
    return ESP_OK;
}

esp_err_t sdcard_get_status(sdcard_status_t *out_status)
{
    if (!out_status) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out_status, 0, sizeof(*out_status));
    out_status->mounted = s_mounted;
    out_status->current_freq_khz = s_current_freq_khz;
    out_status->default_freq_khz = WIMILL_SD_FREQ_KHZ_DEFAULT;

    if (s_mounted && s_card) {
        memcpy(out_status->card_name, s_card->cid.name, sizeof(s_card->cid.name));
        sd_space_info_t space;
        if (sdcard_get_space(&space) == ESP_OK) {
            out_status->total_bytes = space.total_bytes;
            out_status->free_bytes = space.free_bytes;
        }
    }
    return ESP_OK;
}

static esp_err_t build_path(const char *name, char *out, size_t out_size)
{
    if (!name || !out || out_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    int written = snprintf(out, out_size, "%s/%s", WIMILL_SD_MOUNT_POINT, name);
    if (written < 0 || (size_t)written >= out_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

esp_err_t sdcard_list_root(void)
{
    if (!s_mounted) {
        ESP_LOGE(TAG, "SD card is not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    DIR *dir = opendir(WIMILL_SD_MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "opendir failed: errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Contents of %s:", WIMILL_SD_MOUNT_POINT);
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        char full_path[256];
        if (build_path(entry->d_name, full_path, sizeof(full_path)) != ESP_OK) {
            ESP_LOGW(TAG, "Name too long: %s", entry->d_name);
            continue;
        }

        struct stat st;
        if (stat(full_path, &st) != 0) {
            ESP_LOGW(TAG, "stat failed for %s: errno=%d", entry->d_name, errno);
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            ESP_LOGI(TAG, "<DIR>  %-24s", entry->d_name);
        } else {
            ESP_LOGI(TAG, "FILE   %-24s %8ld bytes", entry->d_name, (long)st.st_size);
        }
    }

    closedir(dir);
    return ESP_OK;
}

esp_err_t sdcard_remove(const char *name)
{
    if (!s_mounted) {
        ESP_LOGE(TAG, "SD card is not mounted");
        return ESP_ERR_INVALID_STATE;
    }
    char path[256];
    ESP_RETURN_ON_ERROR(build_path(name, path, sizeof(path)), TAG, "Path build failed");

    struct stat st;
    if (stat(path, &st) != 0) {
        ESP_LOGE(TAG, "stat failed: %s (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    if (S_ISDIR(st.st_mode)) {
        ESP_LOGE(TAG, "%s is a directory (rm only handles files)", name);
        return ESP_ERR_INVALID_ARG;
    }

    if (unlink(path) != 0) {
        ESP_LOGE(TAG, "unlink failed: errno=%d", errno);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Removed %s", name);
    return ESP_OK;
}

esp_err_t sdcard_mkdir(const char *name)
{
    if (!s_mounted) {
        ESP_LOGE(TAG, "SD card is not mounted");
        return ESP_ERR_INVALID_STATE;
    }
    char path[256];
    ESP_RETURN_ON_ERROR(build_path(name, path, sizeof(path)), TAG, "Path build failed");

    if (mkdir(path, 0777) != 0) {
        ESP_LOGE(TAG, "mkdir failed (errno=%d)", errno);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Created directory %s", name);
    return ESP_OK;
}

static void print_hex_line(const uint8_t *data, size_t len)
{
    char ascii[17];
    for (size_t i = 0; i < len; ++i) {
        printf("%02X ", data[i]);
        ascii[i] = (data[i] >= 32 && data[i] < 127) ? (char)data[i] : '.';
    }
    ascii[len] = '\0';
    for (size_t i = len; i < 16; ++i) {
        printf("   ");
    }
    printf("| %s\n", ascii);
}

esp_err_t sdcard_cat(const char *name, size_t max_bytes)
{
    if (!s_mounted) {
        ESP_LOGE(TAG, "SD card is not mounted");
        return ESP_ERR_INVALID_STATE;
    }
    char path[256];
    ESP_RETURN_ON_ERROR(build_path(name, path, sizeof(path)), TAG, "Path build failed");

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s (errno=%d)", path, errno);
        return ESP_FAIL;
    }

    uint8_t buffer[256];
    size_t to_read = max_bytes > sizeof(buffer) ? sizeof(buffer) : max_bytes;
    size_t read_bytes = fread(buffer, 1, to_read, f);
    fclose(f);

    ESP_LOGI(TAG, "First %zu bytes of %s:", read_bytes, name);
    for (size_t offset = 0; offset < read_bytes; offset += 16) {
        size_t line_len = (read_bytes - offset) > 16 ? 16 : (read_bytes - offset);
        printf("%04X: ", (unsigned int)offset);
        print_hex_line(&buffer[offset], line_len);
    }

    return ESP_OK;
}

esp_err_t sdcard_touch(const char *name, size_t size_bytes)
{
    if (!s_mounted) {
        ESP_LOGE(TAG, "SD card is not mounted");
        return ESP_ERR_INVALID_STATE;
    }
    char path[256];
    ESP_RETURN_ON_ERROR(build_path(name, path, sizeof(path)), TAG, "Path build failed");

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to create %s (errno=%d)", path, errno);
        return ESP_FAIL;
    }

    const size_t chunk = 512;
    uint8_t zeros[chunk];
    memset(zeros, 0, sizeof(zeros));
    size_t remaining = size_bytes;
    while (remaining > 0) {
        size_t to_write = remaining > chunk ? chunk : remaining;
        size_t written = fwrite(zeros, 1, to_write, f);
        if (written != to_write) {
            ESP_LOGE(TAG, "Write failed at %zu bytes", size_bytes - remaining);
            fclose(f);
            return ESP_FAIL;
        }
        remaining -= written;
    }

    fclose(f);
    ESP_LOGI(TAG, "Created %s (%zu bytes)", name, size_bytes);
    return ESP_OK;
}

static void fill_pattern(uint8_t *buf, size_t len, uint32_t seed, size_t offset)
{
    for (size_t i = 0; i < len; i += 4) {
        uint32_t v = seed ^ (uint32_t)((offset + i) * 0x45d9f3b);
        if (i + 4 <= len) {
            memcpy(buf + i, &v, 4);
        } else {
            size_t remain = len - i;
            memcpy(buf + i, &v, remain);
        }
    }
}

esp_err_t sdcard_self_test(size_t size_mb, uint32_t freq_khz)
{
    if (size_mb == 0) {
        size_mb = 10;
    }
    if (freq_khz != 0 && !is_supported_freq(freq_khz)) {
        ESP_LOGW(TAG, "Unsupported freq for test: %u kHz", freq_khz);
        return ESP_ERR_INVALID_ARG;
    }

    if (freq_khz != 0) {
        ESP_RETURN_ON_ERROR(sdcard_set_frequency(freq_khz, sdcard_is_mounted()), TAG, "Set freq failed");
    }

    ESP_RETURN_ON_ERROR(sdcard_mount(), TAG, "Mount failed");

    const size_t total_bytes = size_mb * 1024 * 1024;
    uint8_t buffer[SDTEST_BLOCK_SIZE];
    const uint32_t seed = 0xA5A5F00Du;

    ESP_LOGI(TAG, "SDTEST start: %zu MB at %u kHz", size_mb, s_current_freq_khz);

    FILE *f = fopen(SDTEST_FILE_PATH, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s for write", SDTEST_FILE_PATH);
        return ESP_FAIL;
    }

    int fd = fileno(f);
    size_t written_total = 0;
    int64_t write_start = esp_timer_get_time();
    while (written_total < total_bytes) {
        size_t offset = written_total;
        size_t to_write = (total_bytes - written_total) > SDTEST_BLOCK_SIZE ? SDTEST_BLOCK_SIZE : (total_bytes - written_total);
        fill_pattern(buffer, to_write, seed, offset);
        size_t wrote = fwrite(buffer, 1, to_write, f);
        if (wrote != to_write) {
            ESP_LOGE(TAG, "Write failed at offset %zu", written_total);
            fclose(f);
            return ESP_FAIL;
        }
        written_total += wrote;
    }
    fflush(f);
    fsync(fd);
    fclose(f);
    int64_t write_end = esp_timer_get_time();

    f = fopen(SDTEST_FILE_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s for read", SDTEST_FILE_PATH);
        return ESP_FAIL;
    }

    size_t read_total = 0;
    int64_t read_start = esp_timer_get_time();
    while (read_total < total_bytes) {
        size_t offset = read_total;
        size_t to_read = (total_bytes - read_total) > SDTEST_BLOCK_SIZE ? SDTEST_BLOCK_SIZE : (total_bytes - read_total);
        size_t got = fread(buffer, 1, to_read, f);
        if (got != to_read) {
            ESP_LOGE(TAG, "Read failed at offset %zu", read_total);
            fclose(f);
            return ESP_FAIL;
        }

        uint8_t expected[SDTEST_BLOCK_SIZE];
        fill_pattern(expected, to_read, seed, offset);
        if (memcmp(buffer, expected, to_read) != 0) {
            ESP_LOGE(TAG, "SDTEST FAIL at offset %zu", read_total);
            fclose(f);
            return ESP_FAIL;
        }
        read_total += got;
    }
    fclose(f);
    int64_t read_end = esp_timer_get_time();

    double write_time_s = (double)(write_end - write_start) / 1000000.0;
    double read_time_s = (double)(read_end - read_start) / 1000000.0;
    double kb_total = (double)total_bytes / 1024.0;

    ESP_LOGI(TAG, "SDTEST PASS size=%zu MB write=%.1f KB/s read=%.1f KB/s",
             size_mb,
             kb_total / write_time_s,
             kb_total / read_time_s);

    unlink(SDTEST_FILE_PATH);
    return ESP_OK;
}
