#include "sdcard.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "wimill_pins.h"

#define TAG "SDCARD"
#define DEFAULT_ALLOC_UNIT (32 * 1024)
#define SDTEST_FILE_PATH "/sdcard/.wimill_sdtest.bin"
#define SDTEST_BLOCK_MIN 4096

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;
static spi_host_device_t s_host_id = SPI2_HOST;
static uint32_t s_current_freq_khz = WIMILL_SD_FREQ_KHZ_DEFAULT;
static SemaphoreHandle_t s_fs_mutex = NULL;
static TaskHandle_t s_sdtest_task = NULL;
static size_t s_sdtest_buf_bytes = WIMILL_SDTEST_BUF_SZ;

static bool is_supported_freq(uint32_t khz)
{
    return khz == WIMILL_SD_FREQ_KHZ_DEFAULT ||
           khz == WIMILL_SD_FREQ_KHZ_20MHZ ||
           khz == WIMILL_SD_FREQ_KHZ_26MHZ;
}

static bool ensure_mutex(void)
{
    if (s_fs_mutex) {
        return true;
    }
    s_fs_mutex = xSemaphoreCreateRecursiveMutex();
    if (!s_fs_mutex) {
        ESP_LOGE(TAG, "Failed to create SD mutex");
        return false;
    }
    return true;
}

void sdcard_lock(void)
{
    if (!ensure_mutex()) {
        return;
    }
    xSemaphoreTakeRecursive(s_fs_mutex, portMAX_DELAY);
}

void sdcard_unlock(void)
{
    if (s_fs_mutex) {
        xSemaphoreGiveRecursive(s_fs_mutex);
    }
}

esp_err_t sdcard_init_raw(sdmmc_card_t **out_card)
{
    if (!out_card) {
        return ESP_ERR_INVALID_ARG;
    }
    sdcard_lock();

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = s_host_id;
    host.max_freq_khz = s_current_freq_khz;

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
        sdcard_unlock();
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = WIMILL_PIN_SD_CS;
    slot_config.host_id = host.slot;

    sdspi_dev_handle_t dev_handle = 0;
    ret = sdspi_host_init_device(&slot_config, &dev_handle);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        sdcard_unlock();
        return ret;
    }

    sdmmc_card_t *card = calloc(1, sizeof(sdmmc_card_t));
    if (!card) {
        sdcard_unlock();
        return ESP_ERR_NO_MEM;
    }
    ret = sdmmc_card_init(&host, card);
    if (ret != ESP_OK) {
        free(card);
        sdcard_unlock();
        return ret;
    }

    s_card = card;
    *out_card = card;

    char name[8] = {0};
    memcpy(name, s_card->cid.name, sizeof(s_card->cid.name));
    double size_mb = ((double)s_card->csd.capacity) * s_card->csd.sector_size / (1024.0 * 1024.0);
    ESP_LOGI(TAG, "SD raw init OK: %s size=%.2f MB freq=%u kHz", name, size_mb, s_current_freq_khz);

    sdcard_unlock();
    return ESP_OK;
}

uint32_t sdcard_get_current_freq_khz(void) { return s_current_freq_khz; }
uint32_t sdcard_get_default_freq_khz(void) { return WIMILL_SD_FREQ_KHZ_DEFAULT; }

esp_err_t sdcard_set_frequency(uint32_t freq_khz, bool remount)
{
    (void)remount;
    sdcard_lock();
    if (!is_supported_freq(freq_khz)) {
        sdcard_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    s_current_freq_khz = freq_khz;
    sdcard_unlock();
    return ESP_OK;
}

esp_err_t sdcard_mount(void)
{
    sdcard_lock();
    if (s_mounted) {
        sdcard_unlock();
        return ESP_OK;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = s_host_id;
    host.max_freq_khz = s_current_freq_khz;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = WIMILL_PIN_SD_MOSI,
        .miso_io_num = WIMILL_PIN_SD_MISO,
        .sclk_io_num = WIMILL_PIN_SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = WIMILL_PIN_SD_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = DEFAULT_ALLOC_UNIT,
        .disk_status_check_enable = true,
    };

    esp_err_t ret = esp_vfs_fat_sdspi_mount(WIMILL_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret == ESP_OK) {
        s_mounted = true;
    }
    sdcard_unlock();
    return ret;
}

esp_err_t sdcard_unmount(void)
{
    sdcard_lock();
    if (!s_mounted) {
        sdcard_unlock();
        return ESP_OK;
    }
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(WIMILL_SD_MOUNT_POINT, s_card);
    s_mounted = false;
    sdcard_unlock();
    return ret;
}

bool sdcard_is_mounted(void) { return s_mounted; }
const char *sdcard_mount_point(void) { return WIMILL_SD_MOUNT_POINT; }

void sdcard_set_mounted(bool mounted)
{
    sdcard_lock();
    s_mounted = mounted;
    sdcard_unlock();
}

esp_err_t sdcard_get_space(sd_space_info_t *info)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }
    sdcard_lock();
    if (!s_mounted) {
        sdcard_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    FATFS *fs = NULL;
    DWORD free_clusters = 0;
    FRESULT res = f_getfree(WIMILL_SD_MOUNT_POINT, &free_clusters, &fs);
    if (res != FR_OK || !fs) {
        sdcard_unlock();
        return ESP_FAIL;
    }

    uint64_t cluster_size = ((uint64_t)fs->csize) * 512;
    info->total_bytes = ((uint64_t)(fs->n_fatent - 2)) * cluster_size;
    info->free_bytes = ((uint64_t)free_clusters) * cluster_size;
    sdcard_unlock();
    return ESP_OK;
}

esp_err_t sdcard_get_status(sdcard_status_t *out_status)
{
    if (!out_status) {
        return ESP_ERR_INVALID_ARG;
    }
    sdcard_lock();
    memset(out_status, 0, sizeof(*out_status));
    out_status->mounted = s_mounted;
    out_status->current_freq_khz = s_current_freq_khz;
    out_status->default_freq_khz = WIMILL_SD_FREQ_KHZ_DEFAULT;
    out_status->allocation_unit = DEFAULT_ALLOC_UNIT;
    out_status->sdtest_buf_bytes = (uint32_t)s_sdtest_buf_bytes;

    if (s_mounted && s_card) {
        memcpy(out_status->card_name, s_card->cid.name, sizeof(s_card->cid.name));
        sd_space_info_t space;
        if (sdcard_get_space(&space) == ESP_OK) {
            out_status->total_bytes = space.total_bytes;
            out_status->free_bytes = space.free_bytes;
        }
    }
    sdcard_unlock();
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
    sdcard_lock();
    if (!s_mounted) {
        sdcard_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    DIR *dir = opendir(WIMILL_SD_MOUNT_POINT);
    if (!dir) {
        sdcard_unlock();
        return ESP_FAIL;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        char full_path[256];
        if (build_path(entry->d_name, full_path, sizeof(full_path)) != ESP_OK) {
            continue;
        }
        struct stat st;
        if (stat(full_path, &st) != 0) {
            continue;
        }
        if (S_ISDIR(st.st_mode)) {
            ESP_LOGI(TAG, "<DIR>  %s", entry->d_name);
        } else {
            ESP_LOGI(TAG, "FILE   %s (%ld bytes)", entry->d_name, (long)st.st_size);
        }
    }

    closedir(dir);
    sdcard_unlock();
    return ESP_OK;
}

esp_err_t sdcard_remove(const char *name)
{
    sdcard_lock();
    if (!s_mounted) {
        sdcard_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    char path[256];
    if (build_path(name, path, sizeof(path)) != ESP_OK) {
        sdcard_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    struct stat st;
    if (stat(path, &st) != 0 || S_ISDIR(st.st_mode)) {
        sdcard_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    if (unlink(path) != 0) {
        sdcard_unlock();
        return ESP_FAIL;
    }
    sdcard_unlock();
    return ESP_OK;
}

esp_err_t sdcard_mkdir(const char *name)
{
    sdcard_lock();
    if (!s_mounted) {
        sdcard_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    char path[256];
    if (build_path(name, path, sizeof(path)) != ESP_OK) {
        sdcard_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    if (mkdir(path, 0777) != 0) {
        sdcard_unlock();
        return ESP_FAIL;
    }
    sdcard_unlock();
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
    sdcard_lock();
    if (!s_mounted) {
        sdcard_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    char path[256];
    if (build_path(name, path, sizeof(path)) != ESP_OK) {
        sdcard_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    FILE *f = fopen(path, "rb");
    if (!f) {
        sdcard_unlock();
        return ESP_FAIL;
    }
    uint8_t buffer[256];
    size_t to_read = max_bytes > sizeof(buffer) ? sizeof(buffer) : max_bytes;
    size_t read_bytes = fread(buffer, 1, to_read, f);
    fclose(f);
    for (size_t offset = 0; offset < read_bytes; offset += 16) {
        size_t line_len = (read_bytes - offset) > 16 ? 16 : (read_bytes - offset);
        printf("%04X: ", (unsigned int)offset);
        print_hex_line(&buffer[offset], line_len);
    }
    sdcard_unlock();
    return ESP_OK;
}

esp_err_t sdcard_touch(const char *name, size_t size_bytes)
{
    sdcard_lock();
    if (!s_mounted) {
        sdcard_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    char path[256];
    if (build_path(name, path, sizeof(path)) != ESP_OK) {
        sdcard_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    FILE *f = fopen(path, "wb");
    if (!f) {
        sdcard_unlock();
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
            fclose(f);
            sdcard_unlock();
            return ESP_FAIL;
        }
        remaining -= written;
    }
    fclose(f);
    sdcard_unlock();
    return ESP_OK;
}

static void fill_pattern(uint8_t *buf, size_t len, uint32_t seed, size_t offset)
{
    for (size_t i = 0; i < len; i += 4) {
        uint32_t v = seed ^ (uint32_t)((offset + i) * 0x45d9f3b);
        size_t remain = len - i;
        memcpy(buf + i, &v, remain >= 4 ? 4 : remain);
    }
}

static esp_err_t sdcard_run_self_test(size_t size_mb, uint32_t freq_khz, size_t buffer_bytes)
{
    if (size_mb == 0) {
        size_mb = 10;
    }
    sdcard_lock();

    if (freq_khz && is_supported_freq(freq_khz)) {
        s_current_freq_khz = freq_khz;
    }
    if (!s_mounted) {
        sdcard_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    const size_t total_bytes = size_mb * 1024 * 1024;
    const uint32_t seed = 0xA5A5F00Du;
    if (buffer_bytes < SDTEST_BLOCK_MIN) {
        buffer_bytes = SDTEST_BLOCK_MIN;
    }
    s_sdtest_buf_bytes = buffer_bytes;

    uint8_t *io_buf = heap_caps_malloc(buffer_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    uint8_t *exp_buf = heap_caps_malloc(buffer_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    if (!io_buf || !exp_buf) {
        if (io_buf) heap_caps_free(io_buf);
        if (exp_buf) heap_caps_free(exp_buf);
        sdcard_unlock();
        return ESP_ERR_NO_MEM;
    }

    FILE *f = fopen(SDTEST_FILE_PATH, "wb");
    if (!f) {
        heap_caps_free(io_buf);
        heap_caps_free(exp_buf);
        sdcard_unlock();
        return ESP_FAIL;
    }

    size_t written_total = 0;
    int fd = fileno(f);
    int64_t write_start = esp_timer_get_time();
    while (written_total < total_bytes) {
        size_t offset = written_total;
        size_t to_write = (total_bytes - written_total) > buffer_bytes ? buffer_bytes : (total_bytes - written_total);
        fill_pattern(io_buf, to_write, seed, offset);
        size_t wrote = fwrite(io_buf, 1, to_write, f);
        if (wrote != to_write || ferror(f)) {
            fclose(f);
            heap_caps_free(io_buf);
            heap_caps_free(exp_buf);
            sdcard_unlock();
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
        heap_caps_free(io_buf);
        heap_caps_free(exp_buf);
        sdcard_unlock();
        return ESP_FAIL;
    }

    size_t read_total = 0;
    int64_t read_start = esp_timer_get_time();
    while (read_total < total_bytes) {
        size_t offset = read_total;
        size_t to_read = (total_bytes - read_total) > buffer_bytes ? buffer_bytes : (total_bytes - read_total);
        size_t got = fread(io_buf, 1, to_read, f);
        if (got != to_read || ferror(f)) {
            fclose(f);
            heap_caps_free(io_buf);
            heap_caps_free(exp_buf);
            sdcard_unlock();
            return ESP_FAIL;
        }
        fill_pattern(exp_buf, to_read, seed, offset);
        if (memcmp(io_buf, exp_buf, to_read) != 0) {
            fclose(f);
            heap_caps_free(io_buf);
            heap_caps_free(exp_buf);
            sdcard_unlock();
            return ESP_FAIL;
        }
        read_total += got;
    }
    fclose(f);
    int64_t read_end = esp_timer_get_time();

    double write_time_s = (double)(write_end - write_start) / 1e6;
    double read_time_s = (double)(read_end - read_start) / 1e6;
    double kb_total = (double)total_bytes / 1024.0;
    ESP_LOGI(TAG, "SDTEST PASS size=%zu MB write=%.1f KB/s read=%.1f KB/s",
             size_mb, kb_total / write_time_s, kb_total / read_time_s);
    unlink(SDTEST_FILE_PATH);

    heap_caps_free(io_buf);
    heap_caps_free(exp_buf);
    sdcard_unlock();
    return ESP_OK;
}

typedef struct {
    size_t size_mb;
    uint32_t freq_khz;
    size_t buf_bytes;
} sdtest_args_t;

static void sdcard_self_test_task(void *arg)
{
    sdtest_args_t cfg = *(sdtest_args_t *)arg;
    free(arg);
    esp_err_t ret = sdcard_run_self_test(cfg.size_mb, cfg.freq_khz, cfg.buf_bytes);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SDTEST finished with error: %s", esp_err_to_name(ret));
    }
    s_sdtest_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t sdcard_self_test(size_t size_mb, uint32_t freq_khz, size_t buf_bytes)
{
    if (s_sdtest_task) {
        return ESP_ERR_INVALID_STATE;
    }
    sdtest_args_t *args = malloc(sizeof(sdtest_args_t));
    if (!args) {
        return ESP_ERR_NO_MEM;
    }
    args->size_mb = size_mb;
    args->freq_khz = freq_khz;
    args->buf_bytes = buf_bytes;

    BaseType_t res = xTaskCreatePinnedToCore(
        sdcard_self_test_task, "sdtest", 16384 / sizeof(StackType_t),
        args, 4, &s_sdtest_task, tskNO_AFFINITY);
    if (res != pdPASS) {
        free(args);
        s_sdtest_task = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}
