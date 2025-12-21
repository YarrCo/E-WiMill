#include "cli.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdcard.h"
#include "wimill_pins.h"

#define TAG "CLI"
#define CLI_STACK_SIZE 4096
#define CLI_PRIORITY 5
#define CLI_MAX_LINE_LEN 128
#define CLI_MAX_ARGS 4
#define CLI_DEFAULT_CAT_BYTES 256
#define CLI_DEFAULT_SDTEST_MB 10

static void cli_task(void *arg);
static void trim_newline(char *line);
static int parse_args(char *line, char *argv[], size_t max_args);

void cli_print_help(void)
{
    printf("Commands:\n");
    printf("  help              - this help\n");
    printf("  ls                - list files in /sdcard\n");
    printf("  info              - show total/free space\n");
    printf("  rm <name>         - remove file\n");
    printf("  mkdir <dir>       - create directory\n");
    printf("  cat <name>        - show first %d bytes (hex+ascii)\n", CLI_DEFAULT_CAT_BYTES);
    printf("  touch <name> <n>  - create file with n zero bytes\n");
    printf("  mount             - retry SD mount\n");
    printf("  sd freq [kHz]     - show/set SD SPI freq (1000/4000/10000)\n");
    printf("  sdtest [mb] [kHz] - write+verify file (default 10 MB, optional freq)\n");
}

static void print_prompt(void)
{
    printf("> ");
    fflush(stdout);
}

esp_err_t cli_start(void)
{
    static bool started = false;
    if (started) {
        return ESP_OK;
    }

    const uart_config_t uart_config = {
        .baud_rate = WIMILL_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(UART_NUM_0, &uart_config);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return err;
    }

    uart_vfs_dev_use_driver(UART_NUM_0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);

    BaseType_t created = xTaskCreate(cli_task, "cli_task", CLI_STACK_SIZE, NULL, CLI_PRIORITY, NULL);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CLI task");
        return ESP_FAIL;
    }

    started = true;
    return ESP_OK;
}

static void handle_ls(void)
{
    if (sdcard_is_mounted()) {
        sdcard_list_root();
    } else {
        ESP_LOGW(TAG, "SD not mounted. Use 'mount' after fixing wiring.");
    }
}

static void handle_info(void)
{
    sdcard_status_t st;
    if (sdcard_get_status(&st) != ESP_OK) {
        return;
    }

    ESP_LOGI(TAG, "Status: %s", st.mounted ? "mounted" : "unmounted");
    ESP_LOGI(TAG, "Freq: current=%u kHz default=%u kHz", st.current_freq_khz, st.default_freq_khz);
    if (st.mounted) {
        double total_mb = (double)st.total_bytes / (1024.0 * 1024.0);
        double free_mb = (double)st.free_bytes / (1024.0 * 1024.0);
        ESP_LOGI(TAG, "Space: total=%.2f MB, free=%.2f MB", total_mb, free_mb);
        if (st.card_name[0] != '\0') {
            ESP_LOGI(TAG, "Card: %s", st.card_name);
        }
    }
}

static void handle_rm(const char *name)
{
    if (!name) {
        ESP_LOGW(TAG, "Usage: rm <name>");
        return;
    }
    sdcard_remove(name);
}

static void handle_mkdir(const char *name)
{
    if (!name) {
        ESP_LOGW(TAG, "Usage: mkdir <dir>");
        return;
    }
    sdcard_mkdir(name);
}

static void handle_cat(const char *name)
{
    if (!name) {
        ESP_LOGW(TAG, "Usage: cat <name>");
        return;
    }
    sdcard_cat(name, CLI_DEFAULT_CAT_BYTES);
}

static void handle_touch(const char *name, const char *size_str)
{
    if (!name || !size_str) {
        ESP_LOGW(TAG, "Usage: touch <name> <bytes>");
        return;
    }
    char *end = NULL;
    long bytes = strtol(size_str, &end, 10);
    if (end == size_str || bytes <= 0) {
        ESP_LOGW(TAG, "Invalid size: %s", size_str);
        return;
    }
    sdcard_touch(name, (size_t)bytes);
}

static void handle_mount(void)
{
    if (sdcard_is_mounted()) {
        ESP_LOGI(TAG, "Already mounted at %s", sdcard_mount_point());
        return;
    }
    esp_err_t err = sdcard_mount();
    if (err == ESP_OK) {
        sd_space_info_t info;
        if (sdcard_get_space(&info) == ESP_OK) {
            double total_mb = (double)info.total_bytes / (1024.0 * 1024.0);
            double free_mb = (double)info.free_bytes / (1024.0 * 1024.0);
            ESP_LOGI(TAG, "Mounted. Space: total=%.2f MB, free=%.2f MB", total_mb, free_mb);
        }
    }
}

static void handle_sd_freq(int argc, char *argv[])
{
    if (argc < 2 || strcmp(argv[1], "freq") != 0) {
        ESP_LOGW(TAG, "Usage: sd freq [1000|4000|10000]");
        return;
    }

    if (argc == 2) {
        ESP_LOGI(TAG, "SD freq current=%u kHz default=%u kHz",
                 sdcard_get_current_freq_khz(), sdcard_get_default_freq_khz());
        return;
    }

    uint32_t freq = (uint32_t)strtoul(argv[2], NULL, 10);
    esp_err_t err = sdcard_set_frequency(freq, sdcard_is_mounted());
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SD freq set to %u kHz%s",
                 freq,
                 sdcard_is_mounted() ? " (remounted)" : " (applies on next mount)");
    }
}

static void handle_sdtest(int argc, char *argv[])
{
    size_t size_mb = CLI_DEFAULT_SDTEST_MB;
    uint32_t freq = 0;

    if (argc > 1) {
        long v = strtol(argv[1], NULL, 10);
        if (v > 0) {
            size_mb = (size_t)v;
        }
    }
    if (argc > 2) {
        freq = (uint32_t)strtoul(argv[2], NULL, 10);
    }

    esp_err_t err = sdcard_self_test(size_mb, freq);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SDTEST failed (err=%s)", esp_err_to_name(err));
    }
}

static void execute_command(int argc, char *argv[])
{
    if (argc <= 0) {
        return;
    }

    const char *cmd = argv[0];
    if (strcmp(cmd, "help") == 0) {
        cli_print_help();
    } else if (strcmp(cmd, "ls") == 0) {
        handle_ls();
    } else if (strcmp(cmd, "info") == 0) {
        handle_info();
    } else if (strcmp(cmd, "rm") == 0) {
        handle_rm(argc > 1 ? argv[1] : NULL);
    } else if (strcmp(cmd, "mkdir") == 0) {
        handle_mkdir(argc > 1 ? argv[1] : NULL);
    } else if (strcmp(cmd, "cat") == 0) {
        handle_cat(argc > 1 ? argv[1] : NULL);
    } else if (strcmp(cmd, "touch") == 0) {
        handle_touch(argc > 1 ? argv[1] : NULL, argc > 2 ? argv[2] : NULL);
    } else if (strcmp(cmd, "mount") == 0) {
        handle_mount();
    } else if (strcmp(cmd, "sd") == 0) {
        handle_sd_freq(argc, argv);
    } else if (strcmp(cmd, "sdtest") == 0) {
        handle_sdtest(argc, argv);
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd);
        cli_print_help();
    }
}

static void trim_newline(char *line)
{
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        line[len - 1] = '\0';
        len--;
    }
}

static int parse_args(char *line, char *argv[], size_t max_args)
{
    int argc = 0;
    char *save = NULL;
    char *token = strtok_r(line, " ", &save);
    while (token && argc < (int)max_args) {
        argv[argc++] = token;
        token = strtok_r(NULL, " ", &save);
    }
    return argc;
}

static void cli_task(void *arg)
{
    (void)arg;
    char line[CLI_MAX_LINE_LEN];
    char *argv[CLI_MAX_ARGS];

    ESP_LOGI(TAG, "CLI ready. Type 'help' for commands.");
    for (;;) {
        print_prompt();
        if (!fgets(line, sizeof(line), stdin)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        trim_newline(line);
        if (line[0] == '\0') {
            continue;
        }

        int argc = parse_args(line, argv, CLI_MAX_ARGS);
        execute_command(argc, argv);
    }
}
