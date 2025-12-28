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
#include "freertos/queue.h"
#include "freertos/task.h"

#include "msc.h"
#include "sdcard.h"
#include "web_fs.h"
#include "wimill_pins.h"

#define TAG "CLI"
#define CLI_STACK_SIZE 4096
#define CLI_PRIORITY 5
#define CLI_MAX_LINE_LEN 128
#define CLI_MAX_ARGS 8
#define CLI_DEFAULT_CAT_BYTES 256
#define CLI_DEFAULT_SDTEST_MB 10
#define CLI_DEFAULT_SDTEST_BUF WIMILL_SDTEST_BUF_SZ
#define CLI_DEFAULT_SDBENCH_MB 1
#define CLI_DEFAULT_SDBENCH_BUF 4096

#define FILEOP_QUEUE_LEN 4
#define FILEOP_TASK_STACK 4096
#define FILEOP_TASK_PRIO 4

typedef enum {
    FILEOP_TOUCH,
    FILEOP_SDTEST,
    FILEOP_SDBENCH,
} fileop_type_t;

typedef struct {
    fileop_type_t type;
    char path[128];
    size_t size_bytes;
    size_t size_mb;
    uint32_t freq_khz;
    size_t buf_bytes;
} fileop_t;

static QueueHandle_t s_fileop_queue = NULL;
static TaskHandle_t s_fileop_task = NULL;
static bool s_fileop_busy = false;
static bool s_switching = false;

static void cli_task(void *arg);
static void fileop_task(void *arg);
static void trim_newline(char *line);
static int parse_args(char *line, char *argv[], size_t max_args);

static const char *msc_state_str(msc_state_t st)
{
    switch (st) {
    case MSC_STATE_USB_ATTACHED:
        return "ATTACHED";
    case MSC_STATE_USB_DETACHED:
        return "DETACHED";
    case MSC_STATE_ERROR:
    default:
        return "ERROR";
    }
}

void cli_print_help(void)
{
    printf("Commands:\n");
    printf("  help                - this help\n");
    printf("  ls [path]           - list files in /sdcard\n");
    printf("  info                - show total/free space\n");
    printf("  rm <name>           - remove file\n");
    printf("  mkdir <dir>         - create directory\n");
    printf("  cat <name>          - show first %d bytes (hex+ascii)\n", CLI_DEFAULT_CAT_BYTES);
    printf("  touch <name> <n>    - create file with n zero bytes (queued)\n");
    printf("  sdtest [mb] [kHz] [buf N] - write+verify file (queued)\n");
    printf("  sdbench [mb] [buf N] - write+read speed test (queued)\n");
    printf("  sd freq [kHz]       - show/set SD SPI freq (20000..40000)\n");
    printf("  usb status|attach|detach  - manage MSC state\n");
}

static void print_prompt(void)
{
    printf("> ");
    fflush(stdout);
}

static bool fileop_is_busy(void)
{
    if (s_fileop_busy) {
        return true;
    }
    if (web_fs_is_busy()) {
        return true;
    }
    if (!s_fileop_queue) {
        return false;
    }
    return uxQueueMessagesWaiting(s_fileop_queue) > 0;
}

static bool ensure_vfs_ready(void)
{
    if (s_switching) {
        ESP_LOGW(TAG, "BUSY: switching USB state");
        return false;
    }
    if (msc_get_state() == MSC_STATE_USB_ATTACHED) {
        ESP_LOGW(TAG, "BUSY: detach first");
        return false;
    }
    if (!sdcard_is_mounted()) {
        ESP_LOGW(TAG, "SD not mounted. Run 'usb detach'.");
        return false;
    }
    return true;
}

static void fileop_task(void *arg)
{
    (void)arg;
    fileop_t op;
    for (;;) {
        if (xQueueReceive(s_fileop_queue, &op, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        s_fileop_busy = true;
        if (msc_get_state() == MSC_STATE_USB_ATTACHED || !sdcard_is_mounted()) {
            ESP_LOGW(TAG, "File-op skipped: USB attached or SD not mounted");
            s_fileop_busy = false;
            continue;
        }

        switch (op.type) {
        case FILEOP_TOUCH: {
            ESP_LOGI(TAG, "touch start: %s (%u bytes)", op.path, (unsigned)op.size_bytes);
            esp_err_t err = sdcard_touch(op.path, op.size_bytes);
            ESP_LOGI(TAG, "touch done: %s", esp_err_to_name(err));
            break;
        }
        case FILEOP_SDTEST: {
            ESP_LOGI(TAG, "sdtest start: %u MB, freq=%u kHz, buf=%u",
                     (unsigned)op.size_mb, (unsigned)op.freq_khz, (unsigned)op.buf_bytes);
            esp_err_t err = sdcard_self_test(op.size_mb, op.freq_khz, op.buf_bytes);
            ESP_LOGI(TAG, "sdtest done: %s", esp_err_to_name(err));
            break;
        }
        case FILEOP_SDBENCH: {
            ESP_LOGI(TAG, "sdbench start: %u MB, buf=%u", (unsigned)op.size_mb, (unsigned)op.buf_bytes);
            esp_err_t err = sdcard_bench(op.size_mb, op.buf_bytes);
            ESP_LOGI(TAG, "sdbench done: %s", esp_err_to_name(err));
            break;
        }
        default:
            break;
        }
        s_fileop_busy = false;
    }
}

static void fileop_init(void)
{
    if (s_fileop_queue) {
        return;
    }
    s_fileop_queue = xQueueCreate(FILEOP_QUEUE_LEN, sizeof(fileop_t));
    if (!s_fileop_queue) {
        ESP_LOGE(TAG, "Failed to create file-op queue");
        return;
    }
    BaseType_t created = xTaskCreatePinnedToCore(
        fileop_task, "fileop_task", FILEOP_TASK_STACK, NULL, FILEOP_TASK_PRIO, &s_fileop_task, 0);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create file-op task");
    }
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

    fileop_init();

    BaseType_t created = xTaskCreate(cli_task, "cli_task", CLI_STACK_SIZE, NULL, CLI_PRIORITY, NULL);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CLI task");
        return ESP_FAIL;
    }

    started = true;
    return ESP_OK;
}

static void handle_ls(const char *path)
{
    if (!ensure_vfs_ready()) {
        return;
    }
    esp_err_t err = sdcard_list(path);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ls failed: %s", esp_err_to_name(err));
    }
}

static void handle_info(void)
{
    if (!ensure_vfs_ready()) {
        return;
    }
    sdcard_status_t st;
    if (sdcard_get_status(&st) != ESP_OK) {
        ESP_LOGE(TAG, "info failed");
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
    if (!ensure_vfs_ready()) {
        return;
    }
    esp_err_t err = sdcard_remove(name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rm failed: %s", esp_err_to_name(err));
    }
}

static void handle_mkdir(const char *name)
{
    if (!name) {
        ESP_LOGW(TAG, "Usage: mkdir <dir>");
        return;
    }
    if (!ensure_vfs_ready()) {
        return;
    }
    esp_err_t err = sdcard_mkdir(name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mkdir failed: %s", esp_err_to_name(err));
    }
}

static void handle_cat(const char *name)
{
    if (!name) {
        ESP_LOGW(TAG, "Usage: cat <name>");
        return;
    }
    if (!ensure_vfs_ready()) {
        return;
    }
    esp_err_t err = sdcard_cat(name, CLI_DEFAULT_CAT_BYTES);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cat failed: %s", esp_err_to_name(err));
    }
}

static void handle_touch(const char *name, const char *size_str)
{
    if (!name || !size_str) {
        ESP_LOGW(TAG, "Usage: touch <name> <bytes>");
        return;
    }
    if (!ensure_vfs_ready()) {
        return;
    }
    char *end = NULL;
    long bytes = strtol(size_str, &end, 10);
    if (end == size_str || bytes <= 0) {
        ESP_LOGW(TAG, "Invalid size: %s", size_str);
        return;
    }

    fileop_t op = {0};
    op.type = FILEOP_TOUCH;
    strncpy(op.path, name, sizeof(op.path) - 1);
    op.size_bytes = (size_t)bytes;

    if (!s_fileop_queue) {
        ESP_LOGW(TAG, "File-op queue not ready");
        return;
    }
    if (xQueueSend(s_fileop_queue, &op, 0) != pdTRUE) {
        ESP_LOGW(TAG, "File-op queue full");
        return;
    }
    ESP_LOGI(TAG, "touch queued: %s (%ld bytes)", op.path, bytes);
}

static void handle_sd_freq(int argc, char *argv[])
{
    if (argc < 2 || strcmp(argv[1], "freq") != 0) {
        ESP_LOGW(TAG, "Usage: sd freq [20000..40000]");
        return;
    }
    if (msc_get_state() == MSC_STATE_USB_ATTACHED) {
        ESP_LOGW(TAG, "BUSY: detach first");
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
    } else {
        ESP_LOGE(TAG, "SD freq set failed: %s", esp_err_to_name(err));
    }
}

static void handle_sdtest(int argc, char *argv[])
{
    if (!ensure_vfs_ready()) {
        return;
    }
    size_t size_mb = CLI_DEFAULT_SDTEST_MB;
    uint32_t freq = 0;
    size_t buf_bytes = CLI_DEFAULT_SDTEST_BUF;
    bool size_set = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "buf") == 0 && (i + 1) < argc) {
            long v = strtol(argv[i + 1], NULL, 10);
            if (v > 0) {
                buf_bytes = (size_t)v;
            } else {
                ESP_LOGW(TAG, "Invalid buf size: %s", argv[i + 1]);
            }
            ++i;
            continue;
        }

        char *end = NULL;
        long v = strtol(argv[i], &end, 10);
        if (end != argv[i] && v > 0) {
            if (!size_set) {
                size_mb = (size_t)v;
                size_set = true;
            } else {
                freq = (uint32_t)v;
            }
        }
    }

    fileop_t op = {0};
    op.type = FILEOP_SDTEST;
    op.size_mb = size_mb;
    op.freq_khz = freq;
    op.buf_bytes = buf_bytes;

    if (!s_fileop_queue) {
        ESP_LOGW(TAG, "File-op queue not ready");
        return;
    }
    if (xQueueSend(s_fileop_queue, &op, 0) != pdTRUE) {
        ESP_LOGW(TAG, "File-op queue full");
        return;
    }
    ESP_LOGI(TAG, "sdtest queued");
}

static void handle_sdbench(int argc, char *argv[])
{
    if (!ensure_vfs_ready()) {
        return;
    }
    size_t size_mb = CLI_DEFAULT_SDBENCH_MB;
    size_t buf_bytes = CLI_DEFAULT_SDBENCH_BUF;
    bool size_set = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "buf") == 0 && (i + 1) < argc) {
            long v = strtol(argv[i + 1], NULL, 10);
            if (v > 0) {
                buf_bytes = (size_t)v;
            } else {
                ESP_LOGW(TAG, "Invalid buf size: %s", argv[i + 1]);
            }
            ++i;
            continue;
        }

        char *end = NULL;
        long v = strtol(argv[i], &end, 10);
        if (end != argv[i] && v > 0) {
            if (!size_set) {
                size_mb = (size_t)v;
                size_set = true;
            }
        }
    }

    fileop_t op = {0};
    op.type = FILEOP_SDBENCH;
    op.size_mb = size_mb;
    op.buf_bytes = buf_bytes;

    if (!s_fileop_queue) {
        ESP_LOGW(TAG, "File-op queue not ready");
        return;
    }
    if (xQueueSend(s_fileop_queue, &op, 0) != pdTRUE) {
        ESP_LOGW(TAG, "File-op queue full");
        return;
    }
    ESP_LOGI(TAG, "sdbench queued");
}

static void handle_usb(int argc, char *argv[])
{
    if (argc < 2) {
        ESP_LOGW(TAG, "Usage: usb status|attach|detach");
        return;
    }
    const char *sub = argv[1];
    if (strcmp(sub, "status") == 0) {
        ESP_LOGI(TAG, "USB=%s, VFS=%s", msc_state_str(msc_get_state()),
                 sdcard_is_mounted() ? "mounted" : "unmounted");
        return;
    }

    if (strcmp(sub, "attach") == 0) {
        if (fileop_is_busy()) {
            ESP_LOGW(TAG, "BUSY: file ops running");
            return;
        }
        s_switching = true;
        esp_err_t err = msc_attach();
        s_switching = false;
        ESP_LOGI(TAG, "ATTACHED: %s", esp_err_to_name(err));
        return;
    }

    if (strcmp(sub, "detach") == 0) {
        if (fileop_is_busy()) {
            ESP_LOGW(TAG, "BUSY: file ops running");
            return;
        }
        s_switching = true;
        esp_err_t err = msc_detach();
        s_switching = false;
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "DETACHED ok, %s mounted", sdcard_mount_point());
        } else {
            ESP_LOGI(TAG, "DETACHED failed: %s", esp_err_to_name(err));
        }
        return;
    }

    ESP_LOGW(TAG, "Unknown usb subcommand");
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
        handle_ls(argc > 1 ? argv[1] : NULL);
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
    } else if (strcmp(cmd, "sd") == 0) {
        handle_sd_freq(argc, argv);
    } else if (strcmp(cmd, "sdtest") == 0) {
        handle_sdtest(argc, argv);
    } else if (strcmp(cmd, "sdbench") == 0) {
        handle_sdbench(argc, argv);
    } else if (strcmp(cmd, "usb") == 0) {
        handle_usb(argc, argv);
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
