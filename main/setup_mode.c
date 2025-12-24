#include "setup_mode.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/inet.h"
#include "mdns.h"

#include "config_store.h"
#include "led_status.h"
#include "msc.h"
#include "sdcard.h"
#include "web_fs.h"

#define TAG "SETUP"
#define AP_PASS "wimill1234"
#define STA_CONNECT_TIMEOUT_MS 30000
#define MDNS_NAME_LIMIT 24

static bool s_active = false;
static bool s_wifi_inited = false;
static bool s_handlers_registered = false;
static bool s_mdns_inited = false;
static bool s_sta_connecting = false;
static bool s_sta_connected = false;
static bool s_sta_task_running = false;
static bool s_sta_only_mode = false;
static httpd_handle_t s_http = NULL;
static uint16_t s_http_port = 0;
static esp_netif_t *s_ap_netif = NULL;
static esp_netif_t *s_sta_netif = NULL;
static wifi_config_t s_ap_cfg = {0};
static bool s_ap_cfg_valid = false;
static char s_ap_ssid[32] = {0};
static char s_ap_ip[16] = {0};
static char s_sta_ip[16] = "0.0.0.0";
static char s_sta_error[32] = {0};
static char s_mdns_name[32] = {0};
static int s_sta_rssi = 0;
static esp_timer_handle_t s_sta_timer = NULL;
static esp_timer_handle_t s_apply_timer = NULL;
static wimill_config_t s_cfg;
static SemaphoreHandle_t s_cfg_mutex = NULL;
static SemaphoreHandle_t s_state_mutex = NULL;

static esp_err_t setup_http_start(void);
static bool start_sta_connect_async(void);
static void schedule_sta_connect(void);
static esp_err_t setup_wifi_init_base(void);
static esp_err_t setup_sta_only_start(void);

static void cfg_lock(void)
{
    if (s_cfg_mutex) {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    }
}

static void cfg_unlock(void)
{
    if (s_cfg_mutex) {
        xSemaphoreGive(s_cfg_mutex);
    }
}

static void state_lock(void)
{
    if (s_state_mutex) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }
}

static void state_unlock(void)
{
    if (s_state_mutex) {
        xSemaphoreGive(s_state_mutex);
    }
}

static void json_sanitize(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0;
    if (!dst || dst_len == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }
    for (size_t si = 0; src[si] != '\0' && di + 1 < dst_len; ++si) {
        unsigned char ch = (unsigned char)src[si];
        if (ch < 32) {
            continue;
        }
        if (ch == '"' || ch == '\\') {
            dst[di++] = '_';
            continue;
        }
        dst[di++] = (char)ch;
    }
    dst[di] = '\0';
}

static void set_sta_error(const char *msg)
{
    state_lock();
    if (msg) {
        strncpy(s_sta_error, msg, sizeof(s_sta_error));
        s_sta_error[sizeof(s_sta_error) - 1] = '\0';
    } else {
        s_sta_error[0] = '\0';
    }
    state_unlock();
}

static const char *usb_mode_str(void)
{
    msc_state_t st = msc_get_state();
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

static void sanitize_dev_name(const char *src, char *out, size_t out_len, const uint8_t mac[6])
{
    size_t di = 0;
    bool last_dash = false;

    if (!out || out_len == 0) {
        return;
    }

    for (size_t si = 0; src && src[si] != '\0' && di + 1 < out_len && di < MDNS_NAME_LIMIT; ++si) {
        char ch = (char)tolower((unsigned char)src[si]);
        if (ch == ' ' || ch == '_') {
            ch = '-';
        }
        if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '-')) {
            continue;
        }
        if (ch == '-') {
            if (di == 0 || last_dash) {
                continue;
            }
            last_dash = true;
        } else {
            last_dash = false;
        }
        out[di++] = ch;
    }
    if (di > 0 && out[di - 1] == '-') {
        di--;
    }
    out[di] = '\0';

    if (di == 0) {
        snprintf(out, out_len, "ewimill-%02x%02x", mac[4], mac[5]);
    }
}

static void update_mdns_name(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char name[32];
    cfg_lock();
    sanitize_dev_name(s_cfg.dev_name, name, sizeof(name), mac);
    cfg_unlock();

    state_lock();
    strncpy(s_mdns_name, name, sizeof(s_mdns_name));
    s_mdns_name[sizeof(s_mdns_name) - 1] = '\0';
    state_unlock();
}

static esp_err_t setup_mdns_start(uint16_t port)
{
    esp_err_t err = ESP_OK;
    if (!s_mdns_inited) {
        err = mdns_init();
        if (err != ESP_OK) {
            return err;
        }
        s_mdns_inited = true;
    }

    state_lock();
    char host[32];
    strncpy(host, s_mdns_name, sizeof(host));
    host[sizeof(host) - 1] = '\0';
    state_unlock();

    err = mdns_hostname_set(host);
    if (err != ESP_OK) {
        return err;
    }
    mdns_instance_name_set("E-WiMill");
    mdns_service_remove("_http", "_tcp");
    err = mdns_service_add(NULL, "_http", "_tcp", port, NULL, 0);
    return err;
}

static void sta_connect_timeout_cb(void *arg)
{
    (void)arg;
    state_lock();
    bool connected = s_sta_connected;
    state_unlock();

    if (connected) {
        return;
    }

    set_sta_error("timeout");
    state_lock();
    s_sta_connecting = false;
    state_unlock();

    esp_wifi_disconnect();
    if (s_sta_only_mode) {
        led_status_set_wifi(false);
        return;
    }
    if (s_ap_cfg_valid) {
        esp_wifi_set_mode(WIFI_MODE_AP);
        esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
    }
    led_status_set_setup(true);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        char err[32];
        snprintf(err, sizeof(err), "reason:%d", (int)disc->reason);
        set_sta_error(err);
        state_lock();
        s_sta_connected = false;
        s_sta_connecting = false;
        strncpy(s_sta_ip, "0.0.0.0", sizeof(s_sta_ip));
        state_unlock();
        if (s_sta_timer) {
            esp_timer_stop(s_sta_timer);
        }
        if (s_sta_only_mode) {
            led_status_set_wifi(false);
            return;
        }
        if (s_ap_cfg_valid) {
            esp_wifi_set_mode(WIFI_MODE_AP);
            esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
        }
        led_status_set_setup(true);
        return;
    }
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
        char ip_str[16];
        esp_ip4addr_ntoa(&evt->ip_info.ip, ip_str, sizeof(ip_str));

        state_lock();
        strncpy(s_sta_ip, ip_str, sizeof(s_sta_ip));
        s_sta_ip[sizeof(s_sta_ip) - 1] = '\0';
        s_sta_connected = true;
        s_sta_connecting = false;
        s_sta_error[0] = '\0';
        state_unlock();

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            state_lock();
            s_sta_rssi = ap_info.rssi;
            state_unlock();
        }

        if (s_sta_timer) {
            esp_timer_stop(s_sta_timer);
        }

        cfg_lock();
        strncpy(s_cfg.last_sta_ip, ip_str, sizeof(s_cfg.last_sta_ip));
        s_cfg.last_sta_ip[sizeof(s_cfg.last_sta_ip) - 1] = '\0';
        config_save(&s_cfg);
        uint16_t port = s_cfg.web_port ? s_cfg.web_port : 8080;
        bool boot_sta = (s_cfg.wifi_boot_mode == WIFI_BOOT_STA);
        cfg_unlock();

        update_mdns_name();
        setup_mdns_start(port);

        if (s_http && port != s_http_port) {
            httpd_stop(s_http);
            s_http = NULL;
            s_http_port = 0;
        }
        setup_http_start();

        if (boot_sta) {
            s_sta_only_mode = true;
            s_active = false;
        }

        esp_wifi_set_mode(WIFI_MODE_STA);
        led_status_set_wifi(true);
        led_status_set_setup(false);
        return;
    }
}

static esp_err_t setup_wifi_init_base(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (!s_wifi_inited) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
        if (err != ESP_OK) {
            return err;
        }
        s_wifi_inited = true;
    }

    if (!s_handlers_registered) {
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
        s_handlers_registered = true;
    }

    return ESP_OK;
}

static esp_err_t setup_wifi_start(void)
{
    esp_err_t err = setup_wifi_init_base();
    if (err != ESP_OK) {
        return err;
    }

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif) {
            return ESP_FAIL;
        }
    }

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "E-WiMill-%02X%02X", mac[4], mac[5]);

    memset(&s_ap_cfg, 0, sizeof(s_ap_cfg));
    strncpy((char *)s_ap_cfg.ap.ssid, s_ap_ssid, sizeof(s_ap_cfg.ap.ssid));
    s_ap_cfg.ap.ssid_len = strlen(s_ap_ssid);
    strncpy((char *)s_ap_cfg.ap.password, AP_PASS, sizeof(s_ap_cfg.ap.password));
    s_ap_cfg.ap.max_connection = 4;
    s_ap_cfg.ap.channel = 1;
    s_ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (strlen(AP_PASS) == 0) {
        s_ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    }
    s_ap_cfg_valid = true;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    esp_netif_ip_info_t ip;
    if (s_ap_netif && esp_netif_get_ip_info(s_ap_netif, &ip) == ESP_OK) {
        esp_ip4addr_ntoa(&ip.ip, s_ap_ip, sizeof(s_ap_ip));
    } else {
        strncpy(s_ap_ip, "192.168.4.1", sizeof(s_ap_ip));
        s_ap_ip[sizeof(s_ap_ip) - 1] = '\0';
    }

    cfg_lock();
    uint16_t port = s_cfg.web_port ? s_cfg.web_port : 8080;
    cfg_unlock();

    ESP_LOGI(TAG, "AP started: SSID=%s PASS=%s IP=%s PORT=%u", s_ap_ssid, AP_PASS, s_ap_ip, port);
    s_sta_only_mode = false;
    led_status_set_wifi(true);
    return ESP_OK;
}

static esp_err_t setup_sta_only_start(void)
{
    esp_err_t err = setup_wifi_init_base();
    if (err != ESP_OK) {
        return err;
    }

    if (!s_sta_netif) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (!s_sta_netif) {
            return ESP_FAIL;
        }
    }

    s_sta_only_mode = true;
    led_status_set_wifi(false);
    if (!start_sta_connect_async()) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t sta_connect_start(void)
{
    cfg_lock();
    char ssid[CONFIG_STA_SSID_LEN];
    char psk[CONFIG_STA_PSK_LEN];
    strncpy(ssid, s_cfg.sta_ssid, sizeof(ssid));
    ssid[sizeof(ssid) - 1] = '\0';
    strncpy(psk, s_cfg.sta_psk, sizeof(psk));
    psk[sizeof(psk) - 1] = '\0';
    cfg_unlock();

    if (ssid[0] == '\0') {
        set_sta_error("ssid_empty");
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_sta_netif) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (!s_sta_netif) {
            set_sta_error("sta_netif");
            return ESP_FAIL;
        }
    }

    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
    strncpy((char *)sta_cfg.sta.password, psk, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.scan_method = WIFI_FAST_SCAN;
    sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    esp_err_t err = esp_wifi_set_mode(s_sta_only_mode ? WIFI_MODE_STA : WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        set_sta_error("mode");
        return err;
    }
    if (!s_sta_only_mode && s_ap_cfg_valid) {
        esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
    }
    err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    if (err != ESP_OK) {
        set_sta_error("sta_cfg");
        return err;
    }
    if (s_sta_only_mode) {
        esp_err_t start_err = esp_wifi_start();
        if (start_err != ESP_OK && start_err != ESP_ERR_WIFI_CONN && start_err != ESP_ERR_INVALID_STATE) {
            set_sta_error("start");
            return start_err;
        }
    }
    esp_wifi_disconnect();
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        set_sta_error("connect");
        return err;
    }

    state_lock();
    s_sta_connecting = true;
    s_sta_connected = false;
    strncpy(s_sta_ip, "0.0.0.0", sizeof(s_sta_ip));
    s_sta_error[0] = '\0';
    s_sta_rssi = 0;
    state_unlock();

    if (!s_sta_timer) {
        const esp_timer_create_args_t args = {
            .callback = sta_connect_timeout_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "sta_timeout",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&args, &s_sta_timer);
    }
    if (s_sta_timer) {
        esp_timer_stop(s_sta_timer);
        esp_timer_start_once(s_sta_timer, (uint64_t)STA_CONNECT_TIMEOUT_MS * 1000ULL);
    }

    ESP_LOGI(TAG, "STA connect started: ssid=%s", ssid);
    return ESP_OK;
}

static void sta_connect_task(void *arg)
{
    (void)arg;
    sta_connect_start();
    state_lock();
    s_sta_task_running = false;
    state_unlock();
    vTaskDelete(NULL);
}

static bool start_sta_connect_async(void)
{
    state_lock();
    if (s_sta_task_running || s_sta_connecting) {
        state_unlock();
        return false;
    }
    s_sta_task_running = true;
    state_unlock();

    if (xTaskCreate(sta_connect_task, "sta_connect", 4096, NULL, 6, NULL) != pdPASS) {
        state_lock();
        s_sta_task_running = false;
        state_unlock();
        return false;
    }
    return true;
}

static void apply_timer_cb(void *arg)
{
    (void)arg;
    start_sta_connect_async();
}

static void schedule_sta_connect(void)
{
    s_sta_only_mode = false;
    if (!s_apply_timer) {
        const esp_timer_create_args_t args = {
            .callback = apply_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "sta_apply",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args, &s_apply_timer) != ESP_OK) {
            return;
        }
    }
    esp_timer_stop(s_apply_timer);
    esp_timer_start_once(s_apply_timer, 200 * 1000ULL);
}

static void url_decode(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0;
    size_t si = 0;
    while (src[si] && di + 1 < dst_len) {
        char ch = src[si];
        if (ch == '%' && isxdigit((unsigned char)src[si + 1]) && isxdigit((unsigned char)src[si + 2])) {
            char hex[3] = {src[si + 1], src[si + 2], '\0'};
            dst[di++] = (char)strtol(hex, NULL, 16);
            si += 3;
        } else if (ch == '+') {
            dst[di++] = ' ';
            si++;
        } else {
            dst[di++] = ch;
            si++;
        }
    }
    dst[di] = '\0';
}

static bool apply_config_form(const char *body, char *err, size_t err_len)
{
    bool ok = true;
    char *tmp = strdup(body);
    if (!tmp) {
        if (err) {
            strncpy(err, "no_mem", err_len);
            err[err_len - 1] = '\0';
        }
        return false;
    }

    wimill_config_t next;
    cfg_lock();
    next = s_cfg;
    cfg_unlock();

    char *saveptr = NULL;
    for (char *pair = strtok_r(tmp, "&", &saveptr); pair; pair = strtok_r(NULL, "&", &saveptr)) {
        char *eq = strchr(pair, '=');
        if (!eq) {
            continue;
        }
        *eq = '\0';
        char key[32] = {0};
        char val[96] = {0};
        url_decode(key, sizeof(key), pair);
        url_decode(val, sizeof(val), eq + 1);

        if (strcmp(key, "device_name") == 0) {
            strncpy(next.dev_name, val, sizeof(next.dev_name));
            next.dev_name[sizeof(next.dev_name) - 1] = '\0';
        } else if (strcmp(key, "sta_ssid") == 0) {
            strncpy(next.sta_ssid, val, sizeof(next.sta_ssid));
            next.sta_ssid[sizeof(next.sta_ssid) - 1] = '\0';
        } else if (strcmp(key, "sta_psk") == 0) {
            strncpy(next.sta_psk, val, sizeof(next.sta_psk));
            next.sta_psk[sizeof(next.sta_psk) - 1] = '\0';
        } else if (strcmp(key, "web_port") == 0) {
            int port = atoi(val);
            if (port > 0 && port < 65536) {
                next.web_port = (uint16_t)port;
            }
        } else if (strcmp(key, "wifi_boot") == 0) {
            char mode[8] = {0};
            size_t len = strlen(val);
            if (len >= sizeof(mode)) {
                len = sizeof(mode) - 1;
            }
            for (size_t i = 0; i < len; ++i) {
                mode[i] = (char)tolower((unsigned char)val[i]);
            }
            mode[len] = '\0';
            if (strcmp(mode, "sta") == 0) {
                next.wifi_boot_mode = WIFI_BOOT_STA;
            } else if (strcmp(mode, "ap") == 0) {
                next.wifi_boot_mode = WIFI_BOOT_AP;
            }
        }
    }

    if (config_save(&next) == ESP_OK) {
        cfg_lock();
        s_cfg = next;
        cfg_unlock();
        update_mdns_name();
    } else {
        ok = false;
        if (err) {
            strncpy(err, "save_failed", err_len);
            err[err_len - 1] = '\0';
        }
    }

    free(tmp);

    if (!ok) {
        return false;
    }

    return ok;
}

static const char k_index_html[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<title>E-WiMill Setup</title>"
    "<style>"
    "body{font-family:Arial,sans-serif;margin:20px;color:#111;}"
    ".tabs{display:flex;gap:8px;margin-bottom:14px;}"
    ".tab{padding:8px 12px;border:1px solid #ccc;background:#f6f6f6;cursor:pointer;}"
    ".tab.active{background:#111;color:#fff;border-color:#111;}"
    ".view{display:none;}"
    ".view.active{display:block;}"
    "pre{background:#f2f2f2;padding:10px;overflow:auto;}"
    ".row{margin-bottom:12px;}#msg{margin:12px 0;color:#0b5;}#err{color:#c00;margin:6px 0;}"
    ".files-header{display:flex;align-items:center;justify-content:space-between;margin:10px 0;"
    "gap:12px;flex-wrap:wrap;}"
    "#fsPath{font-weight:bold;}"
    ".actions button{margin-right:6px;}"
    "#drop{border:2px dashed #999;padding:16px;text-align:center;margin-bottom:10px;}"
    "#drop.hover{border-color:#333;color:#333;}"
    "#fsTable{width:100%;border-collapse:collapse;}"
    "#fsTable th,#fsTable td{border-bottom:1px solid #ddd;padding:6px;text-align:left;}"
    "#fsTable tr.selected{background:#e6f0ff;}"
    ".banner{background:#b00020;color:#fff;padding:10px;margin:10px 0;display:none;}"
    ".banner button{margin-left:10px;}"
    ".crumb{cursor:pointer;color:#1155cc;text-decoration:underline;margin-right:4px;}"
    ".crumb-sep{margin-right:4px;}"
    "</style>"
    "</head><body>"
    "<h1>E-WiMill Setup</h1>"
    "<div class=\"tabs\">"
    "<button id=\"tabSetup\" class=\"tab active\" type=\"button\">Setup</button>"
    "<button id=\"tabFiles\" class=\"tab\" type=\"button\">Files</button>"
    "</div>"
    "<div id=\"setupView\" class=\"view active\">"
    "<h3>Status</h3><pre id=\"status\">loading...</pre>"
    "<div id=\"msg\"></div><div id=\"err\"></div>"
    "<div class=\"row\"><button id=\"openBtn\" type=\"button\">Open</button></div>"
    "<h3>Config</h3>"
    "<form id=\"cfg\">"
    "<label>Device name<br><input id=\"device_name\" name=\"device_name\"/></label><br><br>"
    "<label>STA SSID<br><input id=\"sta_ssid\" name=\"sta_ssid\"/></label><br><br>"
    "<label>STA PSK<br><input id=\"sta_psk\" name=\"sta_psk\" type=\"password\"/>"
    "<button id=\"togglePsk\" type=\"button\">Show</button></label><br><br>"
    "<label>Web port<br><input id=\"web_port\" name=\"web_port\" type=\"number\"/></label><br><br>"
    "<label>Wi-Fi boot<br><select id=\"wifi_boot\" name=\"wifi_boot\">"
    "<option value=\"ap\">AP (setup)</option>"
    "<option value=\"sta\">STA (auto)</option>"
    "</select></label><br><br>"
    "<button type=\"submit\">Apply</button>"
    "</form>"
    "</div>"
    "<div id=\"filesView\" class=\"view\">"
    "<div id=\"fsBusy\" class=\"banner\"><span id=\"fsBusyText\">BUSY: USB Mass Storage attached. "
    "Detach USB to manage files.</span>"
    "<button id=\"btnDetach\" type=\"button\">USB Detach</button>"
    "</div>"
    "<div class=\"files-header\">"
    "<div id=\"fsPath\">/sdcard</div>"
    "<div class=\"actions\">"
    "<button id=\"btnUpload\" type=\"button\">Upload</button>"
    "<button id=\"btnMkdir\" type=\"button\">New Folder</button>"
    "<button id=\"btnRename\" type=\"button\">Rename</button>"
    "<button id=\"btnDelete\" type=\"button\">Delete</button>"
    "<button id=\"btnDownload\" type=\"button\">Download</button>"
    "<input id=\"fileInput\" type=\"file\" style=\"display:none\"/>"
    "</div>"
    "</div>"
    "<div id=\"drop\">Drop file here or click Upload</div>"
    "<div id=\"fsMsg\"></div>"
    "<table id=\"fsTable\"><thead><tr>"
    "<th>Type</th><th>Name</th><th>Size</th>"
    "</tr></thead><tbody></tbody></table>"
    "</div>"
    "<script>"
    "const statusEl=document.getElementById('status');"
    "const msgEl=document.getElementById('msg');"
    "const errEl=document.getElementById('err');"
    "const openBtn=document.getElementById('openBtn');"
    "const form=document.getElementById('cfg');"
    "const togglePsk=document.getElementById('togglePsk');"
    "const tabSetup=document.getElementById('tabSetup');"
    "const tabFiles=document.getElementById('tabFiles');"
    "const setupView=document.getElementById('setupView');"
    "const filesView=document.getElementById('filesView');"
    "const fsBusy=document.getElementById('fsBusy');"
    "const fsBusyText=document.getElementById('fsBusyText');"
    "const btnDetach=document.getElementById('btnDetach');"
    "const fsPath=document.getElementById('fsPath');"
    "const fsMsg=document.getElementById('fsMsg');"
    "const fsTable=document.getElementById('fsTable');"
    "const fsBody=fsTable.querySelector('tbody');"
    "const btnUpload=document.getElementById('btnUpload');"
    "const btnMkdir=document.getElementById('btnMkdir');"
    "const btnRename=document.getElementById('btnRename');"
    "const btnDelete=document.getElementById('btnDelete');"
    "const btnDownload=document.getElementById('btnDownload');"
    "const fileInput=document.getElementById('fileInput');"
    "const drop=document.getElementById('drop');"
    "let filled=false;let redirected=(localStorage.getItem('mdns_redirected')==='1');"
    "let activeTab='setup';"
    "let currentPath='/';"
    "let selected=null;"
    "let uploading=false;"
    "let fsBlockedReason='';"
    "openBtn.onclick=()=>{if(openBtn.dataset.url){window.location=openBtn.dataset.url;}};"
    "togglePsk.onclick=()=>{const p=document.getElementById('sta_psk');"
    "if(p.type==='password'){p.type='text';togglePsk.textContent='Hide';}"
    "else{p.type='password';togglePsk.textContent='Show';}};"
    "tabSetup.onclick=()=>setTab('setup');"
    "tabFiles.onclick=()=>setTab('files');"
    "function setTab(name){"
    "activeTab=name;"
    "if(name==='setup'){tabSetup.classList.add('active');tabFiles.classList.remove('active');"
    "setupView.classList.add('active');filesView.classList.remove('active');}"
    "else{tabFiles.classList.add('active');tabSetup.classList.remove('active');"
    "filesView.classList.add('active');setupView.classList.remove('active');"
    "refreshFiles();}}"
    "function setFsMsg(msg,isErr){fsMsg.textContent=msg||'';fsMsg.style.color=isErr?'#c00':'#0b5';}"
    "function setButtonsEnabled(enabled){"
    "btnUpload.disabled=!enabled||uploading;"
    "btnMkdir.disabled=!enabled||uploading;"
    "btnRename.disabled=!enabled||uploading||!selected;"
    "btnDelete.disabled=!enabled||uploading||!selected;"
    "btnDownload.disabled=!enabled||uploading||!selected||selected.type!=='file';"
    "}"
    "function updateFsGate(status){"
    "let reason='';"
    "if(status.usb_mode==='ATTACHED'){reason='BUSY: USB Mass Storage attached. Detach USB to manage files.';}"
    "else if(!status.sd_mounted){reason='SD not mounted. Run usb detach first.';}"
    "fsBlockedReason=reason;"
    "if(reason){fsBusy.style.display='block';fsBusyText.textContent=reason;"
    "btnDetach.style.display=status.usb_mode==='ATTACHED'?'inline-block':'none';"
    "setButtonsEnabled(false);}"
    "else{fsBusy.style.display='none';setButtonsEnabled(true);}"
    "}"
    "function renderBreadcrumb(path){"
    "fsPath.innerHTML='';"
    "const root=document.createElement('span');root.textContent='/sdcard';root.className='crumb';"
    "root.onclick=()=>{currentPath='/';refreshFiles();};fsPath.appendChild(root);"
    "const parts=path.split('/').filter(p=>p);"
    "let acc='';"
    "parts.forEach(p=>{acc+='/'+p;const sep=document.createElement('span');"
    "sep.textContent=' / ';sep.className='crumb-sep';fsPath.appendChild(sep);"
    "const c=document.createElement('span');c.textContent=p;c.className='crumb';"
    "c.onclick=()=>{currentPath=acc;refreshFiles();};fsPath.appendChild(c);});"
    "}"
    "function renderList(items){"
    "fsBody.innerHTML='';selected=null;setButtonsEnabled(!fsBlockedReason);"
    "items.forEach(item=>{const tr=document.createElement('tr');"
    "tr.dataset.name=item.name;tr.dataset.type=item.type;"
    "tr.innerHTML='<td>'+(item.type==='dir'?'DIR':'FILE')+'</td><td>'+item.name+'</td><td>'+(item.size||'')+'</td>';"
    "tr.onclick=()=>{selectItem(item,tr);};"
    "tr.ondblclick=()=>{if(item.type==='dir'){currentPath=joinPath(currentPath,item.name);refreshFiles();}};"
    "fsBody.appendChild(tr);});"
    "}"
    "function selectItem(item,row){"
    "Array.from(fsBody.children).forEach(r=>r.classList.remove('selected'));"
    "row.classList.add('selected');"
    "selected=item;setButtonsEnabled(!fsBlockedReason);"
    "}"
    "function joinPath(base,name){return base==='/'?'/'+name:base+'/'+name;}"
    "async function refreshFiles(){"
    "if(fsBlockedReason){setFsMsg(fsBlockedReason,true);return;}"
    "setFsMsg('',false);"
    "const res=await fetch('/api/fs/list?path='+encodeURIComponent(currentPath));"
    "if(!res.ok){let err='ERR';try{const j=await res.json();err=j.error||err;}catch(e){}"
    "setFsMsg(err,true);return;}"
    "const j=await res.json();currentPath=j.path||'/';renderBreadcrumb(currentPath);renderList(j.items||[]);"
    "}"
    "async function uploadFile(file){"
    "if(fsBlockedReason||uploading){return;}"
    "uploading=true;setButtonsEnabled(false);setFsMsg('Uploading...',false);"
    "const fd=new FormData();fd.append('file',file,file.name);"
    "const res=await fetch('/api/fs/upload?path='+encodeURIComponent(currentPath),{method:'POST',body:fd});"
    "uploading=false;"
    "if(!res.ok){let err='UPLOAD_FAIL';try{const j=await res.json();err=j.error||err;}catch(e){}"
    "setFsMsg(err,true);setButtonsEnabled(!fsBlockedReason);return;}"
    "setFsMsg('Done',false);refreshFiles();"
    "}"
    "btnUpload.onclick=()=>{fileInput.value='';fileInput.click();};"
    "fileInput.onchange=()=>{const f=fileInput.files[0];if(f){uploadFile(f);}};"
    "drop.addEventListener('dragover',e=>{e.preventDefault();drop.classList.add('hover');});"
    "drop.addEventListener('dragleave',()=>{drop.classList.remove('hover');});"
    "drop.addEventListener('drop',e=>{e.preventDefault();drop.classList.remove('hover');"
    "const f=e.dataTransfer.files[0];if(f){uploadFile(f);}});"
    "btnMkdir.onclick=async()=>{if(fsBlockedReason){return;}const name=prompt('Folder name');"
    "if(!name){return;}const body=JSON.stringify({path:currentPath,name:name});"
    "const res=await fetch('/api/fs/mkdir',{method:'POST',headers:{'Content-Type':'application/json'},body:body});"
    "if(!res.ok){let err='MKDIR_FAIL';try{const j=await res.json();err=j.error||err;}catch(e){}"
    "setFsMsg(err,true);}else{setFsMsg('Done',false);refreshFiles();}};"
    "btnRename.onclick=async()=>{if(!selected||fsBlockedReason){return;}"
    "const name=prompt('New name',selected.name);if(!name){return;}"
    "const body=JSON.stringify({path:joinPath(currentPath,selected.name),new_name:name});"
    "const res=await fetch('/api/fs/rename',{method:'POST',headers:{'Content-Type':'application/json'},body:body});"
    "if(!res.ok){let err='RENAME_FAIL';try{const j=await res.json();err=j.error||err;}catch(e){}"
    "setFsMsg(err,true);}else{setFsMsg('Done',false);refreshFiles();}};"
    "btnDelete.onclick=async()=>{if(!selected||fsBlockedReason){return;}"
    "if(!confirm('Delete '+selected.name+'?')){return;}"
    "const body=JSON.stringify({path:joinPath(currentPath,selected.name)});"
    "const res=await fetch('/api/fs/delete',{method:'POST',headers:{'Content-Type':'application/json'},body:body});"
    "if(!res.ok){let err='DELETE_FAIL';try{const j=await res.json();err=j.error||err;}catch(e){}"
    "setFsMsg(err,true);}else{setFsMsg('Done',false);refreshFiles();}};"
    "btnDownload.onclick=()=>{if(!selected||selected.type!=='file'){return;}"
    "window.location='/api/fs/download?path='+encodeURIComponent(joinPath(currentPath,selected.name));};"
    "btnDetach.onclick=async()=>{const res=await fetch('/api/usb/detach',{method:'POST'});"
    "if(res.ok){setFsMsg('Detached',false);setTimeout(refreshFiles,500);}else{setFsMsg('DETACH_FAIL',true);}};"
    "async function refreshStatus(){"
    "const r=await fetch('/api/status');if(!r.ok){return;}const j=await r.json();"
    "statusEl.textContent=JSON.stringify(j,null,2);"
    "if(!filled){document.getElementById('device_name').value=j.dev_name||'';"
    "document.getElementById('sta_ssid').value=j.ssid||'';"
    "document.getElementById('web_port').value=j.web_port||'';"
    "document.getElementById('sta_psk').value=j.sta_psk||'';"
    "document.getElementById('wifi_boot').value=(j.wifi_boot||'AP').toLowerCase();"
    "filled=true;}"
    "const port=j.web_port&&j.web_port!=80?(':'+j.web_port):'';"
    "const mdns=j.mdns_name?('http://'+j.mdns_name+'.local'+port):'';"
    "if(mdns){openBtn.textContent='Open '+mdns;openBtn.dataset.url=mdns;}"
    "if(j.sta_connecting){msgEl.textContent='Connecting to '+(j.ssid||'')+'...';errEl.textContent='';}"
    "else if(j.sta_connected){msgEl.textContent='Connected: '+j.sta_ip+' (RSSI '+j.rssi+' dBm)';errEl.textContent='';"
    "if(!redirected&&mdns){redirected=true;localStorage.setItem('mdns_redirected','1');"
    "setTimeout(()=>{window.location=mdns;},2500);}}"
    "else if(j.sta_error){errEl.textContent=j.sta_error;}"
    "updateFsGate(j);"
    "}"
    "form.addEventListener('submit',async(e)=>{"
    "e.preventDefault();msgEl.textContent='Saving...';errEl.textContent='';"
    "localStorage.removeItem('mdns_redirected');redirected=false;"
    "const fd=new FormData(form);"
    "const res=await fetch('/api/config',{method:'POST',body:new URLSearchParams(fd)});"
    "const j=await res.json();"
    "if(j.ok){msgEl.textContent='Saved, connecting...';}"
    "else{errEl.textContent=j.error||'failed';msgEl.textContent='';}"
    "});"
    "setInterval(refreshStatus,2000);refreshStatus();"
    "</script>"
    "</body></html>";

static esp_err_t http_root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, k_index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_status_get(httpd_req_t *req)
{
    char resp[768];
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    bool mounted = sdcard_is_mounted();

    char last_ip[CONFIG_LAST_IP_LEN];
    char ssid[CONFIG_STA_SSID_LEN];
    char psk[CONFIG_STA_PSK_LEN];
    char dev_name[CONFIG_DEV_NAME_LEN];
    uint16_t web_port = 8080;
    uint8_t boot_mode = WIFI_BOOT_AP;
    cfg_lock();
    json_sanitize(last_ip, sizeof(last_ip), s_cfg.last_sta_ip);
    json_sanitize(ssid, sizeof(ssid), s_cfg.sta_ssid);
    json_sanitize(psk, sizeof(psk), s_cfg.sta_psk);
    json_sanitize(dev_name, sizeof(dev_name), s_cfg.dev_name);
    web_port = s_cfg.web_port ? s_cfg.web_port : 8080;
    boot_mode = s_cfg.wifi_boot_mode;
    cfg_unlock();

    char sta_ip[16];
    char sta_error[32];
    char mdns_name[32];
    bool sta_conn = false;
    bool sta_connecting = false;
    int rssi = 0;
    state_lock();
    strncpy(sta_ip, s_sta_ip, sizeof(sta_ip));
    sta_ip[sizeof(sta_ip) - 1] = '\0';
    strncpy(sta_error, s_sta_error, sizeof(sta_error));
    sta_error[sizeof(sta_error) - 1] = '\0';
    strncpy(mdns_name, s_mdns_name, sizeof(mdns_name));
    mdns_name[sizeof(mdns_name) - 1] = '\0';
    sta_conn = s_sta_connected;
    sta_connecting = s_sta_connecting;
    rssi = s_sta_rssi;
    state_unlock();

    if (sta_conn) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            rssi = ap_info.rssi;
        }
    }

    const char *mode = s_active ? "SETUP" : "NORMAL";
    const char *boot_str = (boot_mode == WIFI_BOOT_STA) ? "STA" : "AP";
    const char *usb_host = msc_is_host_connected() ? "connected" : "disconnected";
    snprintf(resp, sizeof(resp),
             "{\"mode\":\"%s\",\"ap_ssid\":\"%s\",\"ap_ip\":\"%s\","
             "\"uptime_s\":%u,\"last_sta_ip\":\"%s\",\"usb_mode\":\"%s\",\"usb_host\":\"%s\","
             "\"sd_mounted\":%s,\"sta_connected\":%s,\"sta_connecting\":%s,"
             "\"sta_ip\":\"%s\",\"sta_error\":\"%s\",\"ssid\":\"%s\",\"sta_psk\":\"%s\","
             "\"rssi\":%d,\"dev_name\":\"%s\",\"mdns_name\":\"%s\",\"web_port\":%u,"
             "\"wifi_boot\":\"%s\"}",
             mode,
             s_active ? s_ap_ssid : "",
             s_active ? s_ap_ip : "",
             (unsigned)uptime_s,
             last_ip[0] ? last_ip : "0.0.0.0",
             usb_mode_str(),
             usb_host,
             mounted ? "true" : "false",
             sta_conn ? "true" : "false",
             sta_connecting ? "true" : "false",
             sta_ip,
             sta_error,
             ssid,
             psk,
             rssi,
             dev_name,
             mdns_name,
             (unsigned)web_port,
             boot_str);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_config_post(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0 || total > 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid size");
        return ESP_FAIL;
    }

    char body[1024];
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
            return ESP_FAIL;
        }
        received += r;
    }
    body[received] = '\0';

    char err[32] = {0};
    bool ok = apply_config_form(body, err, sizeof(err));
    bool has_ssid = false;
    cfg_lock();
    has_ssid = (s_cfg.sta_ssid[0] != '\0');
    cfg_unlock();
    httpd_resp_set_type(req, "application/json");
    if (ok) {
        httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    } else {
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", err[0] ? err : "failed");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
    if (ok && has_ssid) {
        schedule_sta_connect();
    }
    return ESP_OK;
}

static esp_err_t setup_http_start(void)
{
    if (s_http) {
        return ESP_OK;
    }

    cfg_lock();
    uint16_t port = s_cfg.web_port ? s_cfg.web_port : 8080;
    cfg_unlock();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = port;
    cfg.max_uri_handlers = 16;

    esp_err_t err = httpd_start(&s_http, &cfg);
    if (err != ESP_OK) {
        return err;
    }
    s_http_port = port;

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = http_root_get,
        .user_ctx = NULL,
    };
    httpd_uri_t status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = http_status_get,
        .user_ctx = NULL,
    };
    httpd_uri_t config = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = http_config_post,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(s_http, &root);
    httpd_register_uri_handler(s_http, &status);
    httpd_register_uri_handler(s_http, &config);
    web_fs_register_handlers(s_http);

    return ESP_OK;
}

esp_err_t setup_mode_init(void)
{
    if (!s_cfg_mutex) {
        s_cfg_mutex = xSemaphoreCreateMutex();
        if (!s_cfg_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (!s_state_mutex) {
        s_state_mutex = xSemaphoreCreateMutex();
        if (!s_state_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }

    wimill_config_t cfg;
    esp_err_t err = config_load(&cfg);
    if (err != ESP_OK) {
        config_load_defaults(&cfg);
    }
    cfg_lock();
    s_cfg = cfg;
    cfg_unlock();

    update_mdns_name();
    return ESP_OK;
}

esp_err_t setup_mode_start(void)
{
    if (s_active) {
        return ESP_OK;
    }

    s_sta_only_mode = false;
    esp_err_t err = setup_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi AP start failed: %s", esp_err_to_name(err));
        return err;
    }
    err = setup_http_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP start failed: %s", esp_err_to_name(err));
        return err;
    }

    led_status_set_setup(true);
    s_active = true;
    ESP_LOGI(TAG, "SETUP_MODE active");
    return ESP_OK;
}

esp_err_t setup_mode_autostart(void)
{
    if (s_active) {
        return ESP_OK;
    }

    uint8_t boot_mode = WIFI_BOOT_AP;
    bool has_ssid = false;
    cfg_lock();
    boot_mode = s_cfg.wifi_boot_mode;
    has_ssid = (s_cfg.sta_ssid[0] != '\0');
    cfg_unlock();

    if (boot_mode == WIFI_BOOT_AP) {
        return setup_mode_start();
    }

    if (boot_mode == WIFI_BOOT_STA) {
        if (!has_ssid) {
            led_status_set_wifi(false);
            return ESP_OK;
        }
        return setup_sta_only_start();
    }

    return ESP_OK;
}

bool setup_mode_is_active(void)
{
    return s_active;
}
