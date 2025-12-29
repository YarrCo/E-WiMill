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
static bool s_mdns_service_added = false;
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

// --- HELPER FUNCTIONS ---

static void cfg_lock(void)
{
    if (s_cfg_mutex)
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
}

static void cfg_unlock(void)
{
    if (s_cfg_mutex)
        xSemaphoreGive(s_cfg_mutex);
}

static void state_lock(void)
{
    if (s_state_mutex)
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
}

static void state_unlock(void)
{
    if (s_state_mutex)
        xSemaphoreGive(s_state_mutex);
}

static void json_sanitize(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0;
    if (!dst || dst_len == 0)
        return;
    if (!src)
    {
        dst[0] = '\0';
        return;
    }
    for (size_t si = 0; src[si] != '\0' && di + 1 < dst_len; ++si)
    {
        unsigned char ch = (unsigned char)src[si];
        if (ch < 32)
            continue;
        if (ch == '"' || ch == '\\')
        {
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
    if (msg)
    {
        strncpy(s_sta_error, msg, sizeof(s_sta_error));
        s_sta_error[sizeof(s_sta_error) - 1] = '\0';
    }
    else
    {
        s_sta_error[0] = '\0';
    }
    state_unlock();
}

static const char *usb_mode_str(void)
{
    msc_state_t st = msc_get_state();
    switch (st)
    {
    case MSC_STATE_USB_ATTACHED:
        return "ATTACHED";
    case MSC_STATE_USB_DETACHED:
        return "DETACHED";
    default:
        return "ERROR";
    }
}

static void sanitize_dev_name(const char *src, char *out, size_t out_len, const uint8_t mac[6])
{
    size_t di = 0;
    bool last_dash = false;
    if (!out || out_len == 0)
        return;
    for (size_t si = 0; src && src[si] != '\0' && di + 1 < out_len && di < MDNS_NAME_LIMIT; ++si)
    {
        char ch = (char)tolower((unsigned char)src[si]);
        if (ch == ' ' || ch == '_')
            ch = '-';
        if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '-'))
            continue;
        if (ch == '-')
        {
            if (di == 0 || last_dash)
                continue;
            last_dash = true;
        }
        else
        {
            last_dash = false;
        }
        out[di++] = ch;
    }
    if (di > 0 && out[di - 1] == '-')
        di--;
    out[di] = '\0';
    if (di == 0)
        snprintf(out, out_len, "ewimill-%02x%02x", mac[4], mac[5]);
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
    if (!s_mdns_inited)
    {
        err = mdns_init();
        if (err != ESP_OK)
            return err;
        s_mdns_inited = true;
    }
    state_lock();
    char host[32];
    strncpy(host, s_mdns_name, sizeof(host));
    host[sizeof(host) - 1] = '\0';
    state_unlock();
    err = mdns_hostname_set(host);
    if (err != ESP_OK)
        return err;
    mdns_instance_name_set("E-WiMill");
    if (s_mdns_service_added)
        mdns_service_remove("_http", "_tcp");
    err = mdns_service_add(NULL, "_http", "_tcp", port, NULL, 0);
    if (err == ESP_OK)
        s_mdns_service_added = true;
    return err;
}

// --- WIFI LOGIC ---

static void sta_connect_timeout_cb(void *arg)
{
    (void)arg;
    state_lock();
    bool connected = s_sta_connected;
    state_unlock();
    if (connected)
        return;
    set_sta_error("timeout");
    state_lock();
    s_sta_connecting = false;
    state_unlock();
    esp_wifi_disconnect();
    if (s_sta_only_mode)
    {
        led_status_set_wifi(false);
        return;
    }
    if (s_ap_cfg_valid)
    {
        esp_wifi_set_mode(WIFI_MODE_AP);
        esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
    }
    led_status_set_setup(true);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        char err[32];
        snprintf(err, sizeof(err), "reason:%d", (int)disc->reason);
        set_sta_error(err);
        state_lock();
        s_sta_connected = false;
        s_sta_connecting = false;
        strncpy(s_sta_ip, "0.0.0.0", sizeof(s_sta_ip));
        state_unlock();
        if (s_sta_timer)
            esp_timer_stop(s_sta_timer);
        if (s_sta_only_mode)
        {
            led_status_set_wifi(false);
            return;
        }
        if (s_ap_cfg_valid)
        {
            esp_wifi_set_mode(WIFI_MODE_AP);
            esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
        }
        led_status_set_setup(true);
        return;
    }
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
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
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
        {
            state_lock();
            s_sta_rssi = ap_info.rssi;
            state_unlock();
        }
        if (s_sta_timer)
            esp_timer_stop(s_sta_timer);
        cfg_lock();
        strncpy(s_cfg.last_sta_ip, ip_str, sizeof(s_cfg.last_sta_ip));
        s_cfg.last_sta_ip[sizeof(s_cfg.last_sta_ip) - 1] = '\0';
        config_save(&s_cfg);
        uint16_t port = s_cfg.web_port ? s_cfg.web_port : 8080;
        bool boot_sta = (s_cfg.wifi_boot_mode == WIFI_BOOT_STA);
        cfg_unlock();
        update_mdns_name();
        setup_mdns_start(port);
        if (s_http && port != s_http_port)
        {
            httpd_stop(s_http);
            s_http = NULL;
            s_http_port = 0;
        }
        setup_http_start();
        if (boot_sta)
        {
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
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return err;
    if (!s_wifi_inited)
    {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
        if (err != ESP_OK)
            return err;
        s_wifi_inited = true;
    }
    if (!s_handlers_registered)
    {
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
        s_handlers_registered = true;
    }
    return ESP_OK;
}

static esp_err_t setup_wifi_start(void)
{
    esp_err_t err = setup_wifi_init_base();
    if (err != ESP_OK)
        return err;
    if (!s_ap_netif)
    {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif)
            return ESP_FAIL;
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
    if (strlen(AP_PASS) == 0)
        s_ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    s_ap_cfg_valid = true;
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK)
        return err;
    err = esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
    if (err != ESP_OK)
        return err;
    err = esp_wifi_start();
    if (err != ESP_OK)
        return err;
    esp_netif_ip_info_t ip;
    if (s_ap_netif && esp_netif_get_ip_info(s_ap_netif, &ip) == ESP_OK)
    {
        esp_ip4addr_ntoa(&ip.ip, s_ap_ip, sizeof(s_ap_ip));
    }
    else
    {
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
    if (err != ESP_OK)
        return err;
    if (!s_sta_netif)
    {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (!s_sta_netif)
            return ESP_FAIL;
    }
    s_sta_only_mode = true;
    led_status_set_wifi(false);
    if (!start_sta_connect_async())
        return ESP_ERR_INVALID_STATE;
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
    if (ssid[0] == '\0')
    {
        set_sta_error("ssid_empty");
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_sta_netif)
    {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (!s_sta_netif)
            return ESP_FAIL;
    }
    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
    strncpy((char *)sta_cfg.sta.password, psk, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.scan_method = WIFI_FAST_SCAN;
    sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    esp_err_t err = esp_wifi_set_mode(s_sta_only_mode ? WIFI_MODE_STA : WIFI_MODE_APSTA);
    if (err != ESP_OK)
        return err;
    if (!s_sta_only_mode && s_ap_cfg_valid)
        esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
    err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    if (err != ESP_OK)
        return err;
    if (s_sta_only_mode)
    {
        esp_err_t start_err = esp_wifi_start();
        if (start_err != ESP_OK && start_err != ESP_ERR_WIFI_CONN && start_err != ESP_ERR_INVALID_STATE)
            return start_err;
    }
    esp_wifi_disconnect();
    err = esp_wifi_connect();
    if (err != ESP_OK)
        return err;
    state_lock();
    s_sta_connecting = true;
    s_sta_connected = false;
    strncpy(s_sta_ip, "0.0.0.0", sizeof(s_sta_ip));
    s_sta_error[0] = '\0';
    s_sta_rssi = 0;
    state_unlock();
    if (!s_sta_timer)
    {
        const esp_timer_create_args_t args = {.callback = sta_connect_timeout_cb, .name = "sta_timeout", .skip_unhandled_events = true};
        esp_timer_create(&args, &s_sta_timer);
    }
    if (s_sta_timer)
    {
        esp_timer_stop(s_sta_timer);
        esp_timer_start_once(s_sta_timer, (uint64_t)STA_CONNECT_TIMEOUT_MS * 1000ULL);
    }
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
    if (s_sta_task_running || s_sta_connecting)
    {
        state_unlock();
        return false;
    }
    s_sta_task_running = true;
    state_unlock();
    if (xTaskCreate(sta_connect_task, "sta_connect", 4096, NULL, 6, NULL) != pdPASS)
    {
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
    if (!s_apply_timer)
    {
        const esp_timer_create_args_t args = {.callback = apply_timer_cb, .name = "sta_apply", .skip_unhandled_events = true};
        esp_timer_create(&args, &s_apply_timer);
    }
    esp_timer_stop(s_apply_timer);
    esp_timer_start_once(s_apply_timer, 200 * 1000ULL);
}

static void url_decode(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0, si = 0;
    while (src[si] && di + 1 < dst_len)
    {
        char ch = src[si];
        if (ch == '%' && isxdigit((unsigned char)src[si + 1]) && isxdigit((unsigned char)src[si + 2]))
        {
            char hex[3] = {src[si + 1], src[si + 2], '\0'};
            dst[di++] = (char)strtol(hex, NULL, 16);
            si += 3;
        }
        else if (ch == '+')
        {
            dst[di++] = ' ';
            si++;
        }
        else
        {
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
    if (!tmp)
        return false;
    wimill_config_t next;
    cfg_lock();
    next = s_cfg;
    cfg_unlock();
    char *saveptr = NULL;
    for (char *pair = strtok_r(tmp, "&", &saveptr); pair; pair = strtok_r(NULL, "&", &saveptr))
    {
        char *eq = strchr(pair, '=');
        if (!eq)
            continue;
        *eq = '\0';
        char key[32] = {0}, val[96] = {0};
        url_decode(key, sizeof(key), pair);
        url_decode(val, sizeof(val), eq + 1);
        if (strcmp(key, "device_name") == 0)
            strncpy(next.dev_name, val, sizeof(next.dev_name));
        else if (strcmp(key, "sta_ssid") == 0)
            strncpy(next.sta_ssid, val, sizeof(next.sta_ssid));
        else if (strcmp(key, "sta_psk") == 0)
            strncpy(next.sta_psk, val, sizeof(next.sta_psk));
        else if (strcmp(key, "web_port") == 0)
        {
            int port = atoi(val);
            if (port > 0 && port < 65536)
                next.web_port = (uint16_t)port;
        }
        else if (strcmp(key, "wifi_boot") == 0)
        {
            if (strcmp(val, "sta") == 0)
                next.wifi_boot_mode = WIFI_BOOT_STA;
            else
                next.wifi_boot_mode = WIFI_BOOT_AP;
        }
    }
    if (config_save(&next) == ESP_OK)
    {
        cfg_lock();
        s_cfg = next;
        cfg_unlock();
        update_mdns_name();
    }
    else
    {
        ok = false;
        if (err)
            strncpy(err, "save_failed", err_len);
    }
    free(tmp);
    return ok;
}

// --- HTML & RESOURCES (NEON CNC THEME) ---

static const char k_index_html[] =
    "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
    "<title>NEON CNC CONTROL</title><style>"
    ":root{--bg:#050510;--grid:rgba(0,255,255,0.1);--cyan:#00f3ff;--pink:#ff00ff;"
    "--red:#ff3333;--green:#33ff33;--glass:rgba(0,20,40,0.7);}"
    "body{margin:0;padding:20px;background-color:var(--bg);background-image:"
    "linear-gradient(var(--grid) 1px,transparent 1px),linear-gradient(90deg,var(--grid) 1px,transparent 1px);"
    "background-size:30px 30px;color:var(--cyan);font-family:'Courier New',monospace;font-weight:bold;"
    "min-height:100vh;box-sizing:border-box;overflow-x:hidden;}"
    "body::after{content:\"\";position:fixed;top:0;left:0;width:100vw;height:100vh;background:"
    "repeating-linear-gradient(0deg,rgba(0,0,0,0.15),rgba(0,0,0,0.15) 1px,transparent 1px,transparent 2px);"
    "pointer-events:none;z-index:999;}"
    ".container{max-width:800px;margin:0 auto;border:2px solid var(--cyan);box-shadow:0 0 15px var(--cyan),"
    "inset 0 0 20px rgba(0,243,255,0.2);background:var(--glass);backdrop-filter:blur(5px);padding:20px;"
    "border-radius:4px;position:relative;}"
    "header{display:flex;justify-content:space-between;align-items:center;border-bottom:2px solid var(--cyan);"
    "padding-bottom:15px;margin-bottom:20px;}h1{margin:0;text-transform:uppercase;letter-spacing:4px;"
    "text-shadow:2px 2px 0px var(--pink);font-size:1.5rem;}"
    ".sys-status{font-size:0.9rem;text-align:right;}.status-badge{display:inline-block;padding:2px 8px;"
    "background:#000;border:1px solid currentColor;}"
    ".status-ok{color:var(--green);box-shadow:0 0 5px var(--green);}"
    ".status-warn{color:var(--red);box-shadow:0 0 5px var(--red);animation:blink 1s infinite;}"
    ".tabs{display:flex;gap:10px;margin-bottom:20px;}.tab-btn{flex:1;background:transparent;border:1px solid var(--cyan);"
    "color:var(--cyan);padding:10px;cursor:pointer;text-transform:uppercase;font-family:inherit;font-weight:bold;"
    "transition:0.2s;box-shadow:0 0 5px var(--cyan);}.tab-btn:hover{background:rgba(0,243,255,0.1);"
    "transform:translateY(-2px);}.tab-btn.active{background:var(--cyan);color:#000;box-shadow:0 0 15px var(--cyan);}"
    ".view{display:none;}.view.active{display:block;}"
    ".diag-table{width:100%;border-collapse:collapse;margin-top:10px;}"
    ".diag-table td,.diag-table th{border:1px solid var(--cyan);padding:8px;text-align:left;}"
    ".diag-table th{background:rgba(0,243,255,0.2);text-transform:uppercase;}.val-ok{color:var(--green);}"
    ".val-num{color:var(--pink);}.val-err{color:var(--red);}"
    ".cfg-box{padding:15px;border:1px solid var(--grid);margin-top:15px;}"
    ".cfg-input{background:black;border:1px solid var(--cyan);color:var(--cyan);font-family:inherit;"
    "padding:5px;width:100%;box-sizing:border-box;margin-bottom:10px;}"
    ".toolbar{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:15px;padding:10px;border:1px dashed var(--cyan);}"
    ".toolbar.disabled{opacity:0.3;pointer-events:none;}"
    ".btn{background:#000;border:1px solid var(--pink);color:var(--pink);padding:8px 16px;cursor:pointer;"
    "text-transform:uppercase;font-family:inherit;font-size:0.8rem;transition:0.2s;}"
    ".btn:hover{background:var(--pink);color:#000;box-shadow:0 0 10px var(--pink);}"
    ".btn-green{border-color:var(--green);color:var(--green);}.btn-green:hover{background:var(--green);"
    "color:#000;box-shadow:0 0 10px var(--green);}"
    "#dropZone{border:2px dashed var(--cyan);padding:30px;text-align:center;margin-bottom:20px;"
    "color:rgba(0,243,255,0.5);transition:0.3s;cursor:pointer;}"
    "#dropZone.hover{background:rgba(0,243,255,0.1);color:var(--cyan);border-style:solid;box-shadow:inset 0 0 20px var(--cyan);}"
    ".progress-container{border:1px solid var(--cyan);height:20px;margin-bottom:20px;background:#000;"
    "position:relative;display:none;}"
    ".progress-bar{height:100%;width:0%;background:repeating-linear-gradient(45deg,var(--pink),var(--pink) 10px,"
    "#d600d6 10px,#d600d6 20px);box-shadow:0 0 10px var(--pink);transition:width 0.2s linear;}"
    ".progress-text{position:absolute;width:100%;text-align:center;top:0;line-height:20px;color:#fff;"
    "text-shadow:1px 1px 0 #000;font-size:0.8rem;}"
    ".file-list{width:100%;border-collapse:collapse;}.file-list th{text-align:left;border-bottom:2px solid var(--cyan);"
    "padding:5px;}.file-list td{padding:8px 5px;border-bottom:1px solid rgba(0,243,255,0.3);cursor:pointer;}"
    ".file-list tr:hover{background:var(--cyan);color:#000;}.file-list tr.selected{background:rgba(0,243,255,0.3);}"
    ".banner-warn{display:none;border:2px solid var(--red);padding:15px;background:rgba(50,0,0,0.5);color:var(--red);"
    "text-align:center;box-shadow:0 0 15px var(--red);margin-bottom:15px;animation:blink 1s infinite alternate;}"
    "@keyframes blink{from{opacity:1;}to{opacity:0.7;}}"
    "</style></head><body>"
    "<div class=\"container\">"
    "<header><h1>WiMill <span style=\"color:var(--pink)\">//</span> CNC</h1>"
    "<div class=\"sys-status\">SYSTEM: <span id=\"stSys\" class=\"status-badge\">...</span><br>"
    "USB LINK: <span id=\"stUsb\" class=\"status-badge\">...</span></div></header>"
    "<div class=\"tabs\"><button id=\"tabSetup\" class=\"tab-btn active\" onclick=\"setTab('setup')\">SYSTEM</button>"
    "<button id=\"tabFiles\" class=\"tab-btn\" onclick=\"setTab('files')\">STORAGE</button></div>"
    "<div id=\"setupView\" class=\"view active\">"
    "<h3 style=\"border-bottom:1px solid var(--pink);display:inline-block;\">DIAGNOSTICS</h3>"
    "<table class=\"diag-table\">"
    "<tr><th>Parameter</th><th>Value</th></tr>"
    "<tr><td>Device Name</td><td id=\"valDevName\" class=\"val-num\">-</td></tr>"
    "<tr><td>Wi-Fi SSID</td><td id=\"valSsid\" class=\"val-ok\">-</td></tr>"
    "<tr><td>IP Address</td><td id=\"valIp\" class=\"val-num\">-</td></tr>"
    "<tr><td>Last Known IP</td><td id=\"valLastIp\" class=\"val-num\">-</td></tr>"
    "<tr><td>Signal (RSSI)</td><td id=\"valRssi\" class=\"val-num\">-</td></tr>"
    "<tr><td>Uptime</td><td id=\"valUptime\" class=\"val-num\">-</td></tr>"
    "<tr><td>SD Card</td><td id=\"valSd\" class=\"val-ok\">-</td></tr>"
    "</table><br>"
    "<h3 style=\"border-bottom:1px solid var(--pink);display:inline-block;\">CONFIGURATION</h3>"
    "<form id=\"cfgForm\" class=\"cfg-box\">"
    "<label>DEVICE NAME</label><input id=\"device_name\" name=\"device_name\" class=\"cfg-input\">"
    "<label>STA SSID</label><input id=\"sta_ssid\" name=\"sta_ssid\" class=\"cfg-input\">"
    "<label>STA PASSWORD</label><input id=\"sta_psk\" name=\"sta_psk\" type=\"password\" class=\"cfg-input\">"
    "<label>WEB PORT</label><input id=\"web_port\" name=\"web_port\" type=\"number\" class=\"cfg-input\">"
    "<label>WIFI BOOT MODE</label><select id=\"wifi_boot\" name=\"wifi_boot\" class=\"cfg-input\" style=\"background:black;\">"
    "<option value=\"sta\">STA (AUTO CONNECT)</option><option value=\"ap\">AP (SETUP MODE)</option></select>"
    "<button type=\"submit\" class=\"btn btn-green\">APPLY SETTINGS</button><div id=\"saveMsg\"></div>"
    "</form></div>"
    "<div id=\"filesView\" class=\"view\">"
    "<div id=\"usbWarning\" class=\"banner-warn\">⚠ USB CONTROLLED BY HOST ⚠<br>FILE OPERATIONS LOCKED</div>"
    "<div style=\"display:flex;justify-content:space-between;margin-bottom:15px;background:rgba(255,0,255,0.1);padding:10px;border:1px solid var(--pink);\">"
    "<span>USB INTERFACE CONTROL:</span><div><button id=\"btnAttach\" class=\"btn\" onclick=\"usbAction('attach')\">MOUNT (ATTACH)</button>"
    "<button id=\"btnDetach\" class=\"btn btn-green\" onclick=\"usbAction('detach')\" style=\"display:none;\">EJECT (DETACH)</button></div></div>"
    "<div id=\"progressContainer\" class=\"progress-container\"><div id=\"progressBar\" class=\"progress-bar\"></div>"
    "<div id=\"progressText\" class=\"progress-text\"></div></div>"
    "<div id=\"toolbar\" class=\"toolbar\"><button class=\"btn\" onclick=\"triggerUpload()\">[↑] UPLOAD</button>"
    "<button class=\"btn\" onclick=\"fsMkdir()\">[+] NEW DIR</button><button class=\"btn\" onclick=\"fsRename()\">[R] RENAME</button>"
    "<button class=\"btn\" style=\"border-color:var(--red);color:var(--red);\" onclick=\"fsDelete()\">[x] DELETE</button>"
    "<button class=\"btn\" onclick=\"fsDownload()\">[↓] DOWNLOAD</button>"
    "<input id=\"fileInput\" type=\"file\" style=\"display:none\"></div>"
    "<div style=\"margin-bottom:10px;\">PATH: <span id=\"fsPath\" style=\"color:var(--pink)\">/</span></div>"
    "<div id=\"dropZone\">>> DRAG & DROP G-CODE FILES HERE <<</div>"
    "<table class=\"file-list\"><thead><tr><th>TYPE</th><th>NAME</th><th>SIZE</th></tr></thead><tbody id=\"fsBody\"></tbody></table>"
    "</div></div><script>"
    "let currentPath='/';let selected=null;let uploading=false;let filled=false;let pc=null;let pb=null;let pt=null;"
    "function fmt(b){if(b<1024)return b+' B';if(b<1048576)return(b/1024).toFixed(1)+' KB';return(b/1048576).toFixed(1)+' MB';}"
    "function setTab(t){document.querySelectorAll('.view').forEach(e=>e.classList.remove('active'));"
    "document.querySelectorAll('.tab-btn').forEach(e=>e.classList.remove('active'));"
    "document.getElementById(t+'View').classList.add('active');"
    "document.getElementById('tab'+(t==='setup'?'Setup':'Files')).classList.add('active');"
    "if(t==='files') refreshFiles();}"
    "async function updateStatus(){try{const r=await fetch('/api/status');const j=await r.json();"
    "const sSys=document.getElementById('stSys');const sUsb=document.getElementById('stUsb');"
    "if(j.sta_connected){sSys.textContent='ONLINE ('+j.sta_ip+')';sSys.className='status-badge status-ok';}"
    "else if(j.sta_connecting){sSys.textContent='CONNECTING...';sSys.className='status-badge status-warn';}"
    "else{sSys.textContent='OFFLINE (AP)';sSys.className='status-badge';}"
    "if(j.usb_mode==='ATTACHED'){sUsb.textContent='ATTACHED';sUsb.className='status-badge status-warn';"
    "document.getElementById('usbWarning').style.display='block';document.getElementById('toolbar').classList.add('disabled');"
    "document.getElementById('btnAttach').style.display='none';document.getElementById('btnDetach').style.display='inline-block';}"
    "else{sUsb.textContent='DETACHED';sUsb.className='status-badge status-ok';"
    "document.getElementById('usbWarning').style.display='none';document.getElementById('toolbar').classList.remove('disabled');"
    "document.getElementById('btnAttach').style.display='inline-block';document.getElementById('btnDetach').style.display='none';}"
    "document.getElementById('valDevName').textContent=j.dev_name;"
    "document.getElementById('valSsid').textContent=j.ssid||j.ap_ssid;"
    "document.getElementById('valIp').textContent=j.sta_ip;"
    "document.getElementById('valLastIp').textContent=j.last_sta_ip||'-';"
    "document.getElementById('valRssi').textContent=j.rssi+' dBm';"
    "document.getElementById('valUptime').textContent=Math.floor(j.uptime_s/60)+'m '+j.uptime_s%60+'s';"
    "document.getElementById('valSd').textContent=j.sd_mounted?'MOUNTED':'UNMOUNTED';"
    "if(!filled){document.getElementById('device_name').value=j.dev_name||'';document.getElementById('sta_ssid').value=j.ssid||'';"
    "document.getElementById('sta_psk').value=j.sta_psk||'';document.getElementById('web_port').value=j.web_port||80;"
    "document.getElementById('wifi_boot').value=(j.wifi_boot||'ap').toLowerCase();filled=true;}"
    "}catch(e){console.error(e);}}setInterval(updateStatus,2000);updateStatus();"
    "document.getElementById('cfgForm').onsubmit=async(e)=>{e.preventDefault();const msg=document.getElementById('saveMsg');"
    "msg.textContent='SAVING...';const fd=new FormData(e.target);const r=await fetch('/api/config',{method:'POST',body:new URLSearchParams(fd)});"
    "const j=await r.json();msg.textContent=j.ok?'SAVED. CONNECTING...':'ERROR: '+j.error;};"
    "async function usbAction(act){await fetch('/api/usb/'+act,{method:'POST'});updateStatus();}"
    "async function refreshFiles(){const r=await fetch('/api/fs/list?path='+encodeURIComponent(currentPath));"
    "const j=await r.json();currentPath=j.path||'/';document.getElementById('fsPath').textContent=currentPath;"
    "const tb=document.getElementById('fsBody');tb.innerHTML='';selected=null;"
    "if(currentPath!=='/'){addRow({type:'dir',name:'..'});}"
    "(j.items||[]).forEach(i=>addRow(i));}"
    "function addRow(i){const tr=document.createElement('tr');"
    "tr.innerHTML='<td>'+(i.type==='dir'?'[DIR]':'[FILE]')+'</td><td>'+i.name+'</td><td>'+(i.size||'')+'</td>';"
    "tr.onclick=()=>{Array.from(tr.parentNode.children).forEach(r=>r.classList.remove('selected'));tr.classList.add('selected');selected=i;};"
    "tr.ondblclick=()=>{if(i.type==='dir'){currentPath=i.name==='..'?currentPath.split('/').slice(0,-1).join('/')||'/':(currentPath==='/'?'/':currentPath+'/')+i.name;refreshFiles();}};"
    "document.getElementById('fsBody').appendChild(tr);}"
    "function triggerUpload(){document.getElementById('fileInput').click();}"
    "document.getElementById('fileInput').onchange=(e)=>{if(e.target.files[0]) uploadFile(e.target.files[0]);};"
    "const dz=document.getElementById('dropZone');"
    "dz.ondragover=e=>{e.preventDefault();dz.classList.add('hover');};dz.ondragleave=()=>{dz.classList.remove('hover');};"
    "dz.ondrop=e=>{e.preventDefault();dz.classList.remove('hover');if(e.dataTransfer.files[0]) uploadFile(e.dataTransfer.files[0]);};"
    "function uploadFile(file){if(uploading)return;uploading=true;"
    "pc=document.getElementById('progressContainer');pb=document.getElementById('progressBar');pt=document.getElementById('progressText');"
    "pc.style.display='block';uploadRaw(file,true);}"
    "function progressUpdate(e,start){const p=(e.loaded/e.total)*100;const t=(performance.now()-start)/1000;const s=t>0?e.loaded/t:0;"
    "pb.style.width=p+'%';pt.textContent='UPLOADING: '+p.toFixed(0)+'% @ '+fmt(s)+'/s';}"
    "function uploadRaw(file,allowFallback){const xhr=new XMLHttpRequest();const start=performance.now();"
    "xhr.upload.onprogress=e=>{progressUpdate(e,start);};"
    "xhr.onload=()=>{if(xhr.status===200){uploading=false;pc.style.display='none';refreshFiles();}"
    "else if(allowFallback){uploadMultipart(file);}else{uploading=false;alert('Upload failed');pc.style.display='none';}};"
    "xhr.onerror=()=>{if(allowFallback){uploadMultipart(file);}else{uploading=false;alert('Upload failed');pc.style.display='none';}};"
    "const url='/api/fs/upload_raw?path='+encodeURIComponent(currentPath)+'&name='+encodeURIComponent(file.name)+'&overwrite=1';"
    "xhr.open('POST',url);xhr.setRequestHeader('Content-Type','application/octet-stream');xhr.send(file);}"
    "function uploadMultipart(file){const fd=new FormData();fd.append('file',file);const xhr=new XMLHttpRequest();"
    "const start=performance.now();xhr.upload.onprogress=e=>{progressUpdate(e,start);};"
    "xhr.onload=()=>{uploading=false;pc.style.display='none';refreshFiles();};"
    "xhr.onerror=()=>{uploading=false;alert('Upload failed');pc.style.display='none';};"
    "xhr.open('POST','/api/fs/upload?path='+encodeURIComponent(currentPath)+'&overwrite=1');xhr.send(fd);}"
    "async function fsMkdir(){const n=prompt('FOLDER NAME:');if(n) await apiCall('/api/fs/mkdir',{path:currentPath,name:n});}"
    "async function fsRename(){if(!selected)return;const n=prompt('NEW NAME:',selected.name);"
    "if(n) await apiCall('/api/fs/rename',{path:(currentPath==='/'?'/':currentPath+'/')+selected.name,new_name:n});}"
    "async function fsDelete(){if(!selected||!confirm('DELETE '+selected.name+'?'))return;"
    "await apiCall('/api/fs/delete',{path:(currentPath==='/'?'/':currentPath+'/')+selected.name});}"
    "function fsDownload(){if(selected&&selected.type==='file') window.location='/api/fs/download?path='+encodeURIComponent((currentPath==='/'?'/':currentPath+'/')+selected.name);}"
    "async function apiCall(u,d){await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});refreshFiles();}"
    "</script></body></html>";

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

    if (sta_conn)
    {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
        {
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
             last_ip[0] ? last_ip : "", // Передаем last_sta_ip
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
    if (total <= 0 || total > 1024)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid size");
        return ESP_FAIL;
    }

    char body[1024];
    int received = 0;
    while (received < total)
    {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0)
        {
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
    if (ok)
    {
        httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", err[0] ? err : "failed");
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
    if (ok && has_ssid)
    {
        schedule_sta_connect();
    }
    return ESP_OK;
}

static esp_err_t setup_http_start(void)
{
    if (s_http)
    {
        return ESP_OK;
    }

    cfg_lock();
    uint16_t port = s_cfg.web_port ? s_cfg.web_port : 8080;
    cfg_unlock();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = port;
    cfg.max_uri_handlers = 16;
    cfg.stack_size = 8192;

    esp_err_t err = httpd_start(&s_http, &cfg);
    if (err != ESP_OK)
    {
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
    if (!s_cfg_mutex)
    {
        s_cfg_mutex = xSemaphoreCreateMutex();
        if (!s_cfg_mutex)
            return ESP_ERR_NO_MEM;
    }
    if (!s_state_mutex)
    {
        s_state_mutex = xSemaphoreCreateMutex();
        if (!s_state_mutex)
            return ESP_ERR_NO_MEM;
    }

    wimill_config_t cfg;
    esp_err_t err = config_load(&cfg);
    if (err != ESP_OK)
    {
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
    if (s_active)
        return ESP_OK;

    s_sta_only_mode = false;
    esp_err_t err = setup_wifi_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Wi-Fi AP start failed: %s", esp_err_to_name(err));
        return err;
    }
    err = setup_http_start();
    if (err != ESP_OK)
    {
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
    if (s_active)
        return ESP_OK;

    uint8_t boot_mode = WIFI_BOOT_AP;
    bool has_ssid = false;
    cfg_lock();
    boot_mode = s_cfg.wifi_boot_mode;
    has_ssid = (s_cfg.sta_ssid[0] != '\0');
    cfg_unlock();

    if (boot_mode == WIFI_BOOT_AP)
    {
        return setup_mode_start();
    }

    if (boot_mode == WIFI_BOOT_STA)
    {
        if (!has_ssid)
        {
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
