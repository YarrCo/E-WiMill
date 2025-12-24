#include "web_fs.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "msc.h"
#include "sdcard.h"

#define TAG "WEBFS"
#define FILE_BUF_SIZE 2048
#define UPLOAD_HEADER_SIZE 8192
#define UPLOAD_TAIL_SIZE 96
#define UPLOAD_WORK_SIZE (FILE_BUF_SIZE + UPLOAD_TAIL_SIZE)
#define MAX_QUERY_LEN 128
#define MAX_PATH_LEN 256
#define MAX_NAME_LEN 96
#define MAX_BODY_LEN 512

static SemaphoreHandle_t s_fileop_mutex = NULL;
static char s_upload_header[UPLOAD_HEADER_SIZE];
static char s_upload_buf[FILE_BUF_SIZE];
static char s_upload_work[UPLOAD_WORK_SIZE];
static char s_upload_tail[UPLOAD_TAIL_SIZE];

static void send_json_error(httpd_req_t *req, const char *status, const char *json)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static char *find_seq(const char *buf, size_t len, const char *seq, size_t seq_len)
{
    if (!buf || !seq || seq_len == 0 || len < seq_len) {
        return NULL;
    }
    for (size_t i = 0; i + seq_len <= len; ++i) {
        if (memcmp(buf + i, seq, seq_len) == 0) {
            return (char *)(buf + i);
        }
    }
    return NULL;
}

static char *find_header_end(const char *buf, size_t len, size_t *mark_len)
{
    char *pos = find_seq(buf, len, "\r\n\r\n", 4);
    if (pos) {
        if (mark_len) {
            *mark_len = 4;
        }
        return pos;
    }
    pos = find_seq(buf, len, "\n\n", 2);
    if (pos && mark_len) {
        *mark_len = 2;
    }
    return pos;
}

static bool fs_gate(httpd_req_t *req)
{
    if (msc_get_state() == MSC_STATE_USB_ATTACHED) {
        send_json_error(req, "423 Locked", "{\"error\":\"BUSY\",\"mode\":\"ATTACHED\"}");
        return false;
    }
    if (!sdcard_is_mounted()) {
        send_json_error(req, "409 Conflict", "{\"error\":\"NOT_MOUNTED\"}");
        return false;
    }
    return true;
}

static bool fileop_try_lock(httpd_req_t *req)
{
    if (!s_fileop_mutex) {
        s_fileop_mutex = xSemaphoreCreateMutex();
        if (!s_fileop_mutex) {
            send_json_error(req, "500 Internal Server Error", "{\"error\":\"NO_MEM\"}");
            return false;
        }
    }
    if (xSemaphoreTake(s_fileop_mutex, 0) != pdTRUE) {
        send_json_error(req, "423 Locked", "{\"error\":\"FILEOP_IN_PROGRESS\"}");
        return false;
    }
    return true;
}

static void fileop_unlock(void)
{
    if (s_fileop_mutex) {
        xSemaphoreGive(s_fileop_mutex);
    }
}

bool web_fs_is_busy(void)
{
    if (!s_fileop_mutex) {
        return false;
    }
    if (xSemaphoreTake(s_fileop_mutex, 0) == pdTRUE) {
        xSemaphoreGive(s_fileop_mutex);
        return false;
    }
    return true;
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

static bool normalize_path(const char *input, char *out, size_t out_len)
{
    if (!out || out_len < 2) {
        return false;
    }
    if (!input || input[0] == '\0') {
        strncpy(out, "/", out_len);
        return true;
    }

    char tmp[MAX_PATH_LEN];
    if (input[0] == '/') {
        strncpy(tmp, input, sizeof(tmp));
        tmp[sizeof(tmp) - 1] = '\0';
    } else {
        snprintf(tmp, sizeof(tmp), "/%s", input);
    }

    size_t di = 0;
    out[di++] = '/';
    const char *p = tmp;
    if (*p == '/') {
        p++;
    }
    while (*p) {
        const char *seg = p;
        while (*p && *p != '/') {
            p++;
        }
        size_t seg_len = (size_t)(p - seg);
        if (seg_len == 0) {
            if (*p == '/') {
                p++;
            }
            continue;
        }
        if (seg_len == 1 && seg[0] == '.') {
            if (*p == '/') {
                p++;
            }
            continue;
        }
        if (seg_len == 2 && seg[0] == '.' && seg[1] == '.') {
            return false;
        }
        if (di + seg_len + 1 >= out_len) {
            return false;
        }
        if (di > 1) {
            out[di++] = '/';
        }
        memcpy(out + di, seg, seg_len);
        di += seg_len;
        if (*p == '/') {
            p++;
        }
    }
    out[di] = '\0';
    if (di == 0) {
        strncpy(out, "/", out_len);
    }
    return true;
}

static bool sanitize_name(const char *input, char *out, size_t out_len)
{
    if (!input || input[0] == '\0' || !out || out_len == 0) {
        return false;
    }
    size_t di = 0;
    for (size_t si = 0; input[si] && di + 1 < out_len; ++si) {
        unsigned char ch = (unsigned char)input[si];
        if (ch < 32 || ch == '/' || ch == '\\') {
            return false;
        }
        out[di++] = (char)ch;
    }
    out[di] = '\0';
    if (strcmp(out, ".") == 0 || strcmp(out, "..") == 0) {
        return false;
    }
    return di > 0;
}

static void json_escape(char *dst, size_t dst_len, const char *src)
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

static bool build_fs_path(const char *rel_path, char *out, size_t out_len)
{
    const char *mount = sdcard_mount_point();
    if (!mount || !out || out_len == 0) {
        return false;
    }
    size_t mount_len = strlen(mount);
    size_t rel_len = strlen(rel_path);
    size_t needed = 0;
    if (strcmp(rel_path, "/") == 0) {
        needed = mount_len + 1;
        if (needed > out_len) {
            return false;
        }
        memcpy(out, mount, mount_len);
        out[mount_len] = '\0';
        return true;
    }
    needed = mount_len + rel_len + 1;
    if (needed > out_len) {
        return false;
    }
    memcpy(out, mount, mount_len);
    memcpy(out + mount_len, rel_path, rel_len);
    out[mount_len + rel_len] = '\0';
    return true;
}

static bool build_rel_child(const char *base, const char *name, char *out, size_t out_len)
{
    if (!base || !name || !out || out_len == 0) {
        return false;
    }
    size_t base_len = strlen(base);
    size_t name_len = strlen(name);
    bool is_root = (strcmp(base, "/") == 0);
    size_t needed = (is_root ? 1 + name_len : base_len + 1 + name_len) + 1;
    if (needed > out_len) {
        return false;
    }
    if (is_root) {
        out[0] = '/';
        memcpy(out + 1, name, name_len);
        out[1 + name_len] = '\0';
        return true;
    }
    memcpy(out, base, base_len);
    out[base_len] = '/';
    memcpy(out + base_len + 1, name, name_len);
    out[base_len + 1 + name_len] = '\0';
    return true;
}

static bool build_suffix_path(const char *path, const char *suffix, char *out, size_t out_len)
{
    if (!path || !suffix || !out || out_len == 0) {
        return false;
    }
    size_t path_len = strlen(path);
    size_t suffix_len = strlen(suffix);
    size_t needed = path_len + suffix_len + 1;
    if (needed > out_len) {
        return false;
    }
    memcpy(out, path, path_len);
    memcpy(out + path_len, suffix, suffix_len);
    out[path_len + suffix_len] = '\0';
    return true;
}

static void make_content_disposition(char *out, size_t out_len, const char *filename)
{
    const char *prefix = "attachment; filename=\"";
    const char *suffix = "\"";
    const char *fallback = "download.bin";
    const char *name = (filename && filename[0]) ? filename : fallback;
    size_t prefix_len = strlen(prefix);
    size_t suffix_len = strlen(suffix);
    size_t name_len = strlen(name);
    if (!out || out_len == 0) {
        return;
    }
    if (prefix_len + suffix_len + 1 > out_len) {
        strncpy(out, "attachment", out_len);
        out[out_len - 1] = '\0';
        return;
    }
    if (prefix_len + name_len + suffix_len + 1 > out_len) {
        name = fallback;
        name_len = strlen(name);
    }
    if (prefix_len + name_len + suffix_len + 1 > out_len) {
        strncpy(out, "attachment", out_len);
        out[out_len - 1] = '\0';
        return;
    }
    memcpy(out, prefix, prefix_len);
    memcpy(out + prefix_len, name, name_len);
    memcpy(out + prefix_len + name_len, suffix, suffix_len);
    out[prefix_len + name_len + suffix_len] = '\0';
}

static bool get_query_path(httpd_req_t *req, char *out, size_t out_len)
{
    char query[MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        strncpy(out, "/", out_len);
        out[out_len - 1] = '\0';
        return true;
    }
    char raw[MAX_PATH_LEN] = {0};
    if (httpd_query_key_value(query, "path", raw, sizeof(raw)) != ESP_OK) {
        strncpy(out, "/", out_len);
        out[out_len - 1] = '\0';
        return true;
    }
    char decoded[MAX_PATH_LEN];
    url_decode(decoded, sizeof(decoded), raw);
    return normalize_path(decoded, out, out_len);
}

static bool get_query_flag(httpd_req_t *req, const char *key)
{
    char query[MAX_QUERY_LEN] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return false;
    }
    char val[8] = {0};
    if (httpd_query_key_value(query, key, val, sizeof(val)) != ESP_OK) {
        return false;
    }
    for (size_t i = 0; val[i] && i < sizeof(val); ++i) {
        val[i] = (char)tolower((unsigned char)val[i]);
    }
    return (strcmp(val, "1") == 0 || strcmp(val, "true") == 0 || strcmp(val, "yes") == 0 || strcmp(val, "on") == 0);
}

static bool read_body(httpd_req_t *req, char *buf, size_t buf_len)
{
    int total = req->content_len;
    if (total <= 0 || (size_t)total >= buf_len) {
        return false;
    }
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            return false;
        }
        received += r;
    }
    buf[received] = '\0';
    return true;
}

static bool json_get_string(const char *body, const char *key, char *out, size_t out_len)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(body, pattern);
    if (!p) {
        return false;
    }
    p += strlen(pattern);
    while (*p && *p != ':') {
        p++;
    }
    if (*p != ':') {
        return false;
    }
    p++;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p == '"') {
        p++;
        const char *end = strchr(p, '"');
        if (!end) {
            return false;
        }
        size_t len = (size_t)(end - p);
        if (len + 1 > out_len) {
            return false;
        }
        memcpy(out, p, len);
        out[len] = '\0';
        return true;
    }
    const char *end = p;
    while (*end && *end != ',' && *end != '}') {
        end++;
    }
    size_t len = (size_t)(end - p);
    while (len > 0 && isspace((unsigned char)p[len - 1])) {
        len--;
    }
    if (len + 1 > out_len) {
        return false;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static void drain_body(httpd_req_t *req)
{
    char buf[128];
    int remaining = req->content_len;
    while (remaining > 0) {
        int r = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            break;
        }
        remaining -= r;
    }
}

static esp_err_t http_fs_list(httpd_req_t *req)
{
    if (!fs_gate(req)) {
        return ESP_OK;
    }

    char rel_path[MAX_PATH_LEN];
    if (!get_query_path(req, rel_path, sizeof(rel_path))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_PATH\"}");
        return ESP_OK;
    }

    char full_path[MAX_PATH_LEN];
    if (!build_fs_path(rel_path, full_path, sizeof(full_path))) {
        send_json_error(req, "500 Internal Server Error", "{\"error\":\"PATH_FAIL\"}");
        return ESP_OK;
    }

    DIR *dir = opendir(full_path);
    if (!dir) {
        send_json_error(req, "404 Not Found", "{\"error\":\"NOT_FOUND\"}");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "{\"path\":\"");
    httpd_resp_sendstr_chunk(req, rel_path);
    httpd_resp_sendstr_chunk(req, "\",\"items\":[");

    bool first = true;
    struct dirent *ent = NULL;
    while ((ent = readdir(dir)) != NULL) {
        if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) {
            continue;
        }
        char item_rel[MAX_PATH_LEN];
        if (!build_rel_child(rel_path, ent->d_name, item_rel, sizeof(item_rel))) {
            continue;
        }
        char item_full[MAX_PATH_LEN];
        if (!build_fs_path(item_rel, item_full, sizeof(item_full))) {
            continue;
        }
        struct stat st;
        if (stat(item_full, &st) != 0) {
            continue;
        }
        if (!first) {
            httpd_resp_sendstr_chunk(req, ",");
        }
        first = false;

        char safe_name[MAX_NAME_LEN];
        json_escape(safe_name, sizeof(safe_name), ent->d_name);
        if (S_ISDIR(st.st_mode)) {
            char line[160];
            snprintf(line, sizeof(line), "{\"name\":\"%s\",\"type\":\"dir\"}", safe_name);
            httpd_resp_sendstr_chunk(req, line);
        } else {
            char line[200];
            snprintf(line, sizeof(line), "{\"name\":\"%s\",\"type\":\"file\",\"size\":%ld}",
                     safe_name, (long)st.st_size);
            httpd_resp_sendstr_chunk(req, line);
        }
    }
    closedir(dir);
    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static bool extract_filename(const char *header, char *out, size_t out_len)
{
    const char *p = strstr(header, "filename=\"");
    if (!p) {
        return false;
    }
    p += strlen("filename=\"");
    const char *end = strchr(p, '"');
    if (!end) {
        return false;
    }
    size_t len = (size_t)(end - p);
    if (len + 1 > out_len) {
        return false;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static esp_err_t http_fs_upload(httpd_req_t *req)
{
    if (!fs_gate(req)) {
        drain_body(req);
        return ESP_OK;
    }
    if (!fileop_try_lock(req)) {
        drain_body(req);
        return ESP_OK;
    }

    esp_err_t result = ESP_OK;
    char rel_dir[MAX_PATH_LEN];
    if (!get_query_path(req, rel_dir, sizeof(rel_dir))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_PATH\"}");
        goto cleanup;
    }
    bool overwrite = get_query_flag(req, "overwrite");

    char content_type[128];
    if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type)) != ESP_OK) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"NO_CONTENT_TYPE\"}");
        goto cleanup;
    }
    char *b = strstr(content_type, "boundary=");
    if (!b) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"NO_BOUNDARY\"}");
        goto cleanup;
    }
    b += strlen("boundary=");
    char boundary[70];
    strncpy(boundary, b, sizeof(boundary));
    boundary[sizeof(boundary) - 1] = '\0';
    char *semi = strchr(boundary, ';');
    if (semi) {
        *semi = '\0';
    }
    size_t blen = strlen(boundary);
    if (blen >= 2 && boundary[0] == '"' && boundary[blen - 1] == '"') {
        memmove(boundary, boundary + 1, blen - 2);
        boundary[blen - 2] = '\0';
    }
    if (boundary[0] == '\0') {
        send_json_error(req, "400 Bad Request", "{\"error\":\"NO_BOUNDARY\"}");
        goto cleanup;
    }

    char *header_buf = s_upload_header;
    int header_len = 0;
    bool header_done = false;
    char filename[MAX_NAME_LEN] = {0};

    char *buf = s_upload_buf;
    int remaining = req->content_len;
    int r = 0;
    while (!header_done && remaining > 0) {
        r = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (r <= 0) {
            send_json_error(req, "400 Bad Request", "{\"error\":\"RECV_FAIL\"}");
            goto cleanup;
        }
        remaining -= r;
        if (header_len < (int)UPLOAD_HEADER_SIZE - 1) {
            int to_copy = r;
            int space = (int)UPLOAD_HEADER_SIZE - 1 - header_len;
            if (to_copy > space) {
                to_copy = space;
            }
            if (to_copy > 0) {
                memcpy(header_buf + header_len, buf, to_copy);
                header_len += to_copy;
                header_buf[header_len] = '\0';
            }
        }
        size_t header_mark = 0;
        char *header_end = find_header_end(header_buf, (size_t)header_len, &header_mark);
        if (header_end) {
            header_done = true;
            if (!extract_filename(header_buf, filename, sizeof(filename))) {
                send_json_error(req, "400 Bad Request", "{\"error\":\"NO_FILENAME\"}");
                goto cleanup;
            }
            char clean_name[MAX_NAME_LEN];
            if (!sanitize_name(filename, clean_name, sizeof(clean_name))) {
                send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_NAME\"}");
                goto cleanup;
            }
            strncpy(filename, clean_name, sizeof(filename));
            filename[sizeof(filename) - 1] = '\0';

            char rel_file[MAX_PATH_LEN];
            if (!build_rel_child(rel_dir, filename, rel_file, sizeof(rel_file))) {
                send_json_error(req, "400 Bad Request", "{\"error\":\"PATH_TOO_LONG\"}");
                goto cleanup;
            }
            char full_path[MAX_PATH_LEN];
            if (!build_fs_path(rel_file, full_path, sizeof(full_path))) {
                send_json_error(req, "500 Internal Server Error", "{\"error\":\"PATH_FAIL\"}");
                goto cleanup;
            }
            struct stat st;
            if (stat(full_path, &st) == 0) {
                if (S_ISDIR(st.st_mode)) {
                    send_json_error(req, "409 Conflict", "{\"error\":\"IS_DIRECTORY\"}");
                    goto cleanup;
                }
                if (!overwrite) {
                    send_json_error(req, "409 Conflict", "{\"error\":\"FILE_EXISTS\"}");
                    goto cleanup;
                }
                if (unlink(full_path) != 0) {
                    send_json_error(req, "500 Internal Server Error", "{\"error\":\"DELETE_FAIL\"}");
                    goto cleanup;
                }
            }

            char tmp_path[MAX_PATH_LEN];
            if (!build_suffix_path(full_path, ".part", tmp_path, sizeof(tmp_path))) {
                send_json_error(req, "400 Bad Request", "{\"error\":\"PATH_TOO_LONG\"}");
                goto cleanup;
            }
            unlink(tmp_path);

            FILE *fp = fopen(tmp_path, "wb");
            if (!fp) {
                send_json_error(req, "500 Internal Server Error", "{\"error\":\"OPEN_FAIL\"}");
                goto cleanup;
            }

            char boundary_marker[80];
            snprintf(boundary_marker, sizeof(boundary_marker), "\r\n--%s", boundary);
            size_t marker_len = strlen(boundary_marker);
            if (marker_len + 1 > sizeof(s_upload_tail)) {
                fclose(fp);
                unlink(tmp_path);
                send_json_error(req, "400 Bad Request", "{\"error\":\"BOUNDARY_TOO_LONG\"}");
                goto cleanup;
            }
            char *tail = s_upload_tail;
            size_t tail_len = 0;

            char *data_start = header_end + (header_mark ? header_mark : 4);
            int data_len = header_len - (int)(data_start - header_buf);
            if (data_len > 0) {
                memcpy(buf, data_start, (size_t)data_len);
                r = data_len;
            } else {
                r = 0;
            }

            bool done = false;
            while (!done) {
                if (r == 0 && remaining > 0) {
                    r = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining);
                    if (r == HTTPD_SOCK_ERR_TIMEOUT) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                        r = 0;
                        continue;
                    }
                    if (r <= 0) {
                        fclose(fp);
                        unlink(tmp_path);
                        send_json_error(req, "400 Bad Request", "{\"error\":\"RECV_FAIL\"}");
                        goto cleanup;
                    }
                    remaining -= r;
                }
                if (r == 0 && remaining == 0) {
                    break;
                }

                char *work = s_upload_work;
                size_t work_len = 0;
                if (tail_len > 0) {
                    memcpy(work, tail, tail_len);
                    work_len += tail_len;
                }
                if (r > 0) {
                    memcpy(work + work_len, buf, (size_t)r);
                    work_len += (size_t)r;
                }

                char *pos = NULL;
                if (work_len >= marker_len) {
                    pos = find_seq(work, work_len, boundary_marker, marker_len);
                }
                if (pos) {
                    size_t data_bytes = (size_t)(pos - work);
                    if (data_bytes > 0) {
                        fwrite(work, 1, data_bytes, fp);
                    }
                    done = true;
                } else {
                    size_t keep = marker_len > 1 ? marker_len - 1 : 0;
                    if (work_len > keep) {
                        size_t write_len = work_len - keep;
                        fwrite(work, 1, write_len, fp);
                        if (keep > 0) {
                            memcpy(tail, work + write_len, keep);
                            tail_len = keep;
                        } else {
                            tail_len = 0;
                        }
                    } else {
                        memcpy(tail, work, work_len);
                        tail_len = work_len;
                    }
                }
                r = 0;
            }

            while (remaining > 0) {
                int d = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining);
                if (d == HTTPD_SOCK_ERR_TIMEOUT) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                if (d <= 0) {
                    break;
                }
                remaining -= d;
            }

            fflush(fp);
            fsync(fileno(fp));
            fclose(fp);

            if (rename(tmp_path, full_path) != 0) {
                unlink(tmp_path);
                send_json_error(req, "500 Internal Server Error", "{\"error\":\"RENAME_FAIL\"}");
                goto cleanup;
            }

            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
            goto cleanup;
        }
        if (!header_done && header_len >= (int)UPLOAD_HEADER_SIZE - 1) {
            send_json_error(req, "400 Bad Request", "{\"error\":\"HEADER_TOO_LARGE\"}");
            goto cleanup;
        }
    }

    send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_MULTIPART\"}");

cleanup:
    fileop_unlock();
    return result;
}

static esp_err_t http_fs_download(httpd_req_t *req)
{
    if (!fs_gate(req)) {
        return ESP_OK;
    }

    char rel_path[MAX_PATH_LEN];
    if (!get_query_path(req, rel_path, sizeof(rel_path))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_PATH\"}");
        return ESP_OK;
    }
    if (strcmp(rel_path, "/") == 0) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_PATH\"}");
        return ESP_OK;
    }

    char full_path[MAX_PATH_LEN];
    if (!build_fs_path(rel_path, full_path, sizeof(full_path))) {
        send_json_error(req, "500 Internal Server Error", "{\"error\":\"PATH_FAIL\"}");
        return ESP_OK;
    }

    struct stat st;
    if (stat(full_path, &st) != 0) {
        send_json_error(req, "404 Not Found", "{\"error\":\"NOT_FOUND\"}");
        return ESP_OK;
    }
    if (S_ISDIR(st.st_mode)) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"IS_DIRECTORY\"}");
        return ESP_OK;
    }

    const char *filename = strrchr(rel_path, '/');
    filename = filename ? filename + 1 : rel_path;

    char disp[128];
    make_content_disposition(disp, sizeof(disp), filename);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", disp);

    FILE *fp = fopen(full_path, "rb");
    if (!fp) {
        send_json_error(req, "500 Internal Server Error", "{\"error\":\"OPEN_FAIL\"}");
        return ESP_OK;
    }

    char buf[FILE_BUF_SIZE];
    size_t n = 0;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
            fclose(fp);
            return ESP_OK;
        }
    }
    fclose(fp);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t http_fs_mkdir(httpd_req_t *req)
{
    if (!fs_gate(req)) {
        return ESP_OK;
    }
    if (!fileop_try_lock(req)) {
        drain_body(req);
        return ESP_OK;
    }

    char body[MAX_BODY_LEN];
    if (!read_body(req, body, sizeof(body))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_BODY\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char path_raw[MAX_PATH_LEN] = "/";
    char name_raw[MAX_NAME_LEN] = {0};
    json_get_string(body, "path", path_raw, sizeof(path_raw));
    if (!json_get_string(body, "name", name_raw, sizeof(name_raw))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"NAME_REQUIRED\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char rel_dir[MAX_PATH_LEN];
    if (!normalize_path(path_raw, rel_dir, sizeof(rel_dir))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_PATH\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char name[MAX_NAME_LEN];
    if (!sanitize_name(name_raw, name, sizeof(name))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_NAME\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char rel_path[MAX_PATH_LEN];
    if (!build_rel_child(rel_dir, name, rel_path, sizeof(rel_path))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"PATH_TOO_LONG\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char full_path[MAX_PATH_LEN];
    if (!build_fs_path(rel_path, full_path, sizeof(full_path))) {
        send_json_error(req, "500 Internal Server Error", "{\"error\":\"PATH_FAIL\"}");
        fileop_unlock();
        return ESP_OK;
    }

    if (mkdir(full_path, 0775) != 0) {
        if (errno == EEXIST) {
            send_json_error(req, "409 Conflict", "{\"error\":\"FILE_EXISTS\"}");
        } else {
            send_json_error(req, "500 Internal Server Error", "{\"error\":\"MKDIR_FAIL\"}");
        }
        fileop_unlock();
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    fileop_unlock();
    return ESP_OK;
}

static esp_err_t http_fs_delete(httpd_req_t *req)
{
    if (!fs_gate(req)) {
        return ESP_OK;
    }
    if (!fileop_try_lock(req)) {
        drain_body(req);
        return ESP_OK;
    }

    char body[MAX_BODY_LEN];
    if (!read_body(req, body, sizeof(body))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_BODY\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char path_raw[MAX_PATH_LEN] = {0};
    if (!json_get_string(body, "path", path_raw, sizeof(path_raw))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"PATH_REQUIRED\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char rel_path[MAX_PATH_LEN];
    if (!normalize_path(path_raw, rel_path, sizeof(rel_path)) || strcmp(rel_path, "/") == 0) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_PATH\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char full_path[MAX_PATH_LEN];
    if (!build_fs_path(rel_path, full_path, sizeof(full_path))) {
        send_json_error(req, "500 Internal Server Error", "{\"error\":\"PATH_FAIL\"}");
        fileop_unlock();
        return ESP_OK;
    }

    struct stat st;
    if (stat(full_path, &st) != 0) {
        send_json_error(req, "404 Not Found", "{\"error\":\"NOT_FOUND\"}");
        fileop_unlock();
        return ESP_OK;
    }
    if (S_ISDIR(st.st_mode)) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"IS_DIRECTORY\"}");
        fileop_unlock();
        return ESP_OK;
    }

    if (unlink(full_path) != 0) {
        send_json_error(req, "500 Internal Server Error", "{\"error\":\"DELETE_FAIL\"}");
        fileop_unlock();
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    fileop_unlock();
    return ESP_OK;
}

static esp_err_t http_fs_rename(httpd_req_t *req)
{
    if (!fs_gate(req)) {
        return ESP_OK;
    }
    if (!fileop_try_lock(req)) {
        drain_body(req);
        return ESP_OK;
    }

    char body[MAX_BODY_LEN];
    if (!read_body(req, body, sizeof(body))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_BODY\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char path_raw[MAX_PATH_LEN] = {0};
    char new_raw[MAX_NAME_LEN] = {0};
    if (!json_get_string(body, "path", path_raw, sizeof(path_raw))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"PATH_REQUIRED\"}");
        fileop_unlock();
        return ESP_OK;
    }
    if (!json_get_string(body, "new_name", new_raw, sizeof(new_raw))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"NEW_NAME_REQUIRED\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char rel_old[MAX_PATH_LEN];
    if (!normalize_path(path_raw, rel_old, sizeof(rel_old)) || strcmp(rel_old, "/") == 0) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_PATH\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char new_name[MAX_NAME_LEN];
    if (!sanitize_name(new_raw, new_name, sizeof(new_name))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"BAD_NAME\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char dir_path[MAX_PATH_LEN];
    strncpy(dir_path, rel_old, sizeof(dir_path));
    dir_path[sizeof(dir_path) - 1] = '\0';
    char *slash = strrchr(dir_path, '/');
    if (!slash || slash == dir_path) {
        strncpy(dir_path, "/", sizeof(dir_path));
    } else {
        *slash = '\0';
    }

    char rel_new[MAX_PATH_LEN];
    if (!build_rel_child(dir_path, new_name, rel_new, sizeof(rel_new))) {
        send_json_error(req, "400 Bad Request", "{\"error\":\"PATH_TOO_LONG\"}");
        fileop_unlock();
        return ESP_OK;
    }

    char full_old[MAX_PATH_LEN];
    char full_new[MAX_PATH_LEN];
    if (!build_fs_path(rel_old, full_old, sizeof(full_old)) ||
        !build_fs_path(rel_new, full_new, sizeof(full_new))) {
        send_json_error(req, "500 Internal Server Error", "{\"error\":\"PATH_FAIL\"}");
        fileop_unlock();
        return ESP_OK;
    }

    struct stat st;
    if (stat(full_old, &st) != 0) {
        send_json_error(req, "404 Not Found", "{\"error\":\"NOT_FOUND\"}");
        fileop_unlock();
        return ESP_OK;
    }
    if (stat(full_new, &st) == 0) {
        send_json_error(req, "409 Conflict", "{\"error\":\"FILE_EXISTS\"}");
        fileop_unlock();
        return ESP_OK;
    }

    if (rename(full_old, full_new) != 0) {
        send_json_error(req, "500 Internal Server Error", "{\"error\":\"RENAME_FAIL\"}");
        fileop_unlock();
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    fileop_unlock();
    return ESP_OK;
}

static esp_err_t http_usb_detach(httpd_req_t *req)
{
    if (web_fs_is_busy()) {
        send_json_error(req, "423 Locked", "{\"error\":\"FILEOP_IN_PROGRESS\"}");
        return ESP_OK;
    }
    esp_err_t err = msc_detach();
    httpd_resp_set_type(req, "application/json");
    if (err == ESP_OK) {
        httpd_resp_send(req, "{\"ok\":true,\"mode\":\"DETACHED\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_resp_send(req, "{\"ok\":false,\"error\":\"DETACH_FAIL\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_usb_attach(httpd_req_t *req)
{
    if (web_fs_is_busy()) {
        send_json_error(req, "423 Locked", "{\"error\":\"FILEOP_IN_PROGRESS\"}");
        return ESP_OK;
    }
    esp_err_t err = msc_attach();
    httpd_resp_set_type(req, "application/json");
    if (err == ESP_OK) {
        httpd_resp_send(req, "{\"ok\":true,\"mode\":\"ATTACHED\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_resp_send(req, "{\"ok\":false,\"error\":\"ATTACH_FAIL\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t web_fs_register_handlers(httpd_handle_t server)
{
    if (!server) {
        return ESP_ERR_INVALID_ARG;
    }

    httpd_uri_t list = {
        .uri = "/api/fs/list",
        .method = HTTP_GET,
        .handler = http_fs_list,
        .user_ctx = NULL,
    };
    httpd_uri_t upload = {
        .uri = "/api/fs/upload",
        .method = HTTP_POST,
        .handler = http_fs_upload,
        .user_ctx = NULL,
    };
    httpd_uri_t download = {
        .uri = "/api/fs/download",
        .method = HTTP_GET,
        .handler = http_fs_download,
        .user_ctx = NULL,
    };
    httpd_uri_t mkdir_req = {
        .uri = "/api/fs/mkdir",
        .method = HTTP_POST,
        .handler = http_fs_mkdir,
        .user_ctx = NULL,
    };
    httpd_uri_t del_req = {
        .uri = "/api/fs/delete",
        .method = HTTP_POST,
        .handler = http_fs_delete,
        .user_ctx = NULL,
    };
    httpd_uri_t rename_req = {
        .uri = "/api/fs/rename",
        .method = HTTP_POST,
        .handler = http_fs_rename,
        .user_ctx = NULL,
    };
    httpd_uri_t usb_detach = {
        .uri = "/api/usb/detach",
        .method = HTTP_POST,
        .handler = http_usb_detach,
        .user_ctx = NULL,
    };
    httpd_uri_t usb_attach = {
        .uri = "/api/usb/attach",
        .method = HTTP_POST,
        .handler = http_usb_attach,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(server, &list);
    httpd_register_uri_handler(server, &upload);
    httpd_register_uri_handler(server, &download);
    httpd_register_uri_handler(server, &mkdir_req);
    httpd_register_uri_handler(server, &del_req);
    httpd_register_uri_handler(server, &rename_req);
    httpd_register_uri_handler(server, &usb_detach);
    httpd_register_uri_handler(server, &usb_attach);
    return ESP_OK;
}
