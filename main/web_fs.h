#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t web_fs_register_handlers(httpd_handle_t server);
bool web_fs_is_busy(void);
