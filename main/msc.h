#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    MSC_STATE_USB_ATTACHED,
    MSC_STATE_USB_DETACHED,
    MSC_STATE_ERROR,
} msc_state_t;

typedef struct {
    uint64_t read_bytes;
    uint64_t write_bytes;
    uint32_t read_fast_calls;
    uint32_t read_partial_calls;
    uint32_t write_fast_calls;
    uint32_t write_partial_calls;
    uint32_t read_buf_min;
    uint32_t read_buf_max;
    uint32_t write_buf_min;
    uint32_t write_buf_max;
    uint32_t cache_flushes;
    uint32_t cache_misses;
    uint32_t read_lba_min;
    uint32_t read_lba_max;
    uint32_t write_lba_min;
    uint32_t write_lba_max;
} msc_stats_t;

esp_err_t msc_init(void);
msc_state_t msc_get_state(void);
esp_err_t msc_attach(void);
esp_err_t msc_detach(void);
bool msc_is_host_connected(void);
void msc_stats_get(msc_stats_t *out_stats);
void msc_stats_reset(void);
