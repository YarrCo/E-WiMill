#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    MSC_STATE_USB_ATTACHED,
    MSC_STATE_USB_DETACHED,
    MSC_STATE_DETACHING,
    MSC_STATE_ATTACHING,
    MSC_STATE_ERROR,
} msc_state_t;

void msc_init(void);
msc_state_t msc_get_state(void);
uint64_t msc_last_activity_ms(void);
esp_err_t msc_force_attach(void);
esp_err_t msc_force_detach(void);
esp_err_t msc_set_idle_timeout(uint32_t ms);
uint32_t msc_get_idle_timeout(void);

typedef enum {
    MSC_OP_COPYTEST,
} msc_op_type_t;

typedef struct {
    msc_op_type_t type;
    size_t size_mb;
} msc_op_t;

esp_err_t msc_enqueue_op(const msc_op_t *op);
void msc_queue_dump(void);
