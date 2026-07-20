#pragma once
#include "esp_err.h"
#include "rmt-uart.h"

#if CONFIG_RMT_RECV_FUNC_IN_IRAM == 0
#error CONFIG_RMT_RECV_FUNC_IN_IRAM is required
#endif

typedef enum { RU_CH1 = 0, RU_CH2, RU_CH3, RU_CH4 } ru_ub_id;

typedef enum {
    RU_MASTER = 0,
    RU_DEV_1 = 1,
    RU_FALLBACK = 64,
    RU_BR_1 = 128,
    RU_DEBUG = 254,
    RU_BR_ALL = 255,
} ru_bus_addr_t;

// run before any other operations, autoincrements id
esp_err_t RU_bus_add_channels(rmt_uart_spec_t* conf);


// Drain any pending RX result, drive the address byte (9th bit set) then payload,
// then block until TX completes. The TX-done ISR re-arms RX on completion.
ssize_t RU_bus_write(ru_ub_id channel, uint8_t addr, const void* data, size_t size);

// no unsolicited msgs yet, the data stays in the original addr window
// ssize_t RU_bus_read(ru_ub_id channel, uint8_t own_addr, void* dst, size_t size);

// Wrapper for the read queue. Each read discards anything after size
// Setting a lower size wont stop the rx
ssize_t RU_bus_read(ru_ub_id channel, void* dst, size_t size);

// For high-level procedures.
void RU_bus_get_mutex(ru_ub_id channel, TickType_t timeout);
void RU_bus_release_mutex(ru_ub_id channel);

void RU_bus_deinit();


// esp_err_t UB_init_vfs();
