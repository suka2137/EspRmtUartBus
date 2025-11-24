#pragma once
#include "esp_err.h"
#include "rmt-uart.h"

#if CONFIG_RMT_RECV_FUNC_IN_IRAM == 0
#error CONFIG_RMT_RECV_FUNC_IN_IRAM is required
#endif

// Dual ids are 0-3
typedef int ru_ub_id;


esp_err_t RU_UB_init(ru_ub_id new_dual_id, rmt_uart_spec_t* conf);
esp_err_t UB_init_vfs();
int UB_deinit(ru_ub_id id);
ssize_t UB_write(ru_ub_id id, const void* data, size_t size);
ssize_t UB_read(ru_ub_id id, void* dst, size_t size);

void RMT_UART_UB_end();
