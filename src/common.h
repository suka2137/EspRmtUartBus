#pragma once

#include <driver/rmt_common.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_types.h>
#include <hal/gpio_ll.h>
#include <hal/rmt_ll.h>
#include <inttypes.h>
#include <soc/rmt_reg.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/param.h>
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_compiler.h"
#include "esp_err.h"

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"


#include "rmt-uart.h"

extern bool rmt_uart_ready;

typedef struct RMT_UART_DECODER_STATE {
    rmt_uart_rx_result_t lastResult;
    uint32_t bits;
    uint8_t bit;
    uint16_t minDuration;

    uint8_t* outBuf;
    size_t outBufSize;
    size_t outBufPos;
} rmt_uart_decoder_state_t;
// typedef union {
//     struct {
//         uint8_t length0 : 4;
//         uint8_t length1 : 4;
//     };
//     uint8_t val;
// } rmt_uart_enc_item_t;
// typedef struct RMT_UART_ENCODER_TABLE_ENTRY {
//     uint8_t data, stops, parity;
//     rmt_uart_enc_item_t* parts[4];
//     uint8_t offs[256];
// } rmt_uart_enc_lookup_t;

typedef struct RMT_UART_TX {
    rmt_uart_spec_t conf;
    rmt_channel_handle_t rmtCh;
    rmt_uart_id selfId;

    rmt_tx_channel_config_t TxChConf;
    bool active;

    rmt_encoder_handle_t TxEncoder;
    size_t data_pos;
    uint16_t basePeriod;
    // rmt_uart_enc_lookup_t* table;

    uint8_t notifyIndex;
    TaskHandle_t notifyOnTxDone;
} rmt_uart_tx_t;

typedef struct RMT_UART_RX {
    rmt_uart_spec_t conf;
    rmt_channel_handle_t rmtCh;
    rmt_uart_id selfId;

    rmt_rx_channel_config_t RxChConf;
    rmt_receive_config_t RxConf;
    bool active;

    rmt_symbol_word_t* recv_symbol_buffer;
    size_t rx_buf_bytes;
    QueueHandle_t recv_notify_queue;
    rmt_uart_decoder_state_t decoder_state;
    bool reenable_rx;
    /*defer=-1: Partial recv in ISR, start pushing parts before completion (Not for ESP32).
    defer=0: Decoding in ISR with partial receiving if available.
    Other value: Symbol buffer size in bytes for decoding in a task*/
    int defer;
} rmt_uart_rx_t;

static const char rmt_uart_p_ch[] = "NEOMS";

size_t rmt_uart_encoder(const void* data,
                        size_t data_size,
                        size_t symbols_written,
                        size_t symbols_free,
                        rmt_symbol_word_t* symbols,
                        bool* done,
                        void* arg);
bool rmt_uart_recv_cb_decode(rmt_channel_handle_t channel,
                             const rmt_rx_done_event_data_t* edata,
                             void* user_data);


struct freq_selection {
    uint32_t rmt_resolution_hz;
    uint32_t glitch_filter_ns;
    uint32_t idle_range_ns;
    uint16_t basePeriod;
    uint16_t decoder_min_duration;
};
struct freq_selection rmt_uart_choose_freq(rmt_uart_spec_t* conf);

// rmt_uart_enc_lookup_t* rmt_uart_gen_enc_table(const rmt_uart_spec_t* conf);

void RMT_UART_begin();
void RMT_UART_end();
