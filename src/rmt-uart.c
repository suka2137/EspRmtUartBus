#include "esp_log.h"
#include "hal/rmt_types.h"
#include "soc/clk_tree_defs.h"
#include "soc/soc.h"
#include "soc/soc_caps.h"

#include "common.h"

#define DATA_ERR_IF(cond, err, msg)                                 \
    if(unlikely(cond)) {                                            \
        res.result = err;                                           \
        if(msg) ESP_DRAM_LOGE("RU_DEC", "%d: %s", rx->selfId, msg); \
        goto data_err;                                              \
    }

#define RU_BUS_MASTER_ADDR 0x00

// typedef struct RMT_UART_DECODER_JOB {
//     rmt_rx_done_event_data_t event;
//     rmt_uart_rx_t* rx;
// } rmt_uart_decoder_job_t;

static const char* TAG = "RMT-UART";
static const char* err_strs[] = {"success", "part", "malformed", "fault", "drop/recover", "deinit"};
// static const char* state_strs[] = {"INACTIVE", "TX", "TX ONGOING", "RX", "RX ONGOING"};

static bool swglitch = (CONFIG_RU_FLT_MODE >= 2);
static rmt_transmit_config_t TxConf = {.loop_count = 0, .flags.eot_level = 1};

static rmt_uart_rx_t* rxs[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static rmt_uart_tx_t* txs[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};


bool ru_regular_uart_initialized;
// static bool rmtUartHib = 0;
static TaskHandle_t decoder_task __attribute__((unused)) = NULL;
static QueueHandle_t dec_q __attribute__((unused)) = NULL;

static void dec_print_symbols(const rmt_rx_done_event_data_t* edata, rmt_uart_decoder_state_t* dec);
// static void rmt_uart_decoder_task(void* params);
// static void dec_task_start();
IRAM_ATTR static bool byte_parity(uint8_t byte);

IRAM_ATTR size_t rmt_uart_encoder(const void* data,
                                  size_t data_size,
                                  size_t symbols_written,
                                  size_t symbols_free,
                                  rmt_symbol_word_t* symbols,
                                  bool* done,
                                  void* arg) {
    rmt_uart_tx_t* tx = arg;
    size_t data_pos = tx->data_pos;
    uint8_t* data_bytes = (uint8_t*)data;
    size_t symbol_pos = 0;

    uint16_t basePeriod = tx->basePeriod;
    uint8_t data_bits = tx->conf.data_bits;
    uint8_t parity = tx->conf.parity;
    uint8_t stops = tx->conf.stop_bits;
    uint8_t stop_index = data_bits + (parity != 0) + 1;
    uint8_t parity_index = stop_index - 1;
    uint8_t total_bits = stop_index + 1;

    if(data_pos >= data_size) {
        rmt_symbol_word_t EndTx = {.level0 = 1, .duration0 = 0, .level1 = 1, .duration1 = 0};
        symbols[0] = EndTx;
        tx->data_pos = 0;
        *done = 1;
        return 1;
    }
    bool mark_ninth = tx->address_field && symbols_written == 0;
    if(unlikely(mark_ninth)) {
        if(symbols_free - symbol_pos >= total_bits / 2) {
            uint8_t bv = tx->tx_mpcm_target_addr;
            uint16_t bits = (bv << 1) | (1 << stop_index) | (1 << 9);
            if(parity) {
                bool p = (parity == RU_PARITY_MARK || parity == RU_PARITY_ODD);
                if(parity == RU_PARITY_EVEN || parity == RU_PARITY_ODD) p = !byte_parity(bv);
                bits |= (p << parity_index);
            }

            uint8_t rl = 1;
            uint8_t rl_last = 0;
            for(uint16_t i = 0; i < total_bits; i++) {
                if((bits & 0b1) == ((bits & 0b10) != 0)) rl++;
                else {
                    if(rl_last) {
                        rmt_symbol_word_t s = {
                            .level0 = 0,
                            .duration0 = rl_last * basePeriod,
                            .level1 = 1,
                            .duration1 = rl * basePeriod,
                        };
                        symbols[symbol_pos++] = s;
                        rl_last = 0;
                    } else rl_last = rl;
                    rl = 1;
                }
                bits >>= 1;
            }

            if(stops != RU_STOPS_1) {
                uint16_t x = (stops == RU_STOPS_2) ? basePeriod : (basePeriod / 2);
                symbols[symbol_pos - 1].duration1 += x;
            }
        } else return 0;
    }

    while(symbols_free - symbol_pos >= total_bits / 2 && data_pos < data_size) {
        uint8_t bv = data_bytes[data_pos];

        uint16_t bits = (bv << 1) | (1 << stop_index);
        if(parity) {
            bool p = (parity == RU_PARITY_MARK || parity == RU_PARITY_ODD);
            if(parity == RU_PARITY_EVEN || parity == RU_PARITY_ODD) p = byte_parity(bv);
            bits |= (p << parity_index);
        }

        uint8_t rl = 1;
        uint8_t rl_last = 0;
        for(uint16_t i = 0; i < total_bits; i++) {
            if((bits & 0b1) == ((bits & 0b10) != 0)) rl++;
            else {
                if(rl_last) {
                    rmt_symbol_word_t s = {
                        .level0 = 0,
                        .duration0 = rl_last * basePeriod,
                        .level1 = 1,
                        .duration1 = rl * basePeriod,
                    };
                    symbols[symbol_pos++] = s;
                    rl_last = 0;
                } else rl_last = rl;
                rl = 1;
            }
            bits >>= 1;
        }

        if(stops != RU_STOPS_1) {
            uint16_t x = (stops == RU_STOPS_2) ? basePeriod : (basePeriod / 2);
            symbols[symbol_pos - 1].duration1 += x;
        }
        data_pos++;
    }
    tx->data_pos = data_pos;
    return symbol_pos;
}

// IRAM_ATTR static bool rmt_uart_recv_cb_dt(rmt_channel_handle_t channel,
//                                           const rmt_rx_done_event_data_t* edata,
//                                           void* user_data) {
//     rmt_uart_rx_t* rx = (rmt_uart_rx_t*)user_data;
//     assert(rx != NULL && edata != NULL);
//     if(unlikely(rx->active == 0 || rx->rmtCh == NULL)) return false;
//     rmt_uart_decoder_job_t dj = {
//         .event = *edata,
//         .rx = rx,
//     };
//
//     BaseType_t high_task_wakeup = pdFALSE;
//     xQueueSendFromISR(dec_q, &dj, &high_task_wakeup);
//     return high_task_wakeup == pdTRUE;
// }

IRAM_ATTR bool rmt_uart_recv_cb_decode(rmt_channel_handle_t channel,
                                       const rmt_rx_done_event_data_t* edata,
                                       void* user_data) {
    BaseType_t high_wakeup = false;
    rmt_uart_rx_t* rx = (rmt_uart_rx_t*)user_data;
    assert(rx != NULL);
    if(unlikely(rx->rmtCh == NULL)) return false;
    QueueHandle_t notify_q = rx->recv_notify_queue;
    rmt_symbol_word_t* symbols = edata->received_symbols;
    assert(symbols != NULL);

    // decoder state
    rmt_uart_decoder_state_t* decoder_state = &rx->decoder_state;
    uint32_t bits = decoder_state->bits;
    uint8_t bit = decoder_state->bit;
    size_t outBufPos = decoder_state->outBufPos;
    rmt_uart_rx_result_t lastResult = decoder_state->lastResult;
    // decoder info
    uint8_t* outBuf = decoder_state->outBuf;
    size_t outBufSize = decoder_state->outBufSize;
    uint32_t minDuration = decoder_state->minDuration;
    // default result
    rmt_uart_recv_notify_t res = {.result = RU_RECV_FAULT, .data = outBuf, .size = 0};

    // ESP_DRAM_LOGD("RU_DEC",
    //               "calling for %d, last: %s, q space: %d",
    //               rx->selfId,
    //               err_strs[lastResult],
    //               uxQueueSpacesAvailable(notify_q));

    DATA_ERR_IF((uxQueueSpacesAvailable(notify_q) == 0), RU_RECV_DROP_RECOVER, "queue full");
    DATA_ERR_IF(symbols[0].level0, RU_RECV_MALFORMED, "level0 is 1");
    if(lastResult == RU_RECV_MALFORMED) goto recover;  // wrong one wasn't last

    uint8_t parity = rx->conf.parity;
    uint8_t stop_index = rx->conf.data_bits + (parity != 0) + 1;
    uint8_t parity_index = stop_index - 1;
    uint8_t total_bits = stop_index + 1;

    uint32_t data_bits = rx->conf.data_bits;
    uint32_t check_stops_mask = (1 << stop_index) | (1 << 0);
    uint32_t val_mask = (0xFFFFFFFF >> (32 - data_bits)) & 0xFF;  // after 1 right shift

    uint16_t rl, rd = 0;
    uint16_t r = (CONFIG_RU_SLOW_EDGE) ? (decoder_state->minDuration / 5) : 0;
    //  dec_print_symbols(edata, decoder_state);
    for(rmt_symbol_word_t* s = symbols; s < symbols + edata->num_symbols; s++) {
        DATA_ERR_IF(s->duration0 == 0, RU_RECV_MALFORMED, "EOF 0");
        rl = (s->duration0 + r + rd) / minDuration;
        if(swglitch && !rl) rd += s->duration0;
        else {
            rd = 0;
            bit += rl;
            if(swglitch && (s->duration1 + r < minDuration))
                rd = (s->duration0 + r + rd) % minDuration;
        }

        rl = (s->duration1 + r + rd) / minDuration;
        if(s->duration1 == 0) rl = 12;  // level1 (always 1) is the EOF level
        else if(swglitch && (s->duration1 + r) < minDuration) {
            rd += s->duration1;
            continue;
        }

        rd = 0;
        bits |= 0xFFFFFFFF << bit;
        bit += rl;
        bits &= 0xFFFFFFFF >> (32 - bit);

        if(bit >= total_bits) {
            DATA_ERR_IF((bits & check_stops_mask) != (1 << stop_index),
                        RU_RECV_MALFORMED,
                        "wrong stop/start");
            bits >>= 1;
            uint8_t val = bits & val_mask;

            if(parity) {
                bool parity_expected;

                if(parity == RU_PARITY_EVEN || parity == RU_PARITY_ODD) {
                    parity_expected = byte_parity(val);
                    if(data_bits == 9 && (bits & (1 << 8))) parity_expected = !parity_expected;
                    if(parity == RU_PARITY_ODD) parity_expected = !parity_expected;
                } else parity_expected = (parity == RU_PARITY_MARK);

                bool parity_actual = bits & (1 << (parity_index - 1));
                DATA_ERR_IF(parity_actual != parity_expected, RU_RECV_MALFORMED, "parity");
            }
            DATA_ERR_IF(outBufPos == outBufSize, RU_RECV_FAULT, "buf ovf")
            // ESP_DRAM_LOGD("KKK", "%d: %d", outBufPos, val);
            outBuf[outBufPos++] = val;
            bit = 0;
            bits = 0;
            if(rl == 12) break;
        }
    }
    decoder_state->bits = bits;
    decoder_state->bit = bit;
    decoder_state->outBufPos = outBufPos;

    if(edata->flags.is_last) {
        DATA_ERR_IF((bit != 0 || outBufPos == 0) && edata->num_symbols > 1,
                    RU_RECV_MALFORMED,
                    "nothing written or frame incomplete despite eof");
        res.result = RU_RECV_SUCCESS;
        res.size = outBufPos;
        decoder_state->outBufPos = 0;
        decoder_state->lastResult = RU_RECV_SUCCESS;
        goto rx_reenable;
    } else {
        res.result = RU_RECV_PART;
        if(rx->defer == -1) {
            res.size = outBufPos;
            decoder_state->outBufPos = 0;
        } else {
            decoder_state->lastResult = RU_RECV_PART;
            return high_wakeup;
        }
    }

    decoder_state->lastResult = res.result;
    if(rx->defer == 0) xQueueSendFromISR(notify_q, &res, &high_wakeup);
    else xQueueSend(notify_q, &res, 0);
    return high_wakeup;

data_err:
    ESP_DRAM_LOGE("RU_DEC", "%d: %s (pos %d)", rx->selfId, err_strs[res.result], outBufPos);
    decoder_state->lastResult = res.result;
    dec_print_symbols(edata, decoder_state);
recover:
    if(edata->flags.is_last && res.result != RU_RECV_FAULT) {
        decoder_state->lastResult = RU_RECV_DROP_RECOVER;
        ESP_DRAM_LOGD("RU_DEC", "%d: recover", rx->selfId);
        decoder_state->outBufPos = 0;
rx_reenable:
        if(rx->reenable_rx) {
            esp_err_t res =
                rmt_receive(rx->rmtCh, rx->recv_symbol_buffer, rx->rx_buf_bytes, &rx->RxConf);
            if(res != ESP_OK) {
                ESP_DRAM_LOGW("RU_DEC", "%d: failing to reenable", rx->selfId);
                if(res == ESP_ERR_INVALID_STATE && rx->rmtCh) {
                    rmt_enable(rx->rmtCh);
                    ESP_ERROR_CHECK(rmt_receive(
                        rx->rmtCh, rx->recv_symbol_buffer, rx->rx_buf_bytes, &rx->RxConf));
                }
            }
        } else rx->active = 0;
    }

    decoder_state->bit = 0;
    decoder_state->bits = 0;
    if(uxQueueSpacesAvailable(notify_q) == 0) return high_wakeup;
    if(rx->defer == 0) xQueueSendFromISR(notify_q, &res, &high_wakeup);
    else xQueueSend(notify_q, &res, 0);
    return high_wakeup;
}

IRAM_ATTR static bool rmt_uart_tx_txDone_callback(rmt_channel_handle_t tx_chan,
                                                  const rmt_tx_done_event_data_t* edata,
                                                  void* user_data) {
    rmt_uart_tx_t* tx = (rmt_uart_tx_t*)user_data;
    assert(tx != NULL);
    if(unlikely(tx->rmtCh == NULL)) return false;

    tx->active = 0;
    TaskHandle_t notify = tx->notifyOnTxDone;
    if(notify == NULL) return pdFALSE;

    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveIndexedFromISR(notify, tx->notifyIndex, &high_task_wakeup);

    return high_task_wakeup == pdTRUE;
}

esp_err_t RU_init_tx(rmt_uart_id new_id, rmt_uart_spec_t* conf, uint8_t notify_index) {
    ESP_RETURN_ON_FALSE(new_id >= 0 && new_id < 8, ESP_ERR_INVALID_ARG, TAG, "id out of range");
    ESP_RETURN_ON_FALSE(
        txs[new_id] == NULL, ESP_ERR_INVALID_STATE, TAG, "%d: already initialized", new_id);
    ESP_RETURN_ON_FALSE(conf != NULL, ESP_ERR_INVALID_ARG, TAG, "conf NULL");
    ESP_RETURN_ON_FALSE(
        conf->data_bits >= 4 && conf->data_bits <= 9, ESP_ERR_INVALID_ARG, TAG, "data_bits");
    ESP_RETURN_ON_FALSE(conf->rate != 0 && conf->rate <= 3200000, ESP_ERR_INVALID_ARG, TAG, "rate");
    if(!ru_regular_uart_initialized) RMT_UART_begin();

    ESP_LOGD(TAG,
             "Creating %" PRIu32 " %d%c%d RMT UART TX on GPIO %" PRIu32 "",
             conf->rate,
             conf->data_bits,
             rmt_uart_p_ch[conf->parity],
             conf->stop_bits,
             conf->gpio);

    txs[new_id] = malloc(sizeof(rmt_uart_tx_t));
    ESP_RETURN_ON_FALSE(txs[new_id] != NULL, ESP_ERR_NO_MEM, TAG, "malloc");
    rmt_uart_tx_t* tx = txs[new_id];
    *tx = (rmt_uart_tx_t){0};
    tx->conf = *conf;
    tx->selfId = new_id;
    tx->notifyIndex = notify_index;
    tx->address_field = false;

    struct freq_selection freqs = rmt_uart_choose_freq(conf);
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_APB,
        .gpio_num = (uint32_t)(conf->gpio),
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
        .resolution_hz = freqs.rmt_resolution_hz,
        .trans_queue_depth = 1,
        .flags.invert_out = false,
#if !SOC_RMT_SUPPORT_DMA
        .flags.with_dma = 0,
#else
        .flags.with_dma = (new_id == 0),
#endif
        .flags.allow_pd = false,
        .flags.init_level = 1,
    };
    tx->TxChConf = tx_chan_config;
    tx->basePeriod = freqs.basePeriod;
    int total_bits_enc = conf->data_bits + (conf->parity != 0) + 2;
    rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = rmt_uart_encoder,
        .arg = tx,
        .min_chunk_size = total_bits_enc / 2,
    };

    esp_err_t ret;
    ESP_GOTO_ON_ERROR(
        rmt_new_simple_encoder(&simple_encoder_cfg, &tx->TxEncoder), err_free, TAG, "new encoder");
    ESP_GOTO_ON_ERROR(
        rmt_new_tx_channel(&tx->TxChConf, &tx->rmtCh), err_enc, TAG, "new tx channel");
    ESP_GOTO_ON_ERROR(rmt_enable(tx->rmtCh), err_chan, TAG, "enable tx");

    gpio_set_level(conf->gpio, 1);
    gpio_pullup_en(conf->gpio);
    gpio_hold_en(conf->gpio);

    rmt_encoder_handle_t copy_encoder;
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_GOTO_ON_ERROR(
        rmt_new_copy_encoder(&copy_cfg, &copy_encoder), err_chan, TAG, "new copy encoder");
    rmt_symbol_word_t begin_high = {.level0 = 1, .duration0 = 8, .level1 = 1, .duration1 = 0};
    rmt_transmit(tx->rmtCh, copy_encoder, &begin_high, sizeof(begin_high), &TxConf);
    rmt_tx_wait_all_done(tx->rmtCh, -1);
    rmt_del_encoder(copy_encoder);
    gpio_hold_dis(tx->conf.gpio);

    rmt_tx_event_callbacks_t cbs = {.on_trans_done = rmt_uart_tx_txDone_callback};
    ESP_GOTO_ON_ERROR(
        rmt_tx_register_event_callbacks(tx->rmtCh, &cbs, tx), err_chan, TAG, "register cb");
    tx->active = 0;
    ESP_LOGD(TAG, "%d Initialized tx", new_id);
    return ESP_OK;

err_chan:
    rmt_disable(tx->rmtCh);
    rmt_del_channel(tx->rmtCh);
err_enc:
    rmt_del_encoder(tx->TxEncoder);
err_free:
    free(tx);
    txs[new_id] = NULL;
    return ret;
}

esp_err_t RU_transmit(rmt_uart_id id, uint8_t* payload, size_t size, TaskHandle_t xTaskToNotify) {
    ESP_RETURN_ON_FALSE(id >= 0 && id < 8 && txs[id] != NULL, ESP_ERR_INVALID_ARG, TAG, "bad id");
    ESP_RETURN_ON_FALSE(!RU_tx_get_ongoing(id), ESP_ERR_INVALID_STATE, TAG, "%d: ongoing", id);
    rmt_uart_tx_t* tx = txs[id];
    ESP_LOGD(TAG, "%d: tx", id);
    tx->notifyOnTxDone = xTaskToNotify;
    tx->active = 1;
    ESP_ERROR_CHECK(rmt_transmit(tx->rmtCh, tx->TxEncoder, payload, size, &TxConf));
    return ESP_OK;
}

QueueHandle_t RU_init_rx(rmt_uart_id new_id, rmt_uart_spec_t* conf, int defer, bool continuous) {
    ESP_RETURN_ON_FALSE(new_id >= 0 && new_id < 8, NULL, TAG, "id out of range");
    ESP_RETURN_ON_FALSE(rxs[new_id] == NULL, NULL, TAG, "%d: already initialized", new_id);
    ESP_RETURN_ON_FALSE(conf != NULL, NULL, TAG, "conf NULL");
    ESP_RETURN_ON_FALSE(conf->data_bits >= 4 && conf->data_bits <= 9, NULL, TAG, "data_bits");
    ESP_RETURN_ON_FALSE(conf->rate != 0 && conf->rate <= 3200000, NULL, TAG, "rate");
#ifdef CONFIG_IDF_TARGET_ESP32
    if(defer == -1) {
        ESP_LOGE(TAG, "%d: -1 not supported", new_id);
        return NULL;
    }
#endif
    ESP_RETURN_ON_FALSE(defer <= 0, NULL, TAG, "%d: task decoding (defer>0) not supported", new_id);
    if(!ru_regular_uart_initialized) RMT_UART_begin();

    ESP_LOGD(TAG,
             "Creating %" PRIu32 " %d%c%d RMT UART RX on GPIO %" PRIu32 "",
             conf->rate,
             conf->data_bits,
             rmt_uart_p_ch[conf->parity],
             conf->stop_bits,
             conf->gpio);
    rxs[new_id] = malloc(sizeof(rmt_uart_rx_t));
    ESP_RETURN_ON_FALSE(rxs[new_id] != NULL, NULL, TAG, "malloc");
    rmt_uart_rx_t* rx = rxs[new_id];
    *rx = (rmt_uart_rx_t){0};
    rx->conf = *conf;
    rx->selfId = new_id;
    rx->reenable_rx = continuous;
    rx->defer = defer;

    int mem_blocks = 1;
    rx->rx_buf_bytes = mem_blocks * 256;
    rx->recv_symbol_buffer = malloc(rx->rx_buf_bytes);

    struct freq_selection freqs = rmt_uart_choose_freq(conf);
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_APB,
        .gpio_num = (uint32_t)(conf->gpio),
        .mem_block_symbols = mem_blocks * SOC_RMT_MEM_WORDS_PER_CHANNEL,
        .resolution_hz = freqs.rmt_resolution_hz,
        .flags.invert_in = false,
#if !SOC_RMT_SUPPORT_DMA
        .flags.with_dma = 0,
#else
        .flags.with_dma = (new_id == 0),
#endif
        .flags.allow_pd = false,
    };
    rmt_receive_config_t RxConf = {
        .signal_range_max_ns = freqs.idle_range_ns,
        .signal_range_min_ns = freqs.glitch_filter_ns,
#ifdef CONFIG_IDF_TARGET_ESP32
        .flags.en_partial_rx = 0,
#else
        .flags.en_partial_rx = true,
#endif
    };
    rx->RxChConf = rx_chan_config;
    rx->RxConf = RxConf;

    rmt_uart_decoder_state_t ds = {};
    ds.minDuration = freqs.decoder_min_duration;
    rx->decoder_state = ds;
    rx->recv_notify_queue = xQueueCreate(1, sizeof(rmt_uart_recv_notify_t));

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx->RxChConf, &rx->rmtCh));
    rmt_rx_event_callbacks_t cbs = {.on_recv_done = rmt_uart_recv_cb_decode};
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx->rmtCh, &cbs, rx));
    ESP_ERROR_CHECK(rmt_enable(rx->rmtCh));
    rx->active = 0;
    return rx->recv_notify_queue;
}

QueueHandle_t RU_recv(rmt_uart_id id, uint8_t* buffer, size_t bufferSize) {
    ESP_RETURN_ON_FALSE(id >= 0 && id < 8 && rxs[id] != NULL, NULL, TAG, "bad id");
    rmt_uart_rx_t* rx = rxs[id];
    ESP_RETURN_ON_FALSE(!RU_rx_get_ongoing(id), NULL, TAG, "%d: ongoing", id);
    rx->decoder_state.outBuf = buffer;
    rx->decoder_state.outBufSize = bufferSize;
    rx->decoder_state.outBufPos = 0;
    ESP_ERROR_CHECK(rmt_receive(rx->rmtCh, rx->recv_symbol_buffer, rx->rx_buf_bytes, &rx->RxConf));
    rx->active = 1;
    return rx->recv_notify_queue;
}

void RU_end_rx(rmt_uart_id id) {
    if(!RU_rx_get_ongoing(id)) return;
    rmt_uart_rx_t* rx = rxs[id];
    ESP_ERROR_CHECK(rmt_disable(rx->rmtCh));
    ESP_ERROR_CHECK(rmt_enable(rx->rmtCh));
    rx->active = 0;
}

// void RU_disable_all();  // TODO
// void RU_resume_all();   // TODO

void RMT_UART_begin() {
    ru_regular_uart_initialized = true;
}

void RMT_UART_end() {
    for(int i = 0; i < 8; i++) ESP_ERROR_CHECK(RU_deinit(i, 0));
    ru_regular_uart_initialized = false;
}

bool RU_tx_get_ongoing(const rmt_uart_id id) {
    assert(id >= 0 && id < 8 && txs[id] != NULL);
    return txs[id]->active;
}
bool RU_rx_get_ongoing(const rmt_uart_id id) {
    assert(id >= 0 && id < 8 && rxs[id] != NULL);
    return rxs[id]->active;
}

esp_err_t RU_deinit(rmt_uart_id id, bool wait) {
    assert(id >= 0 && id < 8);
    if(rxs[id] != NULL) {
        rmt_uart_rx_t* rx = rxs[id];
        if(RU_rx_get_ongoing(id)) {
            ESP_LOGW(TAG, "%d: RX Deinit: RX ongoing", id);
            RU_end_rx(id);
        }
        if(uxQueueMessagesWaiting(rx->recv_notify_queue)) {
            ESP_LOGE(TAG, "%d: RX Deinit: queue not empty", id);
            return -1;
        }

        rx->decoder_state.lastResult = RU_RECV_DEINIT;
        rmt_channel_handle_t ch = rx->rmtCh;
        rx->rmtCh = NULL;
        rx->reenable_rx = 0;
        rmt_disable(ch);
        if(wait) {
            rmt_uart_recv_notify_t r = {.data = NULL, .size = 0, .result = RU_RECV_DEINIT};
            xQueueSendToFront(rx->recv_notify_queue, &r, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vQueueDelete(rx->recv_notify_queue);
        free(rx->recv_symbol_buffer);
        ESP_ERROR_CHECK(rmt_del_channel(ch));
        free(rx);
        rxs[id] = NULL;
    }
    if(txs[id] != NULL) {
        rmt_uart_tx_t* tx = txs[id];
        rmt_channel_handle_t ch = tx->rmtCh;
        tx->notifyOnTxDone = NULL;
        rmt_tx_event_callbacks_t cbs = {.on_trans_done = NULL};
        rmt_tx_register_event_callbacks(ch, &cbs, NULL);
        rmt_tx_wait_all_done(ch, -1);
        gpio_hold_en(tx->conf.gpio);
        rmt_disable(ch);
        ESP_ERROR_CHECK(rmt_del_channel(ch));
        ESP_ERROR_CHECK(rmt_del_encoder(tx->TxEncoder));
        gpio_reset_pin(tx->conf.gpio);
        gpio_hold_dis(tx->conf.gpio);
        free(tx);
        txs[id] = NULL;
    }
    return ESP_OK;
}

// static void rmt_uart_decoder_task(void* params) {
//     dec_q = xQueueCreate(8, sizeof(rmt_uart_decoder_job_t));
//     rmt_uart_decoder_job_t currentItem;
//     while(1) {
//         xQueueReceive(dec_q, &currentItem, portMAX_DELAY);
//         rmt_uart_recv_cb_decode(currentItem.rx->rmtCh, &currentItem.event, currentItem.rx);
//     }
// }
// static void dec_task_start() {
//     ESP_LOGD(TAG, "Starting decoder");
//     BaseType_t res = xTaskCreatePinnedToCore(
//         rmt_uart_decoder_task, "RU_DEC", 2000, NULL, 2, &decoder_task, RMT_UART_AFFINITY);
//     assert(res == pdPASS);
//     ESP_LOGD(TAG, "Started decoder task");
// }

static bool byte_parity(uint8_t byte) {
    byte ^= byte >> 4;
    byte ^= byte >> 2;
    byte ^= byte >> 1;
    return byte & 1;
}
struct freq_selection rmt_uart_choose_freq(rmt_uart_spec_t* conf) {
    uint32_t max_freq = APB_CLK_FREQ;
    uint32_t max_filter_reg = RMT_LL_MAX_FILTER_VALUE;
    uint32_t max_idle_reg = RMT_LL_MAX_IDLE_VALUE;
    uint32_t minDuration_ns = 0.1 * 1e9 / conf->rate;  // target for glitch filter

    uint32_t max_high_bits_dec =
        conf->data_bits + (conf->parity != 0) + (1 + (conf->stop_bits == RU_STOPS_2)) + 1;

    uint32_t endDuration_ns =
        (uint32_t)((1000000000ULL * max_high_bits_dec) / (uint64_t)conf->rate);
    uint32_t basePeriod_max = 32767 / max_high_bits_dec;

    uint32_t div = 1;
    uint32_t idle_reg = (((uint64_t)max_freq * endDuration_ns) / 1000000000UL);
    uint32_t basePeriod = (max_freq + conf->rate / 2) / conf->rate;
    while(idle_reg > max_idle_reg || basePeriod > basePeriod_max) {
        div *= 2;
        idle_reg /= 2;
        basePeriod /= 2;
    }
    assert(div <= 256);
    max_freq /= div;

    uint32_t filter_reg = (((uint64_t)APB_CLK_FREQ * minDuration_ns) / 1000000000UL);
    if(filter_reg > max_filter_reg) {
        minDuration_ns =
            (uint32_t)((uint64_t)max_filter_reg * 1000000000ULL / (uint64_t)APB_CLK_FREQ);
    }

    // ESP32-S3 TRM: "the minimum value of pulse width should be larger than (3 × Tapb_clk + 5 ×
    // Trmt_sclk )."

    bool hwglitch = (CONFIG_RU_FLT_MODE == 1 || CONFIG_RU_FLT_MODE == 3);
    if(!hwglitch) minDuration_ns = 0;
    uint32_t decoder_min_duration =
        (CONFIG_RU_SLOW_EDGE) ? basePeriod : (basePeriod - (basePeriod / 25));

    assert(basePeriod >= 25);
    ESP_LOGD(TAG,
             "rate: %" PRIu32 " div: %" PRIu32 ", freq: %" PRIu32 ", idlereg: %" PRIu32 " (%" PRIu32
             " ns), filter_ns: %" PRIu32 ", bp: %" PRIu32 " min: %" PRIu32 "",
             conf->rate,
             div,
             max_freq,
             idle_reg,
             endDuration_ns,
             minDuration_ns,
             basePeriod,
             decoder_min_duration);
    struct freq_selection result = {
        .rmt_resolution_hz = max_freq,
        .glitch_filter_ns = minDuration_ns,
        .idle_range_ns = endDuration_ns,
        .basePeriod = basePeriod,
        .decoder_min_duration = decoder_min_duration,
    };
    return result;
}

static void dec_print_symbols(const rmt_rx_done_event_data_t* edata,
                              rmt_uart_decoder_state_t* dec) {
    if(LOG_LOCAL_LEVEL < ESP_LOG_DEBUG) return;
    assert(edata->received_symbols != NULL);

    ESP_DRAM_LOGD(
        TAG, "%lu symbols%s", edata->num_symbols, (edata->flags.is_last) ? " (Last):" : ":");
    for(int i = 0; i < edata->num_symbols && i < 3; i++) {
        ESP_DRAM_LOGD(TAG,
                      "%u %u",
                      (edata->received_symbols[i].level0) ? 1 : 0,
                      edata->received_symbols[i].duration0);
        ESP_DRAM_LOGD(TAG,
                      "%u %u",
                      (edata->received_symbols[i].level1) ? 1 : 0,
                      edata->received_symbols[i].duration1);
    }
    ESP_DRAM_LOGD(TAG, "...");
}
