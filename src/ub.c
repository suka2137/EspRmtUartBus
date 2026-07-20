#include <ctype.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"
#include "esp_vfs.h"

#include "common.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"
#include "rmt-uart-ub.h"
static const char* TAG = "RMT-UART-UB";

bool ru_bus_initialized = false;

typedef struct RMT_UART_DUAL {
    rmt_uart_tx_t tx;
    rmt_uart_rx_t rx;
    int id;
    SemaphoreHandle_t mtx;       // serializes task-context ops (write/read/deinit) vs teardown
    SemaphoreHandle_t user_mtx;  // external coordination mutex, independent of internal locking
} rmt_uart_dual_t;


static rmt_transmit_config_t TxConf = {.loop_count = 0, .flags.eot_level = 1};
static rmt_uart_dual_t* duals[4] = {NULL, NULL, NULL, NULL};

// TX-done ISR: flip the shared pin back to input and re-arm RX immediately so the
// node is listening again the instant the last stop bit drains. rmt_receive() is
// documented ISR-callable. No critical section: the only shared state touched is
// tx.active/rx.active (single-word stores, atomic on Xtensa) and the GPIO OE flip
// (an LL write). The RMT driver owns its own per-channel locking.
IRAM_ATTR static bool rmt_uart_dual_txDone_callback(rmt_channel_handle_t tx_chan,
                                                    const rmt_tx_done_event_data_t* edata,
                                                    void* user_data) {
    rmt_uart_dual_t* dual = user_data;
    // Spurious/done-during-init: tx is not busy, leave the line as-is.
    if(unlikely(dual->tx.active == 0)) return false;

    GPIO.func_out_sel_cfg[dual->tx.conf.gpio].oen_sel = 1;
    gpio_ll_output_disable(&GPIO, dual->tx.conf.gpio);
    dual->tx.active = 0;

    esp_err_t r = rmt_receive(dual->rx.rmtCh, dual->rx.recv_symbol_buffer,
                              dual->rx.rx_buf_bytes, &dual->rx.RxConf);
    if(r == ESP_OK) {
        dual->rx.active = 1;
    } else if(r == ESP_ERR_INVALID_STATE) {
        // RX got disabled (e.g. by a concurrent deinit path); re-enable then re-arm.
        rmt_enable(dual->rx.rmtCh);
        rmt_receive(dual->rx.rmtCh, dual->rx.recv_symbol_buffer,
                    dual->rx.rx_buf_bytes, &dual->rx.RxConf);
        dual->rx.active = 1;
    }
    return false;
}

esp_err_t RU_bus_add_channels(rmt_uart_spec_t* conf) {
    int id = 0;
    while(id < 4 && duals[id] != NULL) id++;
    ESP_RETURN_ON_FALSE(id < 4, ESP_FAIL, TAG, "all ids taken");
    if(conf->data_bits < 4 || conf->data_bits > 9 || conf->parity < 0 || conf->parity > 4 ||
       !(conf->stop_bits == RU_STOPS_1 || conf->stop_bits == RU_STOPS_1_5 ||
         conf->stop_bits == RU_STOPS_2) ||
       conf->rate > 3200000 || conf->rate == 0)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGD(TAG,
             "Creating %" PRIu32 " %d%c%d UB, id: %d, GPIO:%" PRIu32 "",
             conf->rate,
             conf->data_bits,
             rmt_uart_p_ch[conf->parity],
             conf->stop_bits,
             id,
             conf->gpio);
    duals[id] = malloc(sizeof(rmt_uart_dual_t));
    rmt_uart_dual_t* dual = duals[id];
    rmt_uart_dual_t empty = {0};
    *dual = empty;
    rmt_uart_tx_t* tx = &dual->tx;
    rmt_uart_rx_t* rx = &dual->rx;
    tx->conf = *conf;
    rx->conf = *conf;
    tx->selfId = id;
    tx->address_field = true;
    rx->selfId = id;
    rx->reenable_rx = 1;

    int mem_blocks = 1;
    rx->rx_buf_bytes = mem_blocks * 256;
    rx->recv_symbol_buffer = malloc(rx->rx_buf_bytes);

    struct freq_selection freqs = rmt_uart_choose_freq(conf);
    tx->basePeriod = freqs.basePeriod;

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_APB,
        .gpio_num = (uint32_t)(conf->gpio),
        .mem_block_symbols = mem_blocks * SOC_RMT_MEM_WORDS_PER_CHANNEL,
        .resolution_hz = freqs.rmt_resolution_hz,
        .trans_queue_depth = 1,
        .flags.invert_out = false,
#if !SOC_RMT_SUPPORT_DMA
        .flags.with_dma = 0,
#else
        .flags.with_dma = (id == 0),
#endif
        .flags.allow_pd = false,
        .flags.init_level = 1,
    };
    tx->TxChConf = tx_chan_config;

    int total_bits_enc = conf->data_bits + (conf->parity != 0) + 2;
    rmt_simple_encoder_config_t simple_encoder_cfg = {
        .callback = rmt_uart_encoder,
        .arg = tx,
        .min_chunk_size = total_bits_enc / 2,
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &tx->TxEncoder));

    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_APB,
        .gpio_num = (uint32_t)(conf->gpio),
        .mem_block_symbols = mem_blocks * SOC_RMT_MEM_WORDS_PER_CHANNEL,
        .resolution_hz = freqs.rmt_resolution_hz,
        .flags.invert_in = false,
#if !SOC_RMT_SUPPORT_DMA
        .flags.with_dma = 0,
#else
        .flags.with_dma = (id == 0),
#endif
        .flags.allow_pd = false,
    };
    rmt_receive_config_t RxConf = {
        .signal_range_max_ns = freqs.idle_range_ns,
        .signal_range_min_ns = freqs.glitch_filter_ns,
#ifdef CONFIG_IDF_TARGET_ESP32
        .flags.en_partial_rx = 0,
#else
        .flags.en_partial_rx = 1,
#endif
    };
    rx->RxChConf = rx_chan_config;
    rx->RxConf = RxConf;

    rmt_uart_decoder_state_t ds = {0};
    ds.minDuration = freqs.decoder_min_duration;

#ifdef CONFIG_IDF_TARGET_ESP32
    ds.outBufSize = 32 * mem_blocks;
#else
    ds.outBufSize = 1024;
#endif
    // destination might not be known when it needs to be quickly switched to rx
    ESP_LOGD(TAG, "%d: intermediary buf: %zuB", id, ds.outBufSize);
    ds.outBuf = malloc(ds.outBufSize);
    rx->decoder_state = ds;
    rx->recv_notify_queue = xQueueCreate(1, sizeof(rmt_uart_recv_notify_t));

    gpio_set_level(conf->gpio, 1);
    ESP_ERROR_CHECK(gpio_set_pull_mode(conf->gpio, GPIO_PULLUP_ONLY));
    gpio_hold_en(conf->gpio);

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx->RxChConf, &rx->rmtCh));
    rmt_rx_event_callbacks_t rxcb = {
        .on_recv_done = rmt_uart_recv_cb_decode,
    };
    rmt_rx_register_event_callbacks(rx->rmtCh, &rxcb, rx);
    ESP_ERROR_CHECK(rmt_enable(rx->rmtCh));
    rx->active = 0;  // rx begins enabled but not listening

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx->TxChConf, &tx->rmtCh));
    rmt_tx_event_callbacks_t txcb = {.on_trans_done = rmt_uart_dual_txDone_callback};
    rmt_tx_register_event_callbacks(tx->rmtCh, &txcb, dual);
    ESP_ERROR_CHECK(rmt_enable(tx->rmtCh));
    rmt_encoder_handle_t CopyEncoder;
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &CopyEncoder));
    rmt_symbol_word_t begin_high = {.level0 = 1, .duration0 = 8, .level1 = 1, .duration1 = 0};
    rmt_transmit(tx->rmtCh, CopyEncoder, &begin_high, sizeof(begin_high), &TxConf);
    rmt_tx_wait_all_done(tx->rmtCh, -1);
    rmt_del_encoder(CopyEncoder);

    gpio_ll_output_disable(&GPIO, conf->gpio);
    // Force the output enable signal to be sourced from bit n of GPIO_ENABLE_REG
    GPIO.func_out_sel_cfg[conf->gpio].oen_sel = 1;
    tx->active = 0;  // tx begins enabled but with output disabled

    gpio_pullup_en(conf->gpio);
    gpio_hold_dis(conf->gpio);

    dual->id = id;
    dual->mtx = xSemaphoreCreateMutex();
    dual->user_mtx = xSemaphoreCreateMutex();

    ru_bus_initialized = true;
    return ESP_OK;
}

// Take the per-dual mutex for the duration of a bus operation. This serializes
// write/read/deinit against each other and against teardown, so the dual pointer
// and its queue/buffers stay valid for the whole call without TOCTOU windows.
static rmt_uart_dual_t* dual_lock(ru_ub_id channel, TickType_t timeout) {
    if(channel < 0 || channel >= 4) return NULL;
    rmt_uart_dual_t* dual = duals[channel];
    if(dual == NULL) return NULL;
    if(xSemaphoreTake(dual->mtx, timeout) != pdTRUE) return NULL;
    // Re-check after acquiring: deinit may have torn us down between the NULL check
    // and the take. duals[] is mutated only under the mutex.
    if(duals[channel] != dual) {
        xSemaphoreGive(dual->mtx);
        return NULL;
    }
    return dual;
}

ssize_t RU_bus_write(ru_ub_id channel, uint8_t addr, const void* data, size_t size) {
    rmt_uart_dual_t* dual = dual_lock(channel, portMAX_DELAY);
    if(dual == NULL) return -1;
    if(!size) { xSemaphoreGive(dual->mtx); return 0; }

    // Drain any stale RX result so the ISR has queue space to push the next one.
    rmt_uart_recv_notify_t resp = {0};
    xQueueReceive(dual->rx.recv_notify_queue, &resp, 0);
    if(resp.result == RU_RECV_SUCCESS && resp.data != NULL)
        ESP_LOGW(TAG, "RU_bus_write cleared successful receive data");

    if(dual->rx.active) {
        // Restart RX cleanly so a partial buffer doesn't confuse the decoder.
        ESP_ERROR_CHECK(rmt_disable(dual->rx.rmtCh));
        ESP_ERROR_CHECK(rmt_enable(dual->rx.rmtCh));
        dual->rx.active = 0;
    }
    // Enable output and mark busy; the TX-done ISR will flip both back.
    GPIO.func_out_sel_cfg[dual->tx.conf.gpio].oen_sel = 1;
    gpio_ll_output_enable(&GPIO, dual->tx.conf.gpio);
    dual->tx.active = 1;

    dual->tx.tx_mpcm_target_addr = addr;
    ESP_ERROR_CHECK(rmt_transmit(dual->tx.rmtCh, dual->tx.TxEncoder, data, size, &TxConf));
    // Wait for the frame to finish so the ISR has re-armed RX by the time we return.
    rmt_tx_wait_all_done(dual->tx.rmtCh, -1);
    xSemaphoreGive(dual->mtx);
    return size;
}

ssize_t RU_bus_read(ru_ub_id channel, void* dst, size_t size) {
    rmt_uart_dual_t* dual = dual_lock(channel, portMAX_DELAY);
    if(dual == NULL) return -1;
    if(size > dual->rx.decoder_state.outBufSize) { xSemaphoreGive(dual->mtx); return -1; }
    if(!size) { xSemaphoreGive(dual->mtx); return 0; }
    if(dual->tx.active) {
        ESP_LOGE(TAG, "%d: read while tx still active", channel);
        xSemaphoreGive(dual->mtx);
        return -1;
    }

    // Drop any already-completed result from the queue so we block for a fresh one.
    rmt_uart_recv_notify_t resp = {0};
    xQueuePeek(dual->rx.recv_notify_queue, &resp, 0);
    if(resp.result != 0) xQueueReceive(dual->rx.recv_notify_queue, &resp, 0);

    if(!dual->rx.active) {
        ESP_ERROR_CHECK(rmt_receive(dual->rx.rmtCh, dual->rx.recv_symbol_buffer,
                                    dual->rx.rx_buf_bytes, &dual->rx.RxConf));
        dual->rx.active = 1;
    }

    // Block for a completed frame. The RX decode ISR fills the queue; during the
    // wait the mutex is released so other channels (or this channel's writer) can
    // proceed. We drop the lock across the blocking receive to avoid deadlocking
    // a higher-priority task that needs this channel.
    // NOTE: the RX path owns decoder_state/queue/buffer for the channel's lifetime,
    //       so the brief unlock is safe.
    xSemaphoreGive(dual->mtx);
    resp = (rmt_uart_recv_notify_t){0};
    if(xQueueReceive(dual->rx.recv_notify_queue, &resp, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "%d: receive timeout", channel);
        return -1;
    }
    if(resp.result != RU_RECV_SUCCESS) {
        ESP_LOGE(TAG, "%d: Receive error %d", channel, resp.result);
        return -1;
    }
    if(!resp.size) return 0;
    assert(resp.data != NULL);
    size = MIN(size, resp.size);
    memcpy(dst, resp.data, size);
    return size;
}

void RU_bus_get_mutex(ru_ub_id channel, TickType_t timeout) {
    assert(channel >= 0 && channel < 4);
    rmt_uart_dual_t* dual = duals[channel];
    assert(dual != NULL);
    xSemaphoreTake(dual->user_mtx, timeout);
}
void RU_bus_release_mutex(ru_ub_id channel) {
    assert(channel >= 0 && channel < 4);
    rmt_uart_dual_t* dual = duals[channel];
    assert(dual != NULL);
    xSemaphoreGive(dual->user_mtx);
}


static void UB_deinit(int id) {
    assert(id >= 0 && id < 4);
    rmt_uart_dual_t* dual = duals[id];
    if(dual == NULL) return;

    // Serialize against any in-flight write/read on this channel. portMAX_DELAY
    // because teardown must not be denied by a stuck peer.
    xSemaphoreTake(dual->mtx, portMAX_DELAY);

    // Also block external users from holding resources we're about to free.
    xSemaphoreTake(dual->user_mtx, portMAX_DELAY);

    // Quiesce hardware first. rmt_disable is synchronous: it stops the channel and
    // drains any pending interrupt, so once these return no TX-done / RX-done ISR
    // will fire on this core or the other. Unregistering callbacks afterward is
    // then race-free.
    rmt_tx_wait_all_done(dual->tx.rmtCh, -1);
    rmt_disable(dual->tx.rmtCh);
    rmt_disable(dual->rx.rmtCh);

    rmt_tx_event_callbacks_t tx_null = {.on_trans_done = NULL};
    rmt_rx_event_callbacks_t rx_null = {.on_recv_done = NULL};
    rmt_tx_register_event_callbacks(dual->tx.rmtCh, &tx_null, NULL);
    rmt_rx_register_event_callbacks(dual->rx.rmtCh, &rx_null, NULL);

    gpio_hold_en(dual->tx.conf.gpio);
    ESP_ERROR_CHECK(rmt_del_channel(dual->tx.rmtCh));
    ESP_ERROR_CHECK(rmt_del_channel(dual->rx.rmtCh));
    ESP_ERROR_CHECK(rmt_del_encoder(dual->tx.TxEncoder));
    gpio_reset_pin(dual->tx.conf.gpio);
    gpio_hold_dis(dual->tx.conf.gpio);

    // Now that the hardware is gone and ISRs quiesced, the queue/buffers are
    // exclusively ours. Null the handle last so any stale ISR context bails out.
    dual->tx.rmtCh = NULL;
    dual->rx.rmtCh = NULL;
    vQueueDelete(dual->rx.recv_notify_queue);
    free(dual->rx.recv_symbol_buffer);
    free(dual->rx.decoder_state.outBuf);

    duals[id] = NULL;
    xSemaphoreGive(dual->user_mtx);
    vSemaphoreDelete(dual->user_mtx);
    xSemaphoreGive(dual->mtx);
    vSemaphoreDelete(dual->mtx);
    free(dual);
}


void RU_bus_deinit() {
    for(int i = 0; i < 4; i++)
        if(duals[i] != NULL) UB_deinit(i);
    ru_bus_initialized = false;
}
