#include <ctype.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"
#include "esp_vfs.h"

#include "common.h"
#include "freertos/projdefs.h"
#include "rmt-uart-ub.h"
static const char* TAG = "RMT-UART-UB";

bool rmt_uart_ub_vfs_state = 0;

typedef struct RMT_UART_DUAL {
    rmt_uart_tx_t tx;
    rmt_uart_rx_t rx;
    int fd;
} rmt_uart_dual_t;

static rmt_transmit_config_t TxConf = {.loop_count = 0, .flags.eot_level = 1};
static rmt_uart_dual_t* duals[4] = {NULL, NULL, NULL, NULL};

IRAM_ATTR static bool rmt_uart_dual_txDone_callback(rmt_channel_handle_t tx_chan,
                                                    const rmt_tx_done_event_data_t* edata,
                                                    void* user_data) {
    rmt_uart_dual_t* dual = user_data;
    if(unlikely(dual->tx.active == 0)) return false;

    GPIO.func_out_sel_cfg[dual->tx.conf.gpio].oen_sel = 1;
    gpio_ll_output_disable(&GPIO, dual->tx.conf.gpio);
    dual->tx.active = 0;

    ESP_ERROR_CHECK(rmt_receive(
        dual->rx.rmtCh, dual->rx.recv_symbol_buffer, dual->rx.rx_buf_bytes, &dual->rx.RxConf));
    dual->rx.active = 1;
    return 0;
}


static int ub_open(const char* path, int flags, int mode) {
    ///"0.9e1.21.125000"

    int len = strlen(path);
    char s[25];
    for(int i = 1; i < len && i < 25; i++) s[i - 1] = toupper(path[i]);
    s[len - 1] = '\0';

    int id = atoi(strtok(s, "."));
    if(id < 0 || id > 3) {
        ESP_LOGE(TAG, "id %d out of range", id);
        return -1;
    }
    rmt_uart_dual_t* dual = duals[id];
    if(dual != NULL) {
        if(flags == O_RDWR || flags == O_RDONLY) {
            if(!dual->rx.active) {
                ESP_ERROR_CHECK(rmt_receive(dual->rx.rmtCh,
                                            dual->rx.recv_symbol_buffer,
                                            dual->rx.rx_buf_bytes,
                                            &dual->rx.RxConf));
                dual->rx.active = 1;
            }
        }
        return id;
    }

    char* tok = strtok(NULL, ".");
    int bits = tok[0] - '0';

    char* p = strchr(rmt_uart_p_ch, tok[1]);
    if(p == NULL) {
        ESP_LOGE(TAG, "parity: dec: %d", id);
        return -1;
    }
    int parity = p - rmt_uart_p_ch;

    int stops = atoi(tok + 2);

    tok = strtok(NULL, ".");
    uint32_t gpio = atoi(tok);

    tok = strtok(NULL, ".");
    uint32_t rate = atoi(tok);

    rmt_uart_spec_t conf = {
        .rate = rate,
        .gpio = gpio,
        .data_bits = bits,
        .parity = parity,
        .stop_bits = stops,
    };
    ESP_ERROR_CHECK(RU_UB_init(id, &conf));
    dual = duals[id];
    if(flags == O_RDWR || flags == O_RDONLY) {
        if(!dual->rx.active) {
            ESP_ERROR_CHECK(rmt_receive(dual->rx.rmtCh,
                                        dual->rx.recv_symbol_buffer,
                                        dual->rx.rx_buf_bytes,
                                        &dual->rx.RxConf));
            dual->rx.active = 1;
        }
    }
    return id;
}

// Clear RX Queue, write, switch back to RX
ssize_t UB_write(int fd, const void* data, size_t size) {
    // ESP_LOGD(TAG, "write %zu", size);
    if(fd < 0 || fd > 3) return -1;
    rmt_uart_dual_t* dual = duals[fd];
    assert(dual != NULL && data != NULL);
    if(!size) return 0;

    rmt_uart_recv_notify_t resp = {0};
    xQueueReceive(dual->rx.recv_notify_queue, &resp, 0);
    if(resp.result == RU_RECV_SUCCESS && resp.data != NULL)
        ESP_LOGW(TAG, "ub_write cleared successful receive data");
    if(dual->rx.active) {
        ESP_ERROR_CHECK(rmt_disable(dual->rx.rmtCh));
        ESP_ERROR_CHECK(rmt_enable(dual->rx.rmtCh));
        dual->rx.active = 0;
    }
    if(!dual->tx.active) {
        gpio_ll_output_enable(&GPIO, dual->tx.conf.gpio);
        dual->tx.active = 1;
    }
    ESP_ERROR_CHECK(rmt_transmit(dual->tx.rmtCh, dual->tx.TxEncoder, data, size, &TxConf));
    rmt_tx_wait_all_done(dual->tx.rmtCh, -1);
    return size;
}
// Each read discards anything after size
// Setting a lower size wont stop the rx
ssize_t UB_read(int fd, void* dst, size_t size) {
    // ESP_LOGD(TAG, "read %zu", size);
    if(fd < 0 || fd > 3) return -1;
    rmt_uart_dual_t* dual = duals[fd];
    assert(dual != NULL && dst != NULL);
    if(!size) return 0;
    if(size > dual->rx.decoder_state.outBufSize) return -1;
    rmt_uart_recv_notify_t resp = {0};
    rmt_uart_recv_notify_t empty = {0};
    xQueuePeek(dual->rx.recv_notify_queue, &resp, 0);
    if(resp.result != 0) xQueueReceive(dual->rx.recv_notify_queue, &resp, 0);

    if(dual->tx.active) {
        GPIO.func_out_sel_cfg[dual->tx.conf.gpio].oen_sel =
            1;  // Force the output enable signal to be sourced from bit n of GPIO_ENABLE_REG
        gpio_ll_output_disable(&GPIO, dual->tx.conf.gpio);
        dual->tx.active = 0;
    }
    if(!dual->rx.active) {
        ESP_ERROR_CHECK(rmt_receive(
            dual->rx.rmtCh, dual->rx.recv_symbol_buffer, dual->rx.rx_buf_bytes, &dual->rx.RxConf));
        dual->rx.active = 1;
    }

    resp = empty;
    xQueueReceive(dual->rx.recv_notify_queue, &resp, pdMS_TO_TICKS(100));
    if(resp.result != RU_RECV_SUCCESS) return -1;
    if(!resp.size) return 0;
    assert(resp.data != NULL);
    size = MIN(size, resp.size);
    memcpy(dst, resp.data, size);
    return size;
}

static const esp_vfs_t vfst = {
    .write = UB_write,
    .read = UB_read,
    .open = ub_open,
    .close = UB_deinit,
};
esp_err_t UB_init_vfs() {
    ESP_ERROR_CHECK(esp_vfs_register("/dev/ub", &vfst, NULL));

    rmt_uart_ub_vfs_state = 1;

    return ESP_OK;
}


esp_err_t RU_UB_init(ru_ub_id new_id, rmt_uart_spec_t* conf) {
    if(!rmt_uart_ready) RMT_UART_begin();
    assert(new_id >= 0 && new_id < 4);
    if(duals[new_id] != NULL) {
        ESP_LOGE(TAG, "%d: id taken", new_id);
        return -1;
    }
    ESP_LOGD(TAG,
             "Creating %" PRIu32 " %d%c%d UB, id: %d, GPIO:%" PRIu32 "",
             conf->rate,
             conf->data_bits,
             rmt_uart_p_ch[conf->parity],
             conf->stop_bits,
             new_id,
             conf->gpio);
    if(conf->data_bits < 4 || conf->data_bits > 9 || conf->parity < 0 || conf->parity > 4 ||
       !(conf->stop_bits == RU_STOPS_1 || conf->stop_bits == RU_STOPS_1_5 ||
         conf->stop_bits == RU_STOPS_2) ||
       conf->rate > 3200000 || conf->rate == 0)
        return ESP_ERR_INVALID_ARG;
    duals[new_id] = malloc(sizeof(rmt_uart_dual_t));
    rmt_uart_dual_t* dual = duals[new_id];
    rmt_uart_dual_t empty = {0};
    *dual = empty;
    rmt_uart_tx_t* tx = &dual->tx;
    rmt_uart_rx_t* rx = &dual->rx;
    tx->conf = *conf;
    rx->conf = *conf;
    tx->selfId = new_id;
    rx->selfId = new_id;
    rx->reenable_rx = 1;

    int mem_blocks = 1;
    rx->rx_buf_bytes = mem_blocks * 256;
    rx->recv_symbol_buffer = malloc(rx->rx_buf_bytes);

    // if(RMT_UART_NO_ENC_TABLE == 0) {
    //     tx->table = rmt_uart_gen_enc_table(conf);
    //     if(tx->table == NULL) {
    //         ESP_LOGE(TAG, "Gen table failed");
    //         return -1;
    //     }
    // }
    struct freq_selection freqs = rmt_uart_choose_freq(conf);
    tx->basePeriod = freqs.basePeriod;

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_APB,
        .gpio_num = (uint32_t)(conf->gpio),
        .mem_block_symbols = mem_blocks * SOC_RMT_MEM_WORDS_PER_CHANNEL,
        .resolution_hz = freqs.rmt_resolution_hz,
        .trans_queue_depth = 1,
        .flags.invert_out = false,
    //.flags.io_loop_back = 0, TODO
    //.flags.io_od_mode = 0,
#if !SOC_RMT_SUPPORT_DMA
        .flags.with_dma = 0,
#else
        .flags.with_dma = (new_id == 0),
#endif
        .flags.allow_pd = 0,
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
        .flags.with_dma = (new_id == 0),
#endif
        .flags.allow_pd = 0,
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
    ESP_LOGD(TAG, "%d: intermediary buf: %zuB", new_id, ds.outBufSize);
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

    dual->fd = new_id;
    return ESP_OK;
}

int UB_deinit(int fd) {
    assert(fd >= 0 && fd < 4);
    rmt_uart_dual_t* dual = duals[fd];
    if(dual == NULL) return 0;

    dual->tx.active = 0;
    rmt_tx_wait_all_done(dual->tx.rmtCh, -1);
    gpio_hold_en(dual->tx.conf.gpio);
    rmt_disable(dual->tx.rmtCh);

    dual->rx.reenable_rx = 0;
    dual->rx.active = 0;
    rmt_rx_event_callbacks_t cbs = {.on_recv_done = NULL};
    rmt_rx_register_event_callbacks(dual->rx.rmtCh, &cbs, NULL);
    vTaskDelay(pdMS_TO_TICKS(10));
    rmt_disable(dual->rx.rmtCh);
    free(dual->rx.decoder_state.outBuf);

    rmt_del_channel(dual->tx.rmtCh);
    rmt_del_channel(dual->rx.rmtCh);
    rmt_del_encoder(dual->tx.TxEncoder);
    gpio_reset_pin(dual->tx.conf.gpio);
    gpio_hold_dis(dual->tx.conf.gpio);

    vQueueDelete(dual->rx.recv_notify_queue);
    free(dual->rx.recv_symbol_buffer);
    free(dual);
    duals[fd] = NULL;
    return 0;
}


void RMT_UART_UB_end() {
    for(int i = 0; i < 4; i++)
        if(duals[i] != NULL) UB_deinit(i);
    if(rmt_uart_ub_vfs_state) esp_vfs_unregister("/dev/ub");
}
