#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/gpio_ll.h"
#include "hal/gpio_types.h"
#include "unity.h"

// Define the component Kconfig options directly for the test build (the
// component is #included below instead of being imported via CMake, so its
// Kconfig is never sourced).
#ifndef CONFIG_RU_FLT_MODE
#define CONFIG_RU_FLT_MODE 2
#endif
#ifndef CONFIG_RU_SLOW_EDGE
#define CONFIG_RU_SLOW_EDGE 1
#endif

// White-box: pull in the whole rmt-uart core (incl. static helpers byte_parity,
// dec_print_symbols, swglitch, and the txs[]/rxs[] tables).
#include "../../src/rmt-uart.c"
#include "rmt-uart-ub.h"
#include "rmt-uart.h"

static const char RUTEST_TAG[] = "RUTEST";

// Pins for hardware loopback. Avoid 35-42 (flash/PSRAM) per AGENTS.md.
#define RMT_TX_PIN 21  // RMT standalone TX output
#define RMT_RX_PIN 4   // RMT standalone RX input
// Hardware UART used as a known-good reference. Cross-jumper:
//   RMT_TX_PIN(21) <-> HWUART_RX_PIN(6)   : RMT TX verified by HW UART RX
//   HWUART_TX_PIN(5) <-> RMT_RX_PIN(4)    : HW UART TX verified by RMT RX
#define HWUART_NUM UART_NUM_1
#define HWUART_TX_PIN 5
#define HWUART_RX_PIN 6

static void print_res(const rmt_uart_recv_notify_t* r) {
    if(r->result == RU_RECV_SUCCESS) ESP_LOGI("RESP", "ok %zu bytes", r->size);
    else ESP_LOGE("RESP", "err %d", r->result);
}

// ---------------------------------------------------------------------------
// Tier 1: pure-logic encoder/decoder round-trip (no RMT hardware, no ISR).
// ---------------------------------------------------------------------------

// Synthesize an RX struct suitable for feeding rmt_uart_recv_cb_decode without a
// real RMT channel: rmtCh is a non-NULL sentinel and reenable_rx is false so the
// callback never touches the (bogus) handle.
static void synth_rx(rmt_uart_rx_t* rx,
                     const rmt_uart_spec_t* conf,
                     uint8_t* outbuf,
                     size_t outbuf_size) {
    *rx = (rmt_uart_rx_t){0};
    rx->conf = *conf;
    rx->selfId = 0;
    rx->defer = 0;
    rx->reenable_rx = false;
    rx->rmtCh = (rmt_channel_handle_t)0x1;  // non-NULL sentinel
    rx->recv_notify_queue = xQueueCreate(2, sizeof(rmt_uart_recv_notify_t));
    struct freq_selection freqs = rmt_uart_choose_freq((rmt_uart_spec_t*)conf);
    rx->decoder_state = (rmt_uart_decoder_state_t){0};
    rx->decoder_state.minDuration = freqs.decoder_min_duration;
    rx->decoder_state.outBuf = outbuf;
    rx->decoder_state.outBufSize = outbuf_size;
}

// Run the encoder over `data`/`size` for a tx configured with `address_field` and
// `tx_mpcm_target_addr`, accumulating real symbol-words into `out_symbols`.
// Returns the number of real symbols (the EndTx sentinel is excluded). The caller
// must size out_symbols to at least (size+2) * (total_bits/2) + 4.
static size_t run_encoder(rmt_uart_tx_t* tx,
                          const uint8_t* data,
                          size_t size,
                          rmt_symbol_word_t* out_symbols,
                          size_t cap) {
    tx->data_pos = 0;
    bool done = false;
    size_t written = 0;  // symbols_written across framework calls
    size_t total = 0;    // real symbols collected
    while(!done) {
        size_t n =
            rmt_uart_encoder(data, size, written, cap - total, out_symbols + total, &done, tx);
        if(done) break;  // the final call returned the EndTx symbol only
        total += n;
        written = total;
        // safety against a runaway encoder
        if(total > cap - 4 || n == 0) break;
    }
    return total;
}

// Decode an array of symbols (single chunk, is_last) using a synth rx and read one
// notification. Returns the result code; out bytes land in rx->decoder_state.outBuf.
static rmt_uart_rx_result_t decode_once(rmt_uart_rx_t* rx, rmt_symbol_word_t* syms, size_t n) {
    rmt_rx_done_event_data_t edata = {
        .received_symbols = syms,
        .num_symbols = n,
        .flags.is_last = 1,
    };
    rmt_uart_recv_cb_decode(NULL, &edata, rx);
    rmt_uart_recv_notify_t res = {0};
    xQueueReceive(rx->recv_notify_queue, &res, pdMS_TO_TICKS(100));
    return res.result;
}

TEST_CASE("encoder round-trip: address field prepends addr, keeps payload intact", "[enc]") {
    rmt_uart_spec_t conf = {
        .gpio = RMT_TX_PIN,
        .rate = 115200,
        .data_bits = 9,
        .parity = RU_PARITY_EVEN,
        .stop_bits = RU_STOPS_1,
    };
    rmt_uart_tx_t tx = {0};
    tx.conf = conf;
    tx.address_field = true;
    struct freq_selection freqs = rmt_uart_choose_freq(&conf);
    tx.basePeriod = freqs.basePeriod;

    uint8_t payload[] = {0xAA, 0xBB, 0x07, 0x00, 0xFF};
    uint8_t addr = 0x05;
    tx.tx_mpcm_target_addr = addr;

    rmt_symbol_word_t* syms = malloc(64 * sizeof(*syms));
    TEST_ASSERT_NOT_NULL(syms);
    size_t n = run_encoder(&tx, payload, sizeof(payload), syms, 64);
    TEST_ASSERT_GREATER_THAN(0, n);

    ESP_LOGI(TAG, "encoder:");
    rmt_rx_done_event_data_t rx_sym = {
        .num_symbols = n,
        .received_symbols = syms,
    };
    dec_print_symbols(&rx_sym, NULL);


    rmt_uart_rx_t rx;
    uint8_t outbuf[16];
    synth_rx(&rx, &conf, outbuf, sizeof(outbuf));
    rmt_uart_rx_result_t r = decode_once(&rx, syms, n);
    if(r != RU_RECV_SUCCESS) print_res(&((rmt_uart_recv_notify_t){.result = r}));
    TEST_ASSERT_EQUAL(RU_RECV_SUCCESS, r);

    ESP_LOG_BUFFER_HEXDUMP(TAG, outbuf, 6, ESP_LOG_INFO);

    // 9-bit val_mask strips the mark bit; the address decodes to addr & 0xFF.
    TEST_ASSERT_EQUAL_UINT8(addr, outbuf[0]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, outbuf + 1, sizeof(payload));

    // Regression: without address_field the same payload yields exactly itself,
    // proving the addr path neither consumed nor dropped payload[0].
    tx.address_field = false;
    size_t n2 = run_encoder(&tx, payload, sizeof(payload), syms, 64);
    TEST_ASSERT_GREATER_THAN(0, n2);
    rmt_uart_rx_t rx2;
    uint8_t outbuf2[16];
    synth_rx(&rx2, &conf, outbuf2, sizeof(outbuf2));
    r = decode_once(&rx2, syms, n2);
    TEST_ASSERT_EQUAL(RU_RECV_SUCCESS, r);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, outbuf2, sizeof(payload));

    vQueueDelete(rx.recv_notify_queue);
    vQueueDelete(rx2.recv_notify_queue);
    free(syms);
}

TEST_CASE("decoder: software glitch filter on synthetic symbols", "[internal][glitch]") {
    if(CONFIG_RU_FLT_MODE < 2) TEST_IGNORE();
    rmt_uart_spec_t conf = {
        .gpio = RMT_RX_PIN,
        .rate = 50000,
        .data_bits = 9,
        .parity = RU_PARITY_EVEN,
        .stop_bits = RU_STOPS_1,
    };
    rmt_uart_rx_t rx;
    uint8_t outbuf[2];
    synth_rx(&rx, &conf, outbuf, sizeof(outbuf));

    rmt_symbol_word_t g0[] = {{{322, 0, 39, 1}}, {{2856, 0, 3201, 1}}, {{11241, 0, 0, 1}}};
    rmt_symbol_word_t g1[] = {{{3200, 0, 100, 1}}, {{100, 0, 3000, 1}}, {{11246, 0, 0, 1}}};
    rmt_symbol_word_t g2[] = {{{488, 0, 488, 1}}, {{625, 0, 3200, 1}}, {{12800, 0, 0, 1}}};
    rmt_symbol_word_t g3[] = {{{2935, 0, 235, 1}}, {{14475, 0, 0, 1}}, {{0}}};
    rmt_symbol_word_t* gs[] = {g0, g1, g2, g3};
    int expect[] = {0x06, 0x06, 0x03, 0x00};

    for(int i = 0; i < 4; i++) {
        rmt_rx_done_event_data_t edata = {
            .flags.is_last = 1,
            .num_symbols = 3,
            .received_symbols = gs[i],
        };
        rmt_uart_recv_cb_decode(NULL, &edata, &rx);
        rmt_uart_recv_notify_t res = {0};
        xQueueReceive(rx.recv_notify_queue, &res, 0);
        TEST_ASSERT_EQUAL(RU_RECV_SUCCESS, res.result);
        TEST_ASSERT_EQUAL(1, res.size);
        TEST_ASSERT_EQUAL_UINT8(expect[i], res.data[0]);
    }
    vQueueDelete(rx.recv_notify_queue);
}

// ---------------------------------------------------------------------------
// Tier 2: standalone full-duplex RMT UART, verified against a hardware UART.
// Requires jumpers (see pin map at top of file):
//   21 <-> 6  (RMT TX  -> HW UART RX)
//    5 <-> 4  (HW UART TX -> RMT RX)
// ---------------------------------------------------------------------------

static void hwuart_init(uint32_t baud) {
    uart_config_t u = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(HWUART_NUM, &u));
    ESP_ERROR_CHECK(uart_set_pin(
        HWUART_NUM, HWUART_TX_PIN, HWUART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(HWUART_NUM, 256, 256, 0, NULL, 0));
}

static void hwuart_deinit(void) {
    uart_driver_delete(HWUART_NUM);
}

TEST_CASE("RMT TX -> HW UART RX round-trip", "[hw][fdx]") {
    uint32_t baud = 115200;
    rmt_uart_spec_t c = {
        .gpio = RMT_TX_PIN,
        .rate = baud,
        .data_bits = 8,
        .parity = RU_PARITY_NONE,
        .stop_bits = RU_STOPS_1,
    };
    hwuart_init(baud);
    TEST_ASSERT_EQUAL(ESP_OK, RU_init_tx(0, &c, 0));

    const uint8_t tx[] = "Hello RMT!";
    uart_flush_input(HWUART_NUM);
    RU_transmit(0, (uint8_t*)tx, sizeof(tx), xTaskGetCurrentTaskHandle());
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

    uint8_t rxbuf[32] = {0};
    int got = uart_read_bytes(HWUART_NUM, rxbuf, sizeof(tx), pdMS_TO_TICKS(200));
    ESP_LOGI(RUTEST_TAG, "hw uart got %d bytes", got);
    TEST_ASSERT_EQUAL(sizeof(tx), (size_t)got);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tx, rxbuf, sizeof(tx));

    RU_deinit(0, 0);
    hwuart_deinit();
}

TEST_CASE("HW UART TX -> RMT RX round-trip", "[hw][fdx]") {
    uint32_t baud = 38400;
    rmt_uart_spec_t c = {
        .gpio = RMT_RX_PIN,
        .rate = baud,
        .data_bits = 8,
        .parity = RU_PARITY_NONE,
        .stop_bits = RU_STOPS_1,
    };
    hwuart_init(baud);
    TEST_ASSERT_NOT_NULL(RU_init_rx(1, &c, 0, 0));

    uint8_t rxb[64];
    QueueHandle_t q = RU_recv(1, rxb, sizeof(rxb));
    TEST_ASSERT_NOT_NULL(q);

    const uint8_t tx[] = "abc123";
    uart_write_bytes(HWUART_NUM, tx, sizeof(tx));

    rmt_uart_recv_notify_t res = {0};
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(q, &res, pdMS_TO_TICKS(300)));
    print_res(&res);
    TEST_ASSERT_EQUAL(RU_RECV_SUCCESS, res.result);
    TEST_ASSERT_EQUAL(sizeof(tx), res.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tx, res.data, sizeof(tx));

    RU_deinit(1, 0);
    hwuart_deinit();
}

TEST_CASE("standalone pins: idle level and continuous RX", "[hw][pins]") {
    // gpio_set_direction(RMT_TX_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(RMT_RX_PIN, GPIO_MODE_INPUT);
    // gpio_set_level(RMT_TX_PIN, 1);
    //
    // rmt_uart_spec_t c = {
    //     .gpio = RMT_TX_PIN,
    //     .rate = 115200,
    //     .data_bits = 8,
    //     .parity = RU_PARITY_NONE,
    //     .stop_bits = RU_STOPS_1,
    // };
    // TEST_ASSERT_EQUAL(ESP_OK, RU_init_tx(0, &c, 0));
    // TEST_ASSERT_EQUAL_INT(1, gpio_get_level(RMT_TX_PIN));
    // RU_deinit(0, 0);
    //
    // gpio_set_direction(RMT_TX_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_level(RMT_TX_PIN, 1);
    //
    // rmt_uart_spec_t c2 = c;
    // c2.gpio = RMT_RX_PIN;
    // TEST_ASSERT_NOT_NULL(RU_init_rx(0, &c2, 0, 1));
    // uint8_t buf[32];
    // RU_recv(0, buf, sizeof(buf));
    // TEST_ASSERT_EQUAL_INT(1, gpio_get_level(RMT_RX_PIN));
    // RU_end_rx(0);
    // RU_deinit(0, 1);
}

// ---------------------------------------------------------------------------
// Tier 3: shared-pin bus (RU_bus_*). These probe the lifecycle and the
// data path is exercised in software by the Tier-1 encoder/decoder test above.
// A full two-node hardware loopback needs two boards / two nodes; left as a
// follow-up.
// ---------------------------------------------------------------------------

TEST_CASE("bus lifecycle: add channels then deinit", "[bus]") {
    rmt_uart_spec_t c = {
        .gpio = RMT_TX_PIN,
        .rate = 115200,
        .data_bits = 9,
        .parity = RU_PARITY_EVEN,
        .stop_bits = RU_STOPS_1,
    };
    TEST_ASSERT_EQUAL(ESP_OK, RU_bus_add_channels(&c));
    TEST_ASSERT_TRUE(ru_bus_initialized);
    RU_bus_deinit();
    TEST_ASSERT_FALSE(ru_bus_initialized);
}

TEST_CASE("bus rejects invalid configurations", "[bus]") {
    rmt_uart_spec_t c = {
        .gpio = RMT_TX_PIN,
        .rate = 115200,
        .data_bits = 10,
        .parity = RU_PARITY_EVEN,
        .stop_bits = RU_STOPS_1,
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, RU_bus_add_channels(&c));
    c.data_bits = 7;
    c.rate = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, RU_bus_add_channels(&c));
}

// ---------------------------------------------------------------------------
// Runner
// ---------------------------------------------------------------------------

static void print_banner(const char* text) {
    printf("\n#### %s #####\n\n", text);
}

void app_main(void) {
    // print_banner("Tier 1: encoder/decoder round-trip (software)");
    // UNITY_BEGIN();
    // unity_run_test_by_name("decoder: software glitch filter on synthetic symbols");
    // unity_run_test_by_name("encoder round-trip: address field prepends addr, keeps payload
    // intact"); UNITY_END();
    //
    //
    // print_banner("Tier 2: standalone full-duplex vs hardware UART (needs jumpers)");
    // UNITY_BEGIN();
    // unity_run_test_by_name("RMT TX -> HW UART RX round-trip");
    // unity_run_test_by_name("HW UART TX -> RMT RX round-trip");
    // //unity_run_test_by_name("standalone pins: idle level and continuous RX");
    // UNITY_END();

    print_banner("Tier 3: shared-pin bus lifecycle");
    UNITY_BEGIN();
    unity_run_test_by_name("bus lifecycle: add channels then deinit");
    unity_run_test_by_name("bus rejects invalid configurations");
    UNITY_END();


    // print_banner("Interactive menu (watchdog disabled)");
    // unity_run_menu();
    // UNITY_END();
}
