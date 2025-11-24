#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"

// Set the affinity to the core that will initialize RMT and/or run the RMT UART
// Threading is not supported yet - use the API only from that core
#define RMT_UART_AFFINITY CONFIG_ESP_MAIN_TASK_AFFINITY

// Add a non-cumulative rounding value to each run length instead of loosening the basePeriod
// This can work well when most of the error comes from edge transitions (on by default)
#define RMT_UART_COMPENSATE_EDGE 1
// 0: Disabled (debug)  1: Hardware  2: Software filtering (more aggressive, default)  3: Both
#define RMT_UART_GLITCH_FILTER 2
// Do not generate encoder lookup tables
#define RMT_UART_NO_ENC_TABLE 1

// TX and RX can have an id from 0 to 7, optionally the same one
typedef int rmt_uart_id;

typedef enum UB_RX_STAT {
    RU_RECV_SUCCESS,       // Empty queue indicates readiness to receive
    RU_RECV_PART,          // Sent if chunks are not accumulated.
    RU_RECV_MALFORMED,     // Failed to decode. Data integrity issues will also cause this
    RU_RECV_FAULT,         // Not caused directly by wrong data - Buf/queue full or not cleared etc
    RU_RECV_DROP_RECOVER,  // Internal
    RU_RECV_DEINIT,        // Sent on deinit
} rmt_uart_rx_result_t;

typedef struct UB_RxEvent {
    rmt_uart_rx_result_t result;
    size_t size;
    uint8_t* data;
} rmt_uart_recv_notify_t;

enum RMT_UART_PARITY {
    RU_PARITY_NONE = 0,
    RU_PARITY_EVEN = 1,
    RU_PARITY_ODD = 2,
    RU_PARITY_MARK = 3,
    RU_PARITY_SPACE = 4,
};

enum RMT_UART_STOPS {
    RU_STOPS_1 = 1,
    RU_STOPS_1_5 = 15,
    RU_STOPS_2 = 2,
};


typedef struct RMT_UART_SPEC {
    uint32_t gpio;
    uint32_t rate;

    // 4-9 data bits. Transmitting 9 will mark the 9th bit of the first byte.
    // For < 8, only the LSBs are used (the rest must be 0s)
    int data_bits;

    // use enum RU_STOPS_x: 1, 1_5, 2
    enum RMT_UART_STOPS stop_bits;

    // use enum RU_PARITY_x: NONE, EVEN, ODD, MARK, SPACE
    enum RMT_UART_PARITY parity;
} rmt_uart_spec_t;


esp_err_t RU_init_tx(rmt_uart_id new_id, rmt_uart_spec_t* conf, uint8_t notify_index);

// Nonblocking - xTaskToNotify can be NULL, the caller or anything else.
// payload must exist until then.
esp_err_t RU_transmit(rmt_uart_id id, uint8_t* payload, size_t size, TaskHandle_t xTaskToNotify);

bool RU_tx_get_ongoing(const rmt_uart_id id);


/* Returns a 1-item queue of rmt_uart_recv_notify_t. Emptiness states being ready to receive.
 * continous: keep receiving. Use xQueuePeek() and CONFIG_RMT_RECV_FUNC_IN_IRAM .
 * defer=-1: Partial recv in ISR, start pushing parts before completion (Not for ESP32).
 * defer=0: Decoding in ISR with partial receiving if available.
 * Other value: Symbol buffer size in bytes for decoding in a task,
 * 5 RMT words (20 bytes) per byte (8N1).
 */
QueueHandle_t RU_init_rx(rmt_uart_id new_id, rmt_uart_spec_t* conf, int defer, bool continuous);

// Begins receiving and returns the same queue as RU_init_rx.
QueueHandle_t RU_recv(rmt_uart_id id, uint8_t* buffer, size_t bufferSize);

bool RU_rx_get_ongoing(const rmt_uart_id id);

// stop an ongoing receive
void RU_end_rx(rmt_uart_id id);


// Deinitialize TXs and RXs having the id. Receive queue will stop being valid.
// wait: send RU_LINE_DEINIT and wait 10ms
esp_err_t RU_deinit(rmt_uart_id id, bool wait);


// Remove all RMTs and release the APB lock if using PM
// void RU_disable_all();

// Restore RMTs and acquire APB PM lock if needed
// void RU_resume_all();

// total deinit.
// run RMT_UART_UB_end() first if applicable.
void RMT_UART_end();
