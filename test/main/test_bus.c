// This translation unit pulls in the bus (RU_bus_*) implementation so the test
// component links it alongside the rmt-uart core (compiled via test_rmt_uart.c).
// Kept as its own .c because ub.c and rmt-uart.c each declare a file-scope
// `static rmt_transmit_config_t TxConf`; including both in one TU would collide.
#include "../../src/ub.c"
