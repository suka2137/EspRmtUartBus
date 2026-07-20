# Test subproject for the rmt-uart component.
#
# White-box: the test component #includes ../../src/rmt-uart.c and ../../src/ub.c
# directly so it can reach internal helpers (rmt_uart_encoder, rmt_uart_recv_cb_decode,
# rmt_uart_choose_freq, byte_parity, txs[], rxs[], duals[]). The component itself is
# therefore NOT pulled in via REQUIRES/TEST_COMPONENTS.
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rmt_uart_test)
