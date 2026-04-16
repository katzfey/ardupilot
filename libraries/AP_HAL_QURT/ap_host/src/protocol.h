
#include <stdint.h>

#pragma once

#define QURT_MSG_ID_MAVLINK_MSG 1
#define QURT_MSG_ID_REBOOT 2
#define QURT_MSG_ID_PRINTF 3
#define QURT_MSG_ID_UART_CONFIG 4
#define QURT_MSG_ID_UART_DATA 5

#define MAX_MAVLINK_INSTANCES 2

struct __attribute__((__packed__)) qurt_rpc_msg {
    uint8_t msg_id;
    uint8_t inst;
    uint16_t data_length;
    uint32_t seq;
    uint8_t data[300];
};

#define QURT_RPC_MSG_HEADER_LEN 8

// Payload for QURT_MSG_ID_UART_CONFIG. device_id maps directly to the
// Linux device path on the apps processor as "/dev/ttyHS<device_id>".
struct __attribute__((__packed__)) qurt_uart_config {
    uint32_t baudrate;
    uint32_t device_id;
};
