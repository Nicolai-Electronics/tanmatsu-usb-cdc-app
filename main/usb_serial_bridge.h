#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Snapshot of the UART line coding/control state, as last negotiated by the USB host.
typedef struct {
    uint32_t    baud_rate;
    uint8_t     data_bits;   // 5-8
    char        parity;      // 'N', 'O' or 'E'
    const char* stop_bits;   // "1", "1.5" or "2"
    bool        dtr;
    bool        rts;
    bool        host_connected;
} usb_serial_bridge_line_info_t;

// Switches the USB 1.1 PHY to device mode, brings up a TinyUSB CDC-ACM
// interface and a UART on the CATT D5 (TX) / D1 (RX) pins, and starts
// forwarding bytes between them.
esp_err_t usb_serial_bridge_start(void);

// Reads the current line coding/control state and byte counters. Safe to
// call from any task; never blocks on the bridge's data path.
void usb_serial_bridge_get_status(usb_serial_bridge_line_info_t* out_info, uint64_t* out_tx_bytes,
                                   uint64_t* out_rx_bytes);

#ifdef __cplusplus
}
#endif
