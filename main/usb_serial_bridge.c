#include "usb_serial_bridge.h"

#include <string.h>
#include "sdkconfig.h"

// TinyUSB CDC-ACM is only wired up (via esp_tinyusb) on targets with a USB-OTG
// peripheral that shares Tanmatsu's USB 1.1 PHY handling. Other targets get a
// stub below so the app still builds for them.
#if defined(CONFIG_IDF_TARGET_ESP32P4) || defined(CONFIG_IDF_TARGET_ESP32S3)
#define USB_SERIAL_BRIDGE_SUPPORTED 1
#else
#define USB_SERIAL_BRIDGE_SUPPORTED 0
#endif

#if USB_SERIAL_BRIDGE_SUPPORTED

#include <stdatomic.h>
#include <stdio.h>
#include "bsp/catt.h"
#include "bsp/device.h"
#include "bsp/usb_device.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

static const char TAG[] = "usb-serial";

#define BRIDGE_UART_NUM        UART_NUM_1
#define BRIDGE_UART_RX_BUFSIZE 2048
#define BRIDGE_UART_TX_BUFSIZE 2048
#define BRIDGE_UART_QUEUE_LEN  16
#define BRIDGE_CDC_ITF         TINYUSB_CDC_ACM_0

// USB device identification. VID/PID are TinyUSB's own placeholder test IDs
// (0xCafe / CDC-only PID per TinyUSB's examples) until real ones are assigned.
#define USB_DEVICE_TEST_VID       0xCafe
#define USB_DEVICE_TEST_PID       0x4001
#define USB_DEVICE_PRODUCT_STRING "USB serial converter"
#define USB_DEVICE_STRING_LENGTH  32

enum {
    USB_STRID_LANGID = 0,
    USB_STRID_MANUFACTURER,
    USB_STRID_PRODUCT,
    USB_STRID_SERIAL,
    USB_STRID_CDC_INTERFACE,
};

static char s_usb_manufacturer[USB_DEVICE_STRING_LENGTH];
static char s_usb_serial[USB_DEVICE_STRING_LENGTH];

static const char* s_usb_str_desc[] = {
    [USB_STRID_LANGID]        = (char[]){0x09, 0x04},  // English (0x0409)
    [USB_STRID_MANUFACTURER]  = s_usb_manufacturer,
    [USB_STRID_PRODUCT]       = USB_DEVICE_PRODUCT_STRING,
    [USB_STRID_SERIAL]        = s_usb_serial,
    [USB_STRID_CDC_INTERFACE] = "CDC Interface",
};

static const tusb_desc_device_t s_usb_desc_device = {
    .bLength         = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB          = 0x0200,
    // Interface Association Descriptor is used for CDC, so per USB spec the
    // device class/subclass/protocol must be set this way.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_DEVICE_TEST_VID,
    .idProduct          = USB_DEVICE_TEST_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = USB_STRID_MANUFACTURER,
    .iProduct           = USB_STRID_PRODUCT,
    .iSerialNumber      = USB_STRID_SERIAL,
    .bNumConfigurations = 0x01,
};
#define BRIDGE_TASK_STACK_SIZE 4096
#define BRIDGE_TASK_PRIORITY   10

static QueueHandle_t s_uart_event_queue = NULL;

// Incremented only from the data-path task/callback; read from the (lower
// priority) UI task via usb_serial_bridge_get_status(). Lock-free so the
// data path never waits on the UI.
static _Atomic uint64_t s_tx_bytes = 0;  // host -> UART
static _Atomic uint64_t s_rx_bytes = 0;  // UART -> host

static portMUX_TYPE                  s_line_lock = portMUX_INITIALIZER_UNLOCKED;
static usb_serial_bridge_line_info_t s_line_info = {
    .baud_rate      = 115200,
    .data_bits      = 8,
    .parity         = 'N',
    .stop_bits      = "1",
    .dtr            = false,
    .rts            = false,
    .host_connected = false,
};

static void apply_uart_line_coding(const cdc_line_coding_t* coding) {
    uart_word_length_t data_bits         = UART_DATA_8_BITS;
    uint8_t             data_bits_display = 8;
    switch (coding->data_bits) {
        case 5:
            data_bits         = UART_DATA_5_BITS;
            data_bits_display = 5;
            break;
        case 6:
            data_bits         = UART_DATA_6_BITS;
            data_bits_display = 6;
            break;
        case 7:
            data_bits         = UART_DATA_7_BITS;
            data_bits_display = 7;
            break;
        default:
            data_bits         = UART_DATA_8_BITS;
            data_bits_display = 8;
            break;
    }

    uart_parity_t parity         = UART_PARITY_DISABLE;
    char          parity_display = 'N';
    switch (coding->parity) {
        case 1:
            parity         = UART_PARITY_ODD;
            parity_display = 'O';
            break;
        case 2:
            parity         = UART_PARITY_EVEN;
            parity_display = 'E';
            break;
        default:
            // Hardware UART has no mark/space parity; fall back to none.
            parity         = UART_PARITY_DISABLE;
            parity_display = 'N';
            break;
    }

    uart_stop_bits_t stop_bits         = UART_STOP_BITS_1;
    const char*      stop_bits_display = "1";
    switch (coding->stop_bits) {
        case 1:
            stop_bits         = UART_STOP_BITS_1_5;
            stop_bits_display = "1.5";
            break;
        case 2:
            stop_bits         = UART_STOP_BITS_2;
            stop_bits_display = "2";
            break;
        default:
            stop_bits         = UART_STOP_BITS_1;
            stop_bits_display = "1";
            break;
    }

    uint32_t baud_rate = coding->bit_rate > 0 ? coding->bit_rate : 115200;

    const uart_config_t uart_config = {
        .baud_rate  = (int)baud_rate,
        .data_bits  = data_bits,
        .parity     = parity,
        .stop_bits  = stop_bits,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t res = uart_param_config(BRIDGE_UART_NUM, &uart_config);
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "Failed to apply UART line coding: %d", res);
    }

    portENTER_CRITICAL(&s_line_lock);
    s_line_info.baud_rate = baud_rate;
    s_line_info.data_bits = data_bits_display;
    s_line_info.parity    = parity_display;
    s_line_info.stop_bits = stop_bits_display;
    portEXIT_CRITICAL(&s_line_lock);
}

static void cdc_line_coding_changed_cb(int itf, cdcacm_event_t* event) {
    if (itf != BRIDGE_CDC_ITF || event->line_coding_changed_data.p_line_coding == NULL) {
        return;
    }
    apply_uart_line_coding(event->line_coding_changed_data.p_line_coding);
}

static void cdc_line_state_changed_cb(int itf, cdcacm_event_t* event) {
    if (itf != BRIDGE_CDC_ITF) {
        return;
    }
    portENTER_CRITICAL(&s_line_lock);
    s_line_info.dtr            = event->line_state_changed_data.dtr;
    s_line_info.rts            = event->line_state_changed_data.rts;
    s_line_info.host_connected = event->line_state_changed_data.dtr || event->line_state_changed_data.rts;
    portEXIT_CRITICAL(&s_line_lock);
}

// Runs in the TinyUSB task context: pull the bytes the host just sent out of
// the CDC RX buffer and hand them straight to the UART driver's own TX ring
// buffer. No display/UI work happens here.
static void cdc_rx_cb(int itf, cdcacm_event_t* event) {
    (void)event;
    if (itf != BRIDGE_CDC_ITF) {
        return;
    }
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t  rx_size = 0;
    if (tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size) == ESP_OK && rx_size > 0) {
        uart_write_bytes(BRIDGE_UART_NUM, (const char*)buf, rx_size);
        atomic_fetch_add(&s_tx_bytes, (uint64_t)rx_size);
    }
}

// Dedicated, high-priority task: waits on UART RX events and forwards bytes
// straight into the CDC TX buffer. Kept independent from the UI task so a
// slow display refresh can never add latency here.
static void uart_rx_task(void* arg) {
    (void)arg;
    uint8_t      buf[256];
    uart_event_t event;
    while (true) {
        if (xQueueReceive(s_uart_event_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (event.type) {
            case UART_DATA: {
                size_t available = 0;
                uart_get_buffered_data_len(BRIDGE_UART_NUM, &available);
                while (available > 0) {
                    int len = uart_read_bytes(BRIDGE_UART_NUM, buf, MIN(available, sizeof(buf)), 0);
                    if (len <= 0) {
                        break;
                    }
                    tinyusb_cdcacm_write_queue(BRIDGE_CDC_ITF, buf, (size_t)len);
                    tinyusb_cdcacm_write_flush(BRIDGE_CDC_ITF, 0);
                    atomic_fetch_add(&s_rx_bytes, (uint64_t)len);
                    available -= (size_t)len;
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART RX overrun, flushing");
                uart_flush_input(BRIDGE_UART_NUM);
                xQueueReset(s_uart_event_queue);
                break;
            default:
                break;
        }
    }
}

esp_err_t usb_serial_bridge_start(void) {
    gpio_num_t tx_pin = bsp_catt_get_gpio(CATT_PIN_D5);
    gpio_num_t rx_pin = bsp_catt_get_gpio(CATT_PIN_D1);
    if (tx_pin == GPIO_NUM_NC || rx_pin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "CATT D5/D1 pins are not available on this board");
        return ESP_ERR_NOT_SUPPORTED;
    }

    bsp_device_get_manufacturer(s_usb_manufacturer, sizeof(s_usb_manufacturer));

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(s_usb_serial, sizeof(s_usb_serial), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3],
             mac[4], mac[5]);

    // Bring up the TinyUSB stack on the shared USB 1.1 PHY (OTG1.1) first,
    // while the PHY mux is still pointed at the debug/JTAG controller, so
    // the device is fully configured before it appears on the bus.
    tinyusb_config_t tusb_cfg        = TINYUSB_CONFIG_FULL_SPEED(NULL, NULL);
    tusb_cfg.descriptor.device       = &s_usb_desc_device;
    tusb_cfg.descriptor.string       = s_usb_str_desc;
    tusb_cfg.descriptor.string_count = sizeof(s_usb_str_desc) / sizeof(s_usb_str_desc[0]);
    esp_err_t res                    = tinyusb_driver_install(&tusb_cfg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TinyUSB driver: %d", res);
        return res;
    }

    const tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port                     = BRIDGE_CDC_ITF,
        .callback_rx                  = cdc_rx_cb,
        .callback_rx_wanted_char      = NULL,
        .callback_line_state_changed  = cdc_line_state_changed_cb,
        .callback_line_coding_changed = cdc_line_coding_changed_cb,
    };
    res = tinyusb_cdcacm_init(&acm_cfg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CDC-ACM interface: %d", res);
        return res;
    }

    const uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    res = uart_driver_install(BRIDGE_UART_NUM, BRIDGE_UART_RX_BUFSIZE, BRIDGE_UART_TX_BUFSIZE, BRIDGE_UART_QUEUE_LEN,
                               &s_uart_event_queue, 0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %d", res);
        return res;
    }
    uart_param_config(BRIDGE_UART_NUM, &uart_config);
    res = uart_set_pin(BRIDGE_UART_NUM, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to assign UART pins: %d", res);
        return res;
    }

    if (xTaskCreate(uart_rx_task, "uart_rx", BRIDGE_TASK_STACK_SIZE, NULL, BRIDGE_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART RX task");
        return ESP_ERR_NO_MEM;
    }

    // Finally, connect the PHY to the OTG1.1 controller and onto the bus.
    res = bsp_usb_device_mode_set(BSP_USB_MODE_DEVICE);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch USB PHY to device mode: %d", res);
        return res;
    }

    ESP_LOGI(TAG, "USB CDC <-> UART bridge ready (TX=GPIO%d, RX=GPIO%d)", tx_pin, rx_pin);
    return ESP_OK;
}

void usb_serial_bridge_get_status(usb_serial_bridge_line_info_t* out_info, uint64_t* out_tx_bytes,
                                   uint64_t* out_rx_bytes) {
    if (out_info) {
        portENTER_CRITICAL(&s_line_lock);
        *out_info = s_line_info;
        portEXIT_CRITICAL(&s_line_lock);
    }
    if (out_tx_bytes) {
        *out_tx_bytes = atomic_load(&s_tx_bytes);
    }
    if (out_rx_bytes) {
        *out_rx_bytes = atomic_load(&s_rx_bytes);
    }
}

#else  // !USB_SERIAL_BRIDGE_SUPPORTED

esp_err_t usb_serial_bridge_start(void) {
    return ESP_ERR_NOT_SUPPORTED;
}

void usb_serial_bridge_get_status(usb_serial_bridge_line_info_t* out_info, uint64_t* out_tx_bytes,
                                   uint64_t* out_rx_bytes) {
    if (out_info) {
        memset(out_info, 0, sizeof(*out_info));
    }
    if (out_tx_bytes) {
        *out_tx_bytes = 0;
    }
    if (out_rx_bytes) {
        *out_rx_bytes = 0;
    }
}

#endif  // USB_SERIAL_BRIDGE_SUPPORTED
