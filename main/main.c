#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/input.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/lcd_types.h"
#include "nvs_flash.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"
#include "pax_types.h"
#include "portmacro.h"
#include "usb_serial_bridge.h"

// Constants
static char const TAG[] = "main";

#if defined(CONFIG_BSP_TARGET_KAMI)
#define BLACK 0
#define WHITE 1
#define RED   2
#else
#define BLACK 0xFF000000
#define WHITE 0xFFFFFFFF
#define RED   0xFFFF0000
#endif

// Dashboard color palette (dark theme). Only used by ui_task(), which only
// ever runs once the USB<->UART bridge has actually started - in practice
// that means a real RGB framebuffer, never the KAMI/epaper palette display -
// so these are plain ARGB colors rather than BLACK/WHITE/RED palette indices.
#define COLOR_BG           0xFF12161D
#define COLOR_CARD_BG      0xFF1C222C
#define COLOR_CARD_BORDER  0xFF2A313C
#define COLOR_ACCENT       0xFF3FA7F5
#define COLOR_ACCENT_RX    0xFF36D399
#define COLOR_TEXT_PRIMARY 0xFFF2F4F8
#define COLOR_TEXT_MUTED   0xFF8A93A3
#define COLOR_TEXT_LABEL   0xFF6E7684
#define COLOR_GOOD         0xFF2ECC71
#define COLOR_BAD          0xFFE5493F
#define COLOR_BADGE_TEXT   0xFF0B1A12

#define DASHBOARD_FONT pax_font_saira_regular

#define UI_TASK_STACK_SIZE 4096
#define UI_TASK_PRIORITY   3
#define UI_UPDATE_PERIOD_MS 250

// Throughput history: one sample per second, so the graph's X axis always
// spans exactly one minute.
#define RATE_SAMPLE_PERIOD_MS 1000
#define RATE_HISTORY_LEN      60

// Global variables
static size_t                     display_h_res        = 0;
static size_t                     display_v_res        = 0;
static bsp_display_color_format_t display_color_format = 0;
static bsp_display_endianness_t   display_data_endian  = 0;
static pax_buf_t                  fb                   = {0};
static QueueHandle_t              input_event_queue    = NULL;

#if defined(CONFIG_BSP_TARGET_KAMI)
// Temporary addition for supporting epaper devices (irrelevant for Tanmatsu)
static pax_col_t palette[] = {0xffffffff, 0xff000000, 0xffff0000};  // white, black, red
#endif

// One throughput sample (bytes/second) per second, oldest first.
static float      tx_rate_history[RATE_HISTORY_LEN] = {0};
static float      rx_rate_history[RATE_HISTORY_LEN] = {0};
static uint64_t    rate_prev_tx_bytes                = 0;
static uint64_t    rate_prev_rx_bytes                = 0;
static TickType_t  rate_last_sample_tick             = 0;
static bool        rate_history_started              = false;

static void blit(void) {
    esp_err_t res = bsp_display_blit(0, 0, display_h_res, display_v_res, pax_buf_get_pixels(&fb));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to blit to display: %d", res);
    }
}

static void display_message(const char* message) {
    if (pax_buf_get_width(&fb) > 0) {
        pax_background(&fb, BLACK);
        pax_draw_text(&fb, WHITE, DASHBOARD_FONT, 18, 14, 14, message);
        blit();
    } else {
        ESP_LOGI(TAG, "Message: %s", message);
    }
}

// Draws a card background with a small uppercase label in its top-left corner.
// Returns the y coordinate at which the card's content rows should start.
static float draw_card(float x, float y, float w, float h, const char* title) {
    pax_draw_round_rect(&fb, COLOR_CARD_BG, x, y, w, h, 10);
    pax_outline_round_rect(&fb, COLOR_CARD_BORDER, x, y, w, h, 10);
    pax_draw_text(&fb, COLOR_TEXT_LABEL, DASHBOARD_FONT, 13, x + 16, y + 12, title);
    return y + 38;
}

// Draws a muted label on the left and a value on the right, both on one row.
static void draw_label_value(float x, float y, float w, const char* label, const char* value, pax_col_t value_color) {
    pax_draw_text(&fb, COLOR_TEXT_LABEL, DASHBOARD_FONT, 14, x, y, label);
    pax_draw_text_adv(&fb, value_color, DASHBOARD_FONT, 14, x + w, y, value, strlen(value), PAX_ALIGN_END,
                       PAX_ALIGN_BEGIN, -1);
}

// Draws a red/green status dot followed by a label, used for DTR/RTS.
static void draw_state_dot(float x, float y, const char* label, bool active) {
    pax_draw_circle(&fb, active ? COLOR_GOOD : COLOR_BAD, x + 7, y + 8, 7);
    pax_draw_text(&fb, COLOR_TEXT_PRIMARY, DASHBOARD_FONT, 15, x + 22, y, label);
}

// Draws a filled, centered pill-shaped badge - green when active, neutral otherwise.
static void draw_badge(float x, float y, float w, float h, const char* text, bool active) {
    pax_col_t bg = active ? COLOR_GOOD : COLOR_CARD_BORDER;
    pax_col_t fg = active ? COLOR_BADGE_TEXT : COLOR_TEXT_MUTED;
    pax_draw_round_rect(&fb, bg, x, y, w, h, h / 2);
    pax_vec2f size = pax_text_size(DASHBOARD_FONT, 14, text);
    pax_draw_text(&fb, fg, DASHBOARD_FONT, 14, x + (w - size.x) / 2, y + (h - size.y) / 2, text);
}

// Formats a byte count with a unit suffix, e.g. "842 B", "4.3 KB", "1.2 MB".
static void format_byte_count(char* buf, size_t buf_len, uint64_t bytes) {
    static const char* units[] = {"B", "KB", "MB", "GB"};
    double             value   = (double)bytes;
    size_t             unit    = 0;
    while (value >= 1024.0 && unit < 3) {
        value /= 1024.0;
        unit++;
    }
    if (unit == 0) {
        snprintf(buf, buf_len, "%" PRIu64 " B", bytes);
    } else {
        snprintf(buf, buf_len, "%.1f %s", value, units[unit]);
    }
}

// Formats a throughput value with a unit suffix, e.g. "842 B/s", "4.3 KB/s".
static void format_rate(char* buf, size_t buf_len, float bytes_per_sec) {
    char count[24];
    format_byte_count(count, sizeof(count), (uint64_t)(bytes_per_sec < 0 ? 0 : bytes_per_sec + 0.5f));
    snprintf(buf, buf_len, "%s/s", count);
}

// Draws TX (blue) and RX (green) throughput history as two overlaid line
// graphs sharing one auto-scaled Y axis. Both axes are labelled: the Y axis
// with auto-scaled rate values (0, 50% and 100% of the current peak, each
// with their unit), the X axis with elapsed time covering the full history
// (one minute, oldest on the left, "now" on the right).
static void draw_rate_graph(float x, float y, float w, float h, const float* tx_hist, const float* rx_hist,
                             size_t len) {
    const float y_label_w = 52;  // Reserved on the left for Y axis value labels
    const float x_label_h = 14;  // Reserved at the bottom for X axis time labels

    float chart_x = x + y_label_w;
    float chart_y = y;
    float chart_w = w - y_label_w;
    float chart_h = h - x_label_h;

    // Auto-scale the Y axis to the highest sample currently in view, with a
    // bit of headroom so peaks don't touch the top edge of the chart.
    float max_rate = 1.0f;
    for (size_t i = 0; i < len; i++) {
        if (tx_hist[i] > max_rate) max_rate = tx_hist[i];
        if (rx_hist[i] > max_rate) max_rate = rx_hist[i];
    }
    max_rate *= 1.15f;

    // Y axis: gridlines at 0%, 50% and 100% of scale, each labelled with its
    // value and unit.
    for (int i = 0; i <= 2; i++) {
        float frac = (float)i / 2.0f;
        float gy   = chart_y + chart_h * (1.0f - frac);
        pax_simple_rect(&fb, COLOR_CARD_BORDER, chart_x, gy, chart_w, 1);

        char label[16];
        format_rate(label, sizeof(label), max_rate * frac);
        pax_vec2f size = pax_text_size(DASHBOARD_FONT, 10, label);
        float     ly   = gy - size.y / 2;
        if (i == 0) ly = gy - size.y;        // Keep the "0" label above the baseline
        if (i == 2) ly = gy;                 // Keep the top label below the chart's top edge
        pax_draw_text(&fb, COLOR_TEXT_LABEL, DASHBOARD_FONT, 10, chart_x - size.x - 6, ly, label);
    }

    // TX/RX lines. pax_draw_thick_line() does not apply the display's
    // orientation transform in this PAX version, so a plain pax_draw_line()
    // is used instead (each one drawn twice, offset by one pixel, for a
    // slightly thicker, more visible line).
    for (size_t i = 1; i < len; i++) {
        float x0  = chart_x + (float)(i - 1) * chart_w / (float)(len - 1);
        float x1  = chart_x + (float)i * chart_w / (float)(len - 1);
        float ty0 = chart_y + chart_h - (tx_hist[i - 1] / max_rate) * chart_h;
        float ty1 = chart_y + chart_h - (tx_hist[i] / max_rate) * chart_h;
        pax_draw_line(&fb, COLOR_ACCENT, x0, ty0, x1, ty1);
        pax_draw_line(&fb, COLOR_ACCENT, x0, ty0 + 1, x1, ty1 + 1);
        float ry0 = chart_y + chart_h - (rx_hist[i - 1] / max_rate) * chart_h;
        float ry1 = chart_y + chart_h - (rx_hist[i] / max_rate) * chart_h;
        pax_draw_line(&fb, COLOR_ACCENT_RX, x0, ry0, x1, ry1);
        pax_draw_line(&fb, COLOR_ACCENT_RX, x0, ry0 + 1, x1, ry1 + 1);
    }

    // X axis: time labels at the start, middle and end of the one-minute window.
    char left_label[16];
    char mid_label[16];
    snprintf(left_label, sizeof(left_label), "-%ds", (int)(len - 1));
    snprintf(mid_label, sizeof(mid_label), "-%ds", (int)(len - 1) / 2);
    static const char right_label[] = "now";

    pax_draw_text(&fb, COLOR_TEXT_LABEL, DASHBOARD_FONT, 10, chart_x, chart_y + chart_h + 2, left_label);
    pax_vec2f mid_size = pax_text_size(DASHBOARD_FONT, 10, mid_label);
    pax_draw_text(&fb, COLOR_TEXT_LABEL, DASHBOARD_FONT, 10, chart_x + chart_w / 2 - mid_size.x / 2,
                  chart_y + chart_h + 2, mid_label);
    pax_vec2f right_size = pax_text_size(DASHBOARD_FONT, 10, right_label);
    pax_draw_text(&fb, COLOR_TEXT_LABEL, DASHBOARD_FONT, 10, chart_x + chart_w - right_size.x,
                  chart_y + chart_h + 2, right_label);
}

// Periodically redraws the serial port settings, control line state and byte
// counters as a small status dashboard. Runs at a lower priority than the
// UART<->CDC data path tasks so a (comparatively slow) display blit can
// never add latency to the byte forwarding.
static void ui_task(void* arg) {
    (void)arg;
    while (true) {
        float w = pax_buf_get_widthf(&fb);
        float h = pax_buf_get_heightf(&fb);
        if (w > 0 && h > 0) {
            usb_serial_bridge_line_info_t info     = {0};
            uint64_t                      tx_bytes = 0;
            uint64_t                      rx_bytes = 0;
            usb_serial_bridge_get_status(&info, &tx_bytes, &rx_bytes);

            // Sample the throughput history once per second, independent of
            // this task's (faster) redraw period.
            TickType_t now = xTaskGetTickCount();
            if (!rate_history_started || now - rate_last_sample_tick >= pdMS_TO_TICKS(RATE_SAMPLE_PERIOD_MS)) {
                uint64_t delta_tx = rate_history_started ? (tx_bytes - rate_prev_tx_bytes) : 0;
                uint64_t delta_rx = rate_history_started ? (rx_bytes - rate_prev_rx_bytes) : 0;
                rate_prev_tx_bytes = tx_bytes;
                rate_prev_rx_bytes = rx_bytes;

                memmove(tx_rate_history, tx_rate_history + 1, sizeof(tx_rate_history) - sizeof(tx_rate_history[0]));
                memmove(rx_rate_history, rx_rate_history + 1, sizeof(rx_rate_history) - sizeof(rx_rate_history[0]));
                tx_rate_history[RATE_HISTORY_LEN - 1] = (float)delta_tx * 1000.0f / RATE_SAMPLE_PERIOD_MS;
                rx_rate_history[RATE_HISTORY_LEN - 1] = (float)delta_rx * 1000.0f / RATE_SAMPLE_PERIOD_MS;

                rate_last_sample_tick = now;
                rate_history_started  = true;
            }

            char text[72];

            pax_background(&fb, COLOR_BG);

            // Header
            const float margin = 14;
            pax_draw_text(&fb, COLOR_TEXT_PRIMARY, DASHBOARD_FONT, 22, margin, margin, "USB Serial Bridge");
            static const char subtitle[] = "CATT D5 -> TX   ·   CATT D1 -> RX";
            pax_draw_text_adv(&fb, COLOR_TEXT_MUTED, DASHBOARD_FONT, 13, w - margin, margin + 7, subtitle,
                               strlen(subtitle), PAX_ALIGN_END, PAX_ALIGN_BEGIN, -1);
            const float header_h = 34;
            pax_simple_rect(&fb, COLOR_ACCENT, margin, margin + header_h, w - 2 * margin, 2);

            // Layout: two cards side by side, then a full-width card below.
            const float gap        = 12;
            const float footer_h   = 20;
            const float content_y  = margin + header_h + 14;
            const float content_h  = h - content_y - footer_h - margin;
            const float row1_h     = content_h * 0.44f;
            const float row2_y     = content_y + row1_h + gap;
            const float row2_h     = content_h - row1_h - gap;
            const float card_w     = (w - 2 * margin - gap) / 2;

            // --- Port settings card ---
            float px = margin;
            float py = draw_card(px, content_y, card_w, row1_h, "PORT SETTINGS");
            snprintf(text, sizeof(text), "%" PRIu32, info.baud_rate);
            draw_label_value(px + 16, py, card_w - 32, "Baud rate", text, COLOR_TEXT_PRIMARY);
            py += 26;
            snprintf(text, sizeof(text), "%u%c%s", info.data_bits, info.parity, info.stop_bits);
            draw_label_value(px + 16, py, card_w - 32, "Frame format", text, COLOR_TEXT_PRIMARY);
            py += 32;
            draw_label_value(px + 16, py, card_w - 32, "TX pin", "CATT D5", COLOR_TEXT_MUTED);
            py += 22;
            draw_label_value(px + 16, py, card_w - 32, "RX pin", "CATT D1", COLOR_TEXT_MUTED);

            // --- Control lines card ---
            float cx = px + card_w + gap;
            float cy = draw_card(cx, content_y, card_w, row1_h, "CONTROL LINES");
            draw_state_dot(cx + 16, cy, info.dtr ? "DTR asserted" : "DTR low", info.dtr);
            cy += 28;
            draw_state_dot(cx + 16, cy, info.rts ? "RTS asserted" : "RTS low", info.rts);
            cy += 34;
            draw_badge(cx + 16, cy, card_w - 32, 26, info.host_connected ? "HOST CONNECTED" : "HOST WAITING",
                       info.host_connected);

            // --- Data transfer card ---
            float dy = draw_card(margin, row2_y, w - 2 * margin, row2_h, "DATA TRANSFER  ·  LAST 60S");

            char rate_str[24];
            char total_str[24];
            format_rate(rate_str, sizeof(rate_str), tx_rate_history[RATE_HISTORY_LEN - 1]);
            format_byte_count(total_str, sizeof(total_str), tx_bytes);
            snprintf(text, sizeof(text), "TX  %s  ·  %s total", rate_str, total_str);
            pax_draw_text(&fb, COLOR_ACCENT, DASHBOARD_FONT, 14, margin + 16, dy, text);

            format_rate(rate_str, sizeof(rate_str), rx_rate_history[RATE_HISTORY_LEN - 1]);
            format_byte_count(total_str, sizeof(total_str), rx_bytes);
            snprintf(text, sizeof(text), "RX  %s  ·  %s total", rate_str, total_str);
            pax_draw_text_adv(&fb, COLOR_ACCENT_RX, DASHBOARD_FONT, 14, w - margin - 16, dy, text, strlen(text),
                               PAX_ALIGN_END, PAX_ALIGN_BEGIN, -1);

            float graph_y = dy + 26;
            float graph_h = row2_y + row2_h - graph_y - 8;
            draw_rate_graph(margin + 16, graph_y, w - 2 * margin - 32, graph_h, tx_rate_history, rx_rate_history,
                             RATE_HISTORY_LEN);

            // Footer
            static const char footer[] = "Press ESC or F1 to return to the launcher";
            pax_draw_text_adv(&fb, COLOR_TEXT_MUTED, DASHBOARD_FONT, 13, w / 2, h - margin - 14, footer,
                               strlen(footer), PAX_ALIGN_CENTER, PAX_ALIGN_BEGIN, -1);

            blit();
        }
        vTaskDelay(pdMS_TO_TICKS(UI_UPDATE_PERIOD_MS));
    }
}

void app_main(void) {
    // Start the GPIO interrupt service
    gpio_install_isr_service(0);

    // Initialize the Non Volatile Storage partition
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        res = nvs_flash_erase();
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS flash: %d", res);
            return;
        }
        res = nvs_flash_init();
    }
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %d", res);
        return;
    }

    // Initialize the Board Support Package
    const bsp_configuration_t bsp_configuration = {
        .display =
            {
                .requested_color_format = BSP_DISPLAY_COLOR_FORMAT_24_888RGB,
                .num_fbs                = 1,
            },
    };
    res = bsp_device_initialize(&bsp_configuration);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BSP: %d", res);
        return;
    }

    // Get display parameters and rotation
    res = bsp_display_get_parameters(&display_h_res, &display_v_res, &display_color_format, &display_data_endian);
    if (res != ESP_ERR_NOT_SUPPORTED) {
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get display parameters: %d", res);
            return;
        }

        // Convert ESP-IDF color format into PAX buffer type
        pax_buf_type_t format = PAX_BUF_24_888RGB;
        switch (display_color_format) {
            case BSP_DISPLAY_COLOR_FORMAT_1_PAL:
                format = PAX_BUF_1_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_2_PAL:
                format = PAX_BUF_2_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_4_PAL:
                format = PAX_BUF_4_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_8_PAL:
                format = PAX_BUF_8_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_16_PAL:
                format = PAX_BUF_16_PAL;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_1_GREY:
                format = PAX_BUF_1_GREY;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_2_GREY:
                format = PAX_BUF_2_GREY;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_4_GREY:
                format = PAX_BUF_4_GREY;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_8_GREY:
                format = PAX_BUF_8_GREY;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_8_332RGB:
                format = PAX_BUF_8_332RGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_16_565RGB:
                format = PAX_BUF_16_565RGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_4_1111ARGB:
                format = PAX_BUF_4_1111ARGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_8_2222ARGB:
                format = PAX_BUF_8_2222ARGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_16_4444ARGB:
                format = PAX_BUF_16_4444ARGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_24_888RGB:
                format = PAX_BUF_24_888RGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_32_8888ARGB:
                format = PAX_BUF_32_8888ARGB;
                break;
            case BSP_DISPLAY_COLOR_FORMAT_18_666RGB:
            default:
                ESP_LOGW(TAG, "BSP requests color format not supported by PAX (%u)", format);
                break;
        }

        // Convert BSP display rotation format into PAX orientation type
        bsp_display_rotation_t display_rotation = bsp_display_get_default_rotation();
        pax_orientation_t      orientation      = PAX_O_UPRIGHT;
        switch (display_rotation) {
            case BSP_DISPLAY_ROTATION_90:
                orientation = PAX_O_ROT_CCW;
                break;
            case BSP_DISPLAY_ROTATION_180:
                orientation = PAX_O_ROT_HALF;
                break;
            case BSP_DISPLAY_ROTATION_270:
                orientation = PAX_O_ROT_CW;
                break;
            case BSP_DISPLAY_ROTATION_0:
            default:
                orientation = PAX_O_UPRIGHT;
                break;
        }

        // Initialize graphics stack
        printf("Initializing framebuffer with w=%d h=%d format=%d endian=%d orientation=%d\n", display_h_res,
               display_v_res, format, display_data_endian, orientation);
        pax_buf_init(&fb, NULL, display_h_res, display_v_res, format);
        pax_buf_reversed(&fb, display_data_endian == BSP_DISPLAY_ENDIAN_BIG);
#if defined(CONFIG_BSP_TARGET_KAMI)
        // Temporary addition for supporting Kami
        fb.palette      = palette;
        fb.palette_size = sizeof(palette) / sizeof(pax_col_t);
#endif
        pax_buf_set_orientation(&fb, orientation);
    } else {
        ESP_LOGI(TAG, "This board has no display support");
    }

    // Get input event queue from BSP
    ESP_ERROR_CHECK(bsp_input_get_queue(&input_event_queue));

    display_message("Starting USB CDC <-> UART bridge...");

    res = usb_serial_bridge_start();
    if (res == ESP_OK) {
        xTaskCreate(ui_task, "ui", UI_TASK_STACK_SIZE, NULL, UI_TASK_PRIORITY, NULL);
    } else {
        ESP_LOGE(TAG, "USB serial bridge unavailable: %d", res);
        display_message("USB serial bridge unavailable on this board\nPress ESC or F1 to return to the launcher");
    }

    // Main section of the app: wait for input events, acting only on the
    // "return to launcher" keys (ESC and F1). The serial bridge and the
    // on-screen status run independently in their own tasks.
    while (1) {
        bsp_input_event_t event;
        if (xQueueReceive(input_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            if (event.type == INPUT_EVENT_TYPE_NAVIGATION &&
                (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_F1 ||
                 event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_ESC)) {
                bsp_device_restart_to_launcher();
            }
        }
    }
}
