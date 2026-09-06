#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RADAR_UART UART_NUM_1
#define RADAR_RX_GPIO 0
#define RADAR_TX_GPIO 1
#define RADAR_BAUD 115200
#define MAX_DATA 64

static const char *TAG = "R60MON";

static volatile int g_presence = -1;
static volatile int g_breath = -1;
static volatile int g_hr = -1;
static volatile int g_in_bed = -1;
static volatile int g_sleep = -1;
static volatile unsigned g_frames = 0;
static volatile unsigned g_bad = 0;

typedef struct {
    int state;
    uint8_t ctrl;
    uint8_t cmd;
    uint16_t len;
    uint16_t idx;
    uint8_t sum;
    uint8_t data[MAX_DATA];
} parser_t;

static parser_t p = {0};

static void print_frame(uint8_t ctrl, uint8_t cmd, const uint8_t *data, uint16_t len)
{
    char hex[MAX_DATA * 3 + 1];
    size_t pos = 0;
    for (uint16_t i = 0; i < len && pos + 4 < sizeof(hex); i++) {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X%s", data[i], i + 1 < len ? " " : "");
    }
    hex[pos] = 0;
    ESP_LOGI(TAG, "RX ctrl=0x%02X cmd=0x%02X len=%u data=[%s]", ctrl, cmd, (unsigned)len, hex);
}

static void dispatch(uint8_t ctrl, uint8_t cmd, const uint8_t *data, uint16_t len)
{
    g_frames++;
    print_frame(ctrl, cmd, data, len);

    if (len >= 1) {
        if (ctrl == 0x80 && (cmd == 0x01 || cmd == 0x81)) {
            g_presence = data[0];
        } else if (ctrl == 0x81 && (cmd == 0x02 || cmd == 0x82)) {
            g_breath = data[0];
        } else if (ctrl == 0x85 && (cmd == 0x02 || cmd == 0x82)) {
            g_hr = data[0];
        } else if (ctrl == 0x84 && (cmd == 0x01 || cmd == 0x81)) {
            g_in_bed = data[0];
        } else if (ctrl == 0x84 && (cmd == 0x02 || cmd == 0x82)) {
            g_sleep = data[0];
        } else if (ctrl == 0x01 && (cmd == 0x01 || cmd == 0x80)) {
            ESP_LOGI(TAG, "HEARTBEAT reply received");
        } else if ((cmd == 0x80) && (ctrl == 0x80 || ctrl == 0x81 || ctrl == 0x84 || ctrl == 0x85)) {
            ESP_LOGI(TAG, "MONITOR_SWITCH ctrl=0x%02X state=%u", ctrl, data[0]);
        } else if (ctrl == 0x84 && cmd == 0x8C) {
            ESP_LOGI(TAG, "REPORT_MODE=%u", data[0]);
        }
    }

    if (ctrl == 0x02 && (cmd == 0x01 || cmd == 0xA1) && len > 0) {
        char s[48];
        size_t n = len < sizeof(s) - 1 ? len : sizeof(s) - 1;
        memcpy(s, data, n);
        s[n] = 0;
        ESP_LOGI(TAG, "PRODUCT=%s", s);
    }
}

static void feed(uint8_t b)
{
    switch (p.state) {
    case 0:
        if (b == 0x53) p.state = 1;
        break;
    case 1:
        if (b == 0x59) {
            p.sum = 0x53 + 0x59;
            p.state = 2;
        } else {
            p.state = 0;
        }
        break;
    case 2:
        p.ctrl = b;
        p.sum += b;
        p.state = 3;
        break;
    case 3:
        p.cmd = b;
        p.sum += b;
        p.state = 4;
        break;
    case 4:
        p.len = ((uint16_t)b) << 8;
        p.sum += b;
        p.state = 5;
        break;
    case 5:
        p.len |= b;
        p.sum += b;
        p.idx = 0;
        if (p.len > MAX_DATA) {
            g_bad++;
            p.state = 0;
        } else {
            p.state = 6;
        }
        break;
    case 6:
        if (p.idx < p.len) {
            p.data[p.idx++] = b;
            p.sum += b;
        } else {
            if (b == p.sum) p.state = 7;
            else {
                g_bad++;
                p.state = 0;
            }
        }
        break;
    case 7:
        if (b == 0x54) p.state = 8;
        else {
            g_bad++;
            p.state = 0;
        }
        break;
    case 8:
        if (b == 0x43) dispatch(p.ctrl, p.cmd, p.data, p.len);
        else g_bad++;
        p.state = 0;
        break;
    default:
        p.state = 0;
        break;
    }
}

static void send_query(uint8_t ctrl, uint8_t cmd)
{
    uint8_t f[10] = {0x53, 0x59, ctrl, cmd, 0x00, 0x01, 0x0F, 0, 0x54, 0x43};
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++) sum += f[i];
    f[7] = sum;
    uart_write_bytes(RADAR_UART, (const char *)f, sizeof(f));
    uart_wait_tx_done(RADAR_UART, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "TX query ctrl=0x%02X cmd=0x%02X", ctrl, cmd);
}

static void reader_task(void *arg)
{
    uint8_t buf[128];
    while (1) {
        int n = uart_read_bytes(RADAR_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) feed(buf[i]);
    }
}

static void poll_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(800));
    send_query(0x02, 0xA1);
    vTaskDelay(pdMS_TO_TICKS(150));
    send_query(0x80, 0x80);
    vTaskDelay(pdMS_TO_TICKS(150));
    send_query(0x81, 0x80);
    vTaskDelay(pdMS_TO_TICKS(150));
    send_query(0x84, 0x80);
    vTaskDelay(pdMS_TO_TICKS(150));
    send_query(0x85, 0x80);
    vTaskDelay(pdMS_TO_TICKS(150));
    send_query(0x84, 0x8C);

    while (1) {
        send_query(0x01, 0x80);
        vTaskDelay(pdMS_TO_TICKS(120));
        send_query(0x80, 0x81);
        vTaskDelay(pdMS_TO_TICKS(120));
        send_query(0x81, 0x82);
        vTaskDelay(pdMS_TO_TICKS(120));
        send_query(0x84, 0x81);
        vTaskDelay(pdMS_TO_TICKS(120));
        send_query(0x84, 0x82);
        vTaskDelay(pdMS_TO_TICKS(120));
        send_query(0x85, 0x82);
        vTaskDelay(pdMS_TO_TICKS(1400));
    }
}

static void summary_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "DATA presence=%d hr=%d bpm breath=%d rpm in_bed=%d sleep=%d frames=%u bad=%u",
                 g_presence, g_hr, g_breath, g_in_bed, g_sleep, g_frames, g_bad);
        if (g_frames == 0) {
            ESP_LOGW(TAG, "NO_VALID_RADAR_FRAME");
        }
    }
}

void app_main(void)
{
    uart_config_t cfg = {
        .baud_rate = RADAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RADAR_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RADAR_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RADAR_UART, RADAR_TX_GPIO, RADAR_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "R60ABD1 SERIAL MONITOR READY");
    ESP_LOGI(TAG, "UART1 115200 RX=GPIO0 TX=GPIO1; read-only queries only");

    xTaskCreate(reader_task, "radar_reader", 4096, NULL, 5, NULL);
    xTaskCreate(poll_task, "radar_poll", 4096, NULL, 4, NULL);
    xTaskCreate(summary_task, "summary", 3072, NULL, 3, NULL);
}
