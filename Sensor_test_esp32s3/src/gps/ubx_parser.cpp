#include "ubx_parser.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "UBX";

static uart_port_t s_uart = UART_NUM_1;
static bool s_inited = false;

// UBX sync bytes
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

// UBX message classes/IDs
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07

// NAV-PVT payload size
#define NAV_PVT_LEN 92

// Parser state
enum { WAIT_SYNC1, WAIT_SYNC2, WAIT_CLASS, WAIT_ID, WAIT_LEN1, WAIT_LEN2, WAIT_PAYLOAD, WAIT_CKA, WAIT_CKB };

static uint8_t s_state = WAIT_SYNC1;
static uint8_t s_class = 0;
static uint8_t s_id = 0;
static uint16_t s_len = 0;
static uint16_t s_idx = 0;
static uint8_t s_cka = 0;
static uint8_t s_ckb = 0;
static uint8_t s_payload[NAV_PVT_LEN];

static void reset_parser(void)
{
    s_state = WAIT_SYNC1;
    s_class = 0;
    s_id = 0;
    s_len = 0;
    s_idx = 0;
    s_cka = 0;
    s_ckb = 0;
}

static void update_checksum(uint8_t b)
{
    s_cka += b;
    s_ckb += s_cka;
}

static bool parse_nav_pvt(const uint8_t *buf, ubx_nav_pvt_t *out)
{
    memset(out, 0, sizeof(*out));

    out->itow = *(uint32_t *)&buf[0];
    out->year = *(uint16_t *)&buf[4];
    out->month = buf[6];
    out->day = buf[7];
    out->hour = buf[8];
    out->min = buf[9];
    out->sec = buf[10];
    out->valid = buf[11];
    out->tacc = *(uint32_t *)&buf[12];
    out->nano = *(int32_t *)&buf[16];
    out->fix_type = buf[20];
    out->flags = buf[21];
    out->flags2 = buf[22];
    out->num_sv = buf[23];
    out->lon = *(int32_t *)&buf[24];
    out->lat = *(int32_t *)&buf[28];
    out->height = *(int32_t *)&buf[32];
    out->hmsl = *(int32_t *)&buf[36];
    out->hacc = *(uint32_t *)&buf[40];
    out->vacc = *(uint32_t *)&buf[44];
    out->vel_n = *(int32_t *)&buf[48];
    out->vel_e = *(int32_t *)&buf[52];
    out->vel_d = *(int32_t *)&buf[56];
    out->gspeed = *(int32_t *)&buf[60];
    out->heading = *(int32_t *)&buf[64];
    out->sacc = *(uint32_t *)&buf[68];
    out->headacc = *(uint32_t *)&buf[72];
    out->pdop = *(uint16_t *)&buf[76];
    out->flags3 = buf[78];
    // bytes 79-83 reserved
    out->head_veh = *(int32_t *)&buf[84];
    out->mag_dec = *(int16_t *)&buf[88];
    out->mag_acc = *(uint16_t *)&buf[90];

    // Compute human-readable values
    out->lat_deg = out->lat * 1e-7;
    out->lon_deg = out->lon * 1e-7;
    out->alt_m = out->hmsl / 1000.0f;
    out->speed_kmh = out->gspeed * 0.0036f; // mm/s to km/h
    out->heading_deg = out->heading * 1e-5f;
    out->valid_fix = (out->fix_type >= 2) && (out->flags & 0x01);
    out->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    return true;
}

esp_err_t ubx_init(const ubx_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_inited) return ESP_OK;

    s_uart = cfg->uart_num;

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = cfg->baud_rate;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(s_uart, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(s_uart, cfg->tx_pin, cfg->rx_pin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(s_uart, 1024, 0, 0, nullptr, 0));

    reset_parser();
    s_inited = true;
    ESP_LOGI(TAG, "UBX parser ready (UART%d, %d bps)", s_uart, cfg->baud_rate);
    return ESP_OK;
}

bool ubx_read(ubx_nav_pvt_t *out, uint32_t timeout_ms)
{
    if (!s_inited || !out) return false;

    uint8_t buf[64];
    int64_t start = esp_timer_get_time();
    int64_t deadline = start + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        int len = uart_read_bytes(s_uart, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            uint8_t b = buf[i];

            switch (s_state) {
            case WAIT_SYNC1:
                if (b == UBX_SYNC1) s_state = WAIT_SYNC2;
                break;
            case WAIT_SYNC2:
                if (b == UBX_SYNC2) {
                    s_state = WAIT_CLASS;
                    s_cka = 0;
                    s_ckb = 0;
                } else if (b != UBX_SYNC1) {
                    s_state = WAIT_SYNC1;
                }
                break;
            case WAIT_CLASS:
                s_class = b;
                update_checksum(b);
                s_state = WAIT_ID;
                break;
            case WAIT_ID:
                s_id = b;
                update_checksum(b);
                s_state = WAIT_LEN1;
                break;
            case WAIT_LEN1:
                s_len = b;
                update_checksum(b);
                s_state = WAIT_LEN2;
                break;
            case WAIT_LEN2:
                s_len |= (uint16_t)b << 8;
                update_checksum(b);
                s_idx = 0;
                if (s_len > 0 && s_len <= NAV_PVT_LEN) {
                    s_state = WAIT_PAYLOAD;
                } else if (s_len == 0) {
                    s_state = WAIT_CKA;
                } else {
                    reset_parser(); // payload too big
                }
                break;
            case WAIT_PAYLOAD:
                if (s_idx < NAV_PVT_LEN) {
                    s_payload[s_idx] = b;
                }
                update_checksum(b);
                s_idx++;
                if (s_idx >= s_len) {
                    s_state = WAIT_CKA;
                }
                break;
            case WAIT_CKA:
                if (b == s_cka) {
                    s_state = WAIT_CKB;
                } else {
                    reset_parser();
                }
                break;
            case WAIT_CKB:
                if (b == s_ckb) {
                    // Valid message
                    if (s_class == UBX_CLASS_NAV && s_id == UBX_ID_NAV_PVT && s_len == NAV_PVT_LEN) {
                        parse_nav_pvt(s_payload, out);
                        reset_parser();
                        return true;
                    }
                }
                reset_parser();
                break;
            }
        }
    }

    return false;
}
