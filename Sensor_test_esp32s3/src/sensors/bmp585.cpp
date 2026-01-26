#include "bmp585.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common/i2c_bus.h"
#include "third_party/bmp5/bmp5.h"
#include "third_party/bmp5/bmp5_defs.h"
#include <math.h>
#include <string.h>

static const char *TAG = "BMP585";

typedef struct {
    i2c_port_t port;
    uint8_t addr;
} bmp585_i2c_ctx_t;

static bmp585_i2c_ctx_t s_ctx = {};
static struct bmp5_dev s_dev = {};
static struct bmp5_osr_odr_press_config s_osr_cfg = {};
static float s_sea_level_pa = 101325.0f;
static gpio_num_t s_int_pin = GPIO_NUM_NC;
static bool s_int_active_low = true;
static bool s_use_int = false;
static volatile bool s_drdy = false;
static uint64_t s_last_read_us = 0;
static const uint64_t k_force_poll_interval_us = 1200000;
static bool s_inited = false;

static BMP5_INTF_RET_TYPE bmp585_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    bmp585_i2c_ctx_t *ctx = (bmp585_i2c_ctx_t *)intf_ptr;
    esp_err_t ret = i2c_master_write_read_device(ctx->port, ctx->addr, &reg_addr, 1, read_data, len, pdMS_TO_TICKS(100));
    return (ret == ESP_OK) ? BMP5_INTF_RET_SUCCESS : (BMP5_INTF_RET_TYPE)-1;
}

static BMP5_INTF_RET_TYPE bmp585_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    bmp585_i2c_ctx_t *ctx = (bmp585_i2c_ctx_t *)intf_ptr;
    uint8_t buf[1 + 32];
    if (len > 32) {
        return (BMP5_INTF_RET_TYPE)-1;
    }
    buf[0] = reg_addr;
    if (len > 0) {
        memcpy(&buf[1], data, len);
    }
    esp_err_t ret = i2c_master_write_to_device(ctx->port, ctx->addr, buf, len + 1, pdMS_TO_TICKS(100));
    return (ret == ESP_OK) ? BMP5_INTF_RET_SUCCESS : (BMP5_INTF_RET_TYPE)-1;
}

static void bmp585_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if (period == 0) return;
    esp_rom_delay_us(period);
}

static void IRAM_ATTR bmp585_isr(void *arg)
{
    (void)arg;
    s_drdy = true;
}

static esp_err_t bmp585_configure_interrupts(void)
{
    if (!s_use_int || s_int_pin == GPIO_NUM_NC) {
        return ESP_OK;
    }

    gpio_config_t io_cfg = {};
    io_cfg.pin_bit_mask = 1ULL << s_int_pin;
    io_cfg.mode = GPIO_MODE_INPUT;
    io_cfg.intr_type = s_int_active_low ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
    io_cfg.pull_up_en = s_int_active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io_cfg.pull_down_en = s_int_active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;

    esp_err_t ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) return ret;

    ret = gpio_isr_handler_add(s_int_pin, bmp585_isr, NULL);
    if (ret != ESP_OK) return ret;

    struct bmp5_int_source_select src = {};
    src.drdy_en = BMP5_ENABLE;
    src.fifo_full_en = BMP5_DISABLE;
    src.fifo_thres_en = BMP5_DISABLE;
    src.oor_press_en = BMP5_DISABLE;

    int8_t rslt = bmp5_int_source_select(&src, &s_dev);
    if (rslt != BMP5_OK) return ESP_FAIL;

    enum bmp5_intr_polarity pol = s_int_active_low ? BMP5_ACTIVE_LOW : BMP5_ACTIVE_HIGH;
    rslt = bmp5_configure_interrupt(BMP5_PULSED, pol, BMP5_INTR_OPEN_DRAIN, BMP5_INTR_ENABLE, &s_dev);
    if (rslt != BMP5_OK) return ESP_FAIL;

    s_drdy = false;
    return ESP_OK;
}

static esp_err_t bmp585_try_init(uint8_t addr)
{
    s_ctx.addr = addr;
    s_dev.intf_ptr = &s_ctx;
    s_dev.intf = BMP5_I2C_INTF;
    s_dev.read = bmp585_i2c_read;
    s_dev.write = bmp585_i2c_write;
    s_dev.delay_us = bmp585_delay_us;

    int8_t rslt = bmp5_soft_reset(&s_dev);
    if (rslt != BMP5_OK) {
        ESP_LOGW(TAG, "soft reset before init failed (%d)", rslt);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    bool skip_reset_after_init = false;
    rslt = bmp5_init(&s_dev);
    if (rslt != BMP5_OK) {
        uint8_t chip_id = 0;
        int8_t id_rslt = bmp5_get_regs(BMP5_REG_CHIP_ID, &chip_id, 1, &s_dev);
        if (id_rslt == BMP5_OK &&
            (chip_id == BMP5_CHIP_ID_PRIM || chip_id == BMP5_CHIP_ID_SEC)) {
            if (rslt != BMP5_E_POWER_UP && rslt != BMP5_E_POR_SOFTRESET) {
                ESP_LOGW(TAG, "bmp5_init failed (%d), continuing with chip_id=0x%02X", rslt, chip_id);
            }
            if (rslt == BMP5_E_POWER_UP || rslt == BMP5_E_POR_SOFTRESET) {
                skip_reset_after_init = true;
            }
        } else {
            return ESP_FAIL;
        }
    }

    if (!skip_reset_after_init) {
        rslt = bmp5_soft_reset(&s_dev);
        if (rslt != BMP5_OK) {
            ESP_LOGW(TAG, "soft reset after init failed (%d)", rslt);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_osr_cfg.osr_t = BMP5_OVERSAMPLING_128X;
    s_osr_cfg.osr_p = BMP5_OVERSAMPLING_128X;
    s_osr_cfg.press_en = BMP5_ENABLE;
    s_osr_cfg.odr = BMP5_ODR_10_HZ;

    rslt = bmp5_set_osr_odr_press_config(&s_osr_cfg, &s_dev);
    if (rslt != BMP5_OK) {
        return ESP_FAIL;
    }

    struct bmp5_iir_config iir_cfg = {};
    iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_127;
    iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_127;
    rslt = bmp5_set_iir_config(&iir_cfg, &s_dev);
    if (rslt != BMP5_OK) {
        return ESP_FAIL;
    }

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &s_dev);
    if (rslt != BMP5_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t bmp585_try_init_with_retry(uint8_t addr, int attempts, uint32_t delay_ms)
{
    for (int i = 0; i < attempts; ++i) {
        if (bmp585_try_init(addr) == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    return ESP_FAIL;
}

esp_err_t bmp585_init(const bmp585_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_inited) return ESP_OK;

    s_ctx.port = cfg->i2c_port;
    s_sea_level_pa = (cfg->sea_level_pa > 0.0f) ? cfg->sea_level_pa : 101325.0f;
    s_int_pin = cfg->int_pin;
    s_int_active_low = cfg->int_active_low;
    s_use_int = (s_int_pin != GPIO_NUM_NC);

    esp_err_t ret = i2c_bus_init(cfg->i2c_port, cfg->sda_pin, cfg->scl_pin, cfg->freq_hz);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t addr = cfg->i2c_addr;
    if (addr != 0) {
        ret = bmp585_try_init_with_retry(addr, 3, 20);
        if (ret != ESP_OK) return ret;
    } else {
        ret = bmp585_try_init_with_retry(BMP5_I2C_ADDR_PRIM, 3, 20);
        if (ret != ESP_OK) {
            ret = bmp585_try_init_with_retry(BMP5_I2C_ADDR_SEC, 3, 20);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "BMP5 not found on I2C (0x%02X/0x%02X)", BMP5_I2C_ADDR_PRIM, BMP5_I2C_ADDR_SEC);
                return ret;
            }
        }
    }

    ret = bmp585_configure_interrupts();
    if (ret != ESP_OK) return ret;

    s_inited = true;
    ESP_LOGI(TAG, "BMP5 ready (I2C%d, addr=0x%02X)", s_ctx.port, s_ctx.addr);
    return ESP_OK;
}

esp_err_t bmp585_set_power(bool enable)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    enum bmp5_powermode mode = enable ? BMP5_POWERMODE_NORMAL : BMP5_POWERMODE_STANDBY;
    int8_t rslt = bmp5_set_power_mode(mode, &s_dev);
    return (rslt == BMP5_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t bmp585_read(bmp585_data_t *out)
{
    if (!s_inited || !out) return ESP_ERR_INVALID_STATE;
    if (s_use_int && !s_drdy) {
        uint64_t now = esp_timer_get_time();
        if (s_last_read_us != 0 && (now - s_last_read_us) < k_force_poll_interval_us) {
            return ESP_ERR_TIMEOUT;
        }
        s_drdy = true;
    }

    struct bmp5_sensor_data data = {};
    int8_t rslt = bmp5_get_sensor_data(&data, &s_osr_cfg, &s_dev);
    if (rslt != BMP5_OK) {
        return ESP_FAIL;
    }
    s_drdy = false;
    s_last_read_us = esp_timer_get_time();

    out->temperature_c = data.temperature;
    out->pressure_pa = data.pressure;
    out->pressure_hpa = data.pressure / 100.0f;
    if (s_sea_level_pa > 0.0f && data.pressure > 0.0f) {
        out->altitude_m = 44330.0f * (1.0f - powf(data.pressure / s_sea_level_pa, 0.1903f));
    } else {
        out->altitude_m = 0.0f;
    }
    out->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    out->valid = true;
    return ESP_OK;
}
