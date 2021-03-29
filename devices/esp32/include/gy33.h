#ifndef _ESP_GY33_H_
#define _ESP_GY33_H_
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "i2c_function.h"

#define GY_33_ADDR       (0x5A)
#define GY_33_PORT       I2C_NUM_0
#define GY_I2C_MODE      I2C_MODE_MASTER
#define GY_SCL_GPIO      GPIO_NUM_26
#define GY_SDA_GPIO      GPIO_NUM_27
#define GY_SCL_PULLUP_EN GPIO_PULLUP_ENABLE
#define GY_SDA_PULLUP_EN GPIO_PULLUP_ENABLE
#define GY_CLK_FREQ      (100000)
#define GY_ACK_CHECK_EN  (0x1)
#define GY_ACK_CHECK_DIS (0x0)
#define ACK_VAL          (0x0)
#define NACK_VAL         (0x1)

#define GY_RED_RAW_REG   (0x00)
#define GY_GREEN_RAW_REG (0x02)
#define GY_BLUE_RAW_REG  (0x04)
#define GY_CLEAR_RAW_REG (0x06)
#define GY_LUX_REG       (0x08)
#define GY_CT_REG        (0x0A)
#define GY_R_REG         (0x0C)
#define GY_G_REG         (0x0D)
#define GY_B_REG         (0x0E)
#define GY_COLOR_REG     (0x0F)
#define GY_CONFIG_REG    (0x10)

#define WAIT_25_MS     (25)
#define WAIT_50_MS     (50)
#define CHECK_ERR(err) (err != ESP_OK)

i2c_config_t gy33_i2c_config = {.scl_io_num = GY_SCL_GPIO,
                                .sda_io_num = GY_SDA_GPIO,
                                .scl_pullup_en = GY_SCL_PULLUP_EN,
                                .sda_pullup_en = GY_SDA_PULLUP_EN,
                                .mode = GY_I2C_MODE,
                                .master.clk_speed = GY_CLK_FREQ};
i2c_device_t gy33;
const uint8_t rgbc_raw_register_address[4] = {GY_RED_RAW_REG, GY_GREEN_RAW_REG, GY_BLUE_RAW_REG,
                                              GY_CLEAR_RAW_REG};
const uint8_t rgb_register_address[3] = {GY_R_REG, GY_G_REG, GY_B_REG};
i2c_cmd_handle_t cmd;

typedef struct
{
    uint16_t rgbc[4];
} rgbc_raw;

typedef struct
{
    uint8_t rgb[3];
} rgb;

void gy33_err_print(char *fn_name, char *read_write, uint8_t addr)
{
    fprintf(stderr, "ERROR in gy33::%s:\n\t%s@%02X failed\n", fn_name, read_write, addr);
}

esp_err_t internal_gy33_init(uint8_t led_brightness, bool auto_white_balance)
{
    esp_err_t ret = esp_i2c_init_device(&gy33, GY_33_ADDR, GY_33_PORT, &gy33_i2c_config);
    if (CHECK_ERR(ret))
        fprintf(stderr, "ERROR in gy33::internal_gy33_init: initialization failed\n");
    uint8_t led_brightness_cmd = 0x0a - led_brightness;
    esp_i2c_write_byte(&gy33, GY_CONFIG_REG, (led_brightness << 4) | (uint8_t)auto_white_balance,
                       WAIT_50_MS);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return ret;
}

esp_err_t internal_get_rgbc_raw(rgbc_raw *rgbc_raw_store)
{
    for (int i = 0; i < sizeof(rgbc_raw_store->rgbc) / sizeof(*(rgbc_raw_store->rgbc)); i++) {
        esp_err_t ret = esp_i2c_read_16(&gy33, rgbc_raw_register_address[i],
                                        (uint8_t *)&(rgbc_raw_store->rgbc[i]), WAIT_25_MS);
        if (CHECK_ERR(ret)) {
            gy33_err_print("internal_get_rgbc_raw", "read", rgbc_raw_register_address[i]);
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t internal_get_rgb(rgb *rgb_store)
{
    for (int i = 0; i < sizeof(rgb_store->rgb) / sizeof(*(rgb_store->rgb)); i++) {
        esp_err_t ret = esp_i2c_read_byte(&gy33, rgb_register_address[i],
                                          (uint8_t *)&(rgb_store->rgb[i]), WAIT_25_MS);
        if (CHECK_ERR(ret)) {
            gy33_err_print("internal_get_rgb", "read", rgb_register_address[i]);
            return ret;
        }
    }
}

esp_err_t internal_get_lux(uint16_t *lux)
{
    esp_err_t ret = esp_i2c_read_16(&gy33, GY_LUX_REG, (uint8_t *)lux, WAIT_25_MS);
    if (CHECK_ERR(ret)) { gy33_err_print("internal_get_lux", "read", GY_COLOR_REG); }
    return ret;
}

esp_err_t internal_get_color_temperature(uint16_t *ct)
{
    esp_err_t ret = esp_i2c_read_16(&gy33, GY_CT_REG, (uint8_t *)ct, WAIT_25_MS);
    if (CHECK_ERR(ret)) { gy33_err_print("internal_get_color_temperature", "read", GY_COLOR_REG); }
    return ret;
}

esp_err_t internal_get_red(uint8_t *red_v)
{
    esp_err_t ret = esp_i2c_read_byte(&gy33, GY_R_REG, red_v, WAIT_25_MS);
    if (CHECK_ERR(ret)) { gy33_err_print("internal_get_red", "read", GY_COLOR_REG); }
    return ret;
}

esp_err_t internal_get_green(uint8_t *green_v)
{
    esp_err_t ret = esp_i2c_read_byte(&gy33, GY_G_REG, green_v, WAIT_25_MS);
    if (CHECK_ERR(ret)) { gy33_err_print("internal_get_green", "read", GY_COLOR_REG); }
    return ret;
}

esp_err_t internal_get_blue(uint8_t *blue_v)
{
    esp_err_t ret = esp_i2c_read_byte(&gy33, GY_B_REG, blue_v, WAIT_25_MS);
    if (CHECK_ERR(ret)) { gy33_err_print("internal_get_blue", "read", GY_COLOR_REG); }
    return ret;
}

esp_err_t internal_get_basic_color(uint8_t *color_v)
{
    uint8_t res;
    esp_err_t ret = esp_i2c_read_byte(&gy33, GY_COLOR_REG, &res, WAIT_25_MS);
    if (CHECK_ERR(ret)) {
        gy33_err_print("internal_get_basic_color", "read", GY_COLOR_REG);
        return ret;
    }
    for (uint8_t i = 0; i < 8; ++i) {
        if (ret & (1 << i)) {
            *color_v = i;
            break;
        }
    }
    return ret;
}
#endif
