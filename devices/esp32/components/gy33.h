#ifndef _ESP_GY33_H_
#define _ESP_GY33_H_
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
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

typedef struct
{
    uint16_t rgbc[4];
} rgbc_raw;

typedef struct
{
    uint8_t rgb[3];
} rgb;

esp_err_t internal_gy33_init(uint8_t led_brightness, bool auto_white_balance);

esp_err_t internal_get_rgbc_raw(rgbc_raw *rgbc_raw_store);

esp_err_t internal_get_rgb(rgb *rgb_store);

esp_err_t internal_get_lux(uint16_t *lux);

esp_err_t internal_get_color_temperature(uint16_t *ct);

esp_err_t internal_get_red(uint8_t *red_v);

esp_err_t internal_get_green(uint8_t *green_v);

esp_err_t internal_get_blue(uint8_t *blue_v);

esp_err_t internal_get_basic_color(uint8_t *color_v);

#endif
