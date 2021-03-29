#ifndef _ESP_TCS34725_H_
#define _ESP_TCS34725_H_
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"

#define TCS34725_ADDRESS (0x29) /**< I2C address **/

#define TCS_ESP_MODE      I2C_MODE_MASTER
#define I2C_MASTER_SCL_IO GPIO_NUM_26
#define I2C_MASTER_SDA_IO GPIO_NUM_27
#define I2C_CLK_FREQ      40000
#define TCS_I2C_CHANNEL   I2C_NUM_0
#define TCS_ACK_EN        (0x1)
#define TCS_CMD_S         (0xA0) /**< Use if need to read byte successively */
#define TCS_CMD_R         (0x80) /**< Use if want to read 1 byte only */
#define TCS_ATIME_ADDR    (0x01) /**< Set rgb integration time here */
#define TCS_WTIME_ADDR    (0x03) /**< Set wait time here */
#define TCS_STATUS_ADDR   (0x13) /**< Address of current status */
#define TCS_CL_ADDR       (0x14) /**< Clear low byte */
#define TCS_RL_ADDR       (0x16) /**< Red low byte */
#define TCS_GL_ADDR       (0x18) /**< Green low byte */
#define TCS_BL_ADDR       (0x1A) /**< Blue low byte */
#define TCS_ENABLE        (0x00) /*< Enable register */
#define TCS_ENABLE_AEN    (0x02) /**< RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define TCS_ENABLE_PON    (0x01) /**< Power on - activates the internal oscillator */
#define TCS_RGB_INT_TIME  (0xF6) /**< 24ms colour integration cycle */
#define TCS_NO_REG        (0x1F) /**< register place holder */

#define I2C_BEGIN(mode) i2c_master_write_byte(cmd, (TCS34725_ADDRESS << 1) | (mode), TCS_ACK_EN);

uint8_t crgb_register_address[4] = {TCS_CL_ADDR, TCS_RL_ADDR, TCS_GL_ADDR, TCS_BL_ADDR};
uint8_t cur_register = TCS_NO_REG;
uint8_t tcs_cmd_mode = 0x00;
i2c_cmd_handle_t cmd;

typedef struct
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} rgbc;

typedef uint8_t tcs_status_t;

i2c_config_t tcs_i2c_config = {.mode = TCS_ESP_MODE,
                               .sda_io_num = I2C_MASTER_SDA_IO,
                               .scl_io_num = I2C_MASTER_SCL_IO,
                               .sda_pullup_en = GPIO_PULLUP_ENABLE,
                               .scl_pullup_en = GPIO_PULLUP_ENABLE,
                               .master.clk_speed = I2C_CLK_FREQ};

esp_err_t read_byte(uint8_t reg, uint8_t *res)
{
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    uint8_t read_mode = TCS_CMD_R;
    if (cur_register != reg || tcs_cmd_mode != read_mode) {
        I2C_BEGIN(I2C_MASTER_WRITE);
        i2c_master_write_byte(cmd, (read_mode | reg), TCS_ACK_EN);
        cur_register = reg;
        tcs_cmd_mode = read_mode;
    }
    I2C_BEGIN(I2C_MASTER_READ);
    i2c_master_read(cmd, (uint8_t *)res, 16, TCS_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(TCS_I2C_CHANNEL, cmd, 50 / portTICK_PERIOD_MS);
    if (err != ESP_OK) { fprintf(stderr, "read byte failed\n"); }
    i2c_cmd_link_delete(cmd);
    fprintf(stdout, "register#%02x, Read: %02x\n", reg, *res);
    return err;
}

esp_err_t read_16(uint8_t reg, uint16_t *res)
{
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    uint8_t read_mode = TCS_CMD_S;
    if (cur_register != reg || tcs_cmd_mode != read_mode) {
        I2C_BEGIN(I2C_MASTER_WRITE);
        i2c_master_write_byte(cmd, (read_mode | reg), TCS_ACK_EN);
        cur_register = reg;
        tcs_cmd_mode = read_mode;
    }
    I2C_BEGIN(I2C_MASTER_READ);
    i2c_master_read(cmd, (uint8_t *)res, 16, TCS_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(TCS_I2C_CHANNEL, cmd, 50 / portTICK_PERIOD_MS);
    if (err != ESP_OK) { fprintf(stderr, "read word failed\n"); }
    i2c_cmd_link_delete(cmd);
    fprintf(stdout, "register#%02x, Read: %04x\n", reg, *res);
    return err;
}

esp_err_t write_byte(uint8_t reg, uint8_t data)
{
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    I2C_BEGIN(I2C_MASTER_WRITE);
    if (cur_register != reg || tcs_cmd_mode != TCS_CMD_R) {
        i2c_master_write_byte(cmd, (TCS_CMD_R | reg), TCS_ACK_EN);
        cur_register = reg;
        tcs_cmd_mode = TCS_CMD_R;
    }
    i2c_master_write_byte(cmd, data, TCS_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(TCS_I2C_CHANNEL, cmd, 50 / portTICK_PERIOD_MS);
    if (err != ESP_OK) { fprintf(stderr, "write byte failed\n"); }
    i2c_cmd_link_delete(cmd);
    fprintf(stdout, "register#%02x, Wrote: %02x\n", reg, data);
    return err;
}

void internal_tcs_init()
{
    if (i2c_param_config(TCS_I2C_CHANNEL, &tcs_i2c_config) != ESP_OK)
        fprintf(stderr, "param config failed\n");
    if (i2c_driver_install(TCS_I2C_CHANNEL, TCS_ESP_MODE, 0, 0, 0) != ESP_OK)
        fprintf(stderr, "driver install failed\n");
    write_byte(TCS_ENABLE, (TCS_ENABLE_PON | TCS_ENABLE_AEN));
}

esp_err_t internal_get_rgb(rgbc *rgb_store)
{
    tcs_status_t status;
    uint16_t raw_arr[4];
    for (int i = 0; i < 4; ++i) read_16(crgb_register_address[i], &raw_arr[i]);
    rgb_store->clear = raw_arr[0];
    rgb_store->red = raw_arr[1];
    rgb_store->green = raw_arr[2];
    rgb_store->blue = raw_arr[3];
}

#endif
