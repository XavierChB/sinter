#ifndef _ESP_SOURCE_I2C_FUNCTION_H_
#define _ESP_SOURCE_I2C_FUNCTION_H_

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"

#define ACK_CHECK_EN  (0x1)
#define ACK_CHECK_DIS (0x0)
#define ACK_VAL       (0x0)
#define NACK_VAL      (0x1)

typedef uint8_t i2c_device_address_t;
typedef uint8_t i2c_register_address_t;

typedef enum
{
    esp_i2c_0 = 0,
    esp_i2c_1 = 1
} esp_i2c_port_t;

typedef struct
{
    i2c_register_address_t last_read_register;
    i2c_register_address_t last_write_register;
} i2c_device_store;

typedef struct
{
    i2c_config_t *config;
    i2c_device_address_t address;
    esp_i2c_port_t port;
    i2c_device_store *mem;
} i2c_device_t;

static const i2c_device_store empty_store = {.last_read_register = 0xff,
                                             .last_write_register = 0xff};

esp_err_t esp_i2c_init_device(i2c_device_t *device,
                              i2c_device_address_t addr,
                              esp_i2c_port_t port,
                              i2c_config_t *channel_config)
{
    i2c_device_store mem = empty_store;
    i2c_device_t temp = {.config = channel_config, .address = addr, .port = port, .mem = &mem};
    *device = temp;
    esp_err_t err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    i2c_param_config(port, channel_config);
    if (err != ESP_OK) {
        fprintf(stderr,
                "ERROR in i2c_function::init_device:\n\tAttempt to initialize device@%02x to IIC "
                "port-%d is unsuccessful\n",
                addr, port);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        fprintf(stderr, "%02x initialized and connected\n", device->address);
    } else if (ret == ESP_ERR_TIMEOUT) {
        fprintf(stderr, "%02x time out\n", device->address);
    } else {
        fprintf(stderr, "%02x connection failed\n", device->address);
    }
    return err;
}

esp_err_t esp_i2c_write_byte(i2c_device_t *device, uint8_t reg_addr, uint8_t data, int wait_ms)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(device->port, cmd, wait_ms / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        fprintf(stderr,
                "ERROR in i2c_function::write_byte:\n\tAttempt to write %d to device@%02x "
                "register-%02x is unsuccessful; error code: %03X\n",
                data, device->address, reg_addr, ret);
    } else {
        fprintf(stderr,
                "i2c_function::write_byte:\n\tAttempt to write %d to device@%02x "
                "register-%02x is successful\n",
                data, device->address, reg_addr);
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t esp_i2c_write_16(i2c_device_t *device, uint8_t reg_addr, uint16_t data, int wait_ms)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t *)&data, 16, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(device->port, cmd, wait_ms / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        fprintf(stderr,
                "ERROR in i2c_function::write_16:\n\tAttempt to write %d to device@%02x "
                "register-%02x is unsuccessful; error code: %03X\n",
                data, device->address, reg_addr, ret);
    } else {
        fprintf(stderr,
                "i2c_function::write_16:\n\tAttempt to write %d to device@%02x "
                "register-%02x is successful\n",
                data, device->address, reg_addr);
    }
    i2c_cmd_link_delete(cmd);
}

esp_err_t esp_i2c_read_byte(i2c_device_t *device, uint8_t reg_addr, uint8_t *res, int wait_ms)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, res, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(device->port, cmd, wait_ms / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        fprintf(stderr,
                "ERROR in i2c_function::read_byte:\n\tAttempt to read from device@%02X "
                "register-%02X is unsuccessful; error code: %03X\n",
                device->address, reg_addr, ret);
    } else {
        fprintf(stderr,
                "in i2c_function::read_byte:\n\tAttempt to read from to device@%02x "
                "register-%02x is successful\n",
                device->address, reg_addr);
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t esp_i2c_read_16(i2c_device_t *device, uint8_t reg_addr, uint8_t *res, int wait_ms)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, res, 2, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(device->port, cmd, wait_ms / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        fprintf(stderr,
                "ERROR in i2c_function::read_byte:\n\tAttempt to read from device@%02x "
                "register-%02x is unsuccessful\n",
                device->address, reg_addr);
    } else {
        fprintf(stderr,
                "i2c_function::read_16:\n\tAttempt to read from to device@%02x "
                "register-%02x is successful\n",
                device->address, reg_addr);
    }
    i2c_cmd_link_delete(cmd);
    return ret;
}
#endif
