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

esp_err_t esp_i2c_init_device(i2c_device_t *device,
                              i2c_device_address_t addr,
                              esp_i2c_port_t port,
                              i2c_config_t *channel_config);

esp_err_t esp_i2c_write_byte(i2c_device_t *device, uint8_t reg_addr, uint8_t data, int wait_ms);

esp_err_t esp_i2c_write_16(i2c_device_t *device, uint8_t reg_addr, uint16_t data, int wait_ms);

esp_err_t esp_i2c_read_byte(i2c_device_t *device, uint8_t reg_addr, uint8_t *res, int wait_ms);

esp_err_t esp_i2c_read_16(i2c_device_t *device, uint8_t reg_addr, uint8_t *res, int wait_ms);

#endif
