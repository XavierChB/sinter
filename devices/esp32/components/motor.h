#ifndef _ESP_MOTOR_H_
#define _ESP_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CW  1
#define CCW 0

#define MOTOR_GPIO_0 GPIO_NUM_15
#define MOTOR_GPIO_1 GPIO_NUM_2
#define MOTOR_GPIO_2 GPIO_NUM_0
#define MOTOR_GPIO_3 GPIO_NUM_4

typedef struct
{
    uint8_t pin1;
    uint8_t pin2;
    uint8_t pin3;
    uint8_t pin4;
} stepper_pins;

static stepper_pins default_motor_config = {.pin1 = MOTOR_GPIO_0,
                                            .pin2 = MOTOR_GPIO_1,
                                            .pin3 = MOTOR_GPIO_2,
                                            .pin4 = MOTOR_GPIO_3};

void internal_motor_init(stepper_pins *stepper_ptr);

void internal_motor_step(stepper_pins *stepper_ptr, int step);

void internal_motor_full_steps(stepper_pins *stepper_ptr, int steps, bool dir);

#endif
