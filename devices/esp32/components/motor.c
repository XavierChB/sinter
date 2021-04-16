#include "motor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const uint8_t steps_port[4] = {0x03, 0x06, 0x0C, 0x09};

void internal_motor_init(stepper_pins *stepper_ptr)
{
    gpio_set_direction(stepper_ptr->pin1, GPIO_MODE_OUTPUT);
    gpio_set_direction(stepper_ptr->pin2, GPIO_MODE_OUTPUT);
    gpio_set_direction(stepper_ptr->pin3, GPIO_MODE_OUTPUT);
    gpio_set_direction(stepper_ptr->pin4, GPIO_MODE_OUTPUT);
}

void internal_motor_step(stepper_pins *stepper_ptr, int phase)
{
    gpio_set_level(stepper_ptr->pin1, phase & 1);
    gpio_set_level(stepper_ptr->pin2, (phase & 2) >> 1);
    gpio_set_level(stepper_ptr->pin3, (phase & 4) >> 2);
    gpio_set_level(stepper_ptr->pin4, (phase & 8) >> 3);
}

void internal_motor_full_steps(stepper_pins *stepper_ptr, int steps, bool dir)
{
    int n;
    int i = 0;

    if (dir) {
        for (n = 0; n < steps; n++) {
            i = n % 4;
            internal_motor_step(stepper_ptr, steps_port[i]);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            fprintf(stderr, "%d\n", n);
        }
    } else {
        i = 7;
        for (n = 0; n < steps; n++) {
            if (i < 0) i = 7;
            internal_motor_step(stepper_ptr, steps_port[i]);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            i--;
        }
    }

    internal_motor_stop(stepper_ptr);
}

void internal_motor_stop(stepper_pins *stepper_ptr)
{
    gpio_set_level(stepper_ptr->pin1, 0);
    gpio_set_level(stepper_ptr->pin2, 0);
    gpio_set_level(stepper_ptr->pin3, 0);
    gpio_set_level(stepper_ptr->pin4, 0);
}
