#ifndef _ESP_LED_H_
#define _ESP_LED_H_

#include <esp_vfs_dev.h>
#include <stdbool.h>

/* LEDC (LED Controller) fade example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_HIGH_SPEED_MODE
#define LEDC_CH0_GPIO    (18)
#define LEDC_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_CH1_GPIO    (19)
#define LEDC_CH1_CHANNEL LEDC_CHANNEL_1
//#define LEDC_CH2_GPIO    (4)
//#define LEDC_CH2_CHANNEL LEDC_CHANNEL_2
//#define LEDC_CH3_GPIO    (5)
//#define LEDC_CH3_CHANNEL LEDC_CHANNEL_3
//#define LEDC_CH4_GPIO    (17)
//#define LEDC_CH4_CHANNEL LEDC_CHANNEL_4
//#define LEDC_CH5_GPIO    (16)
//#define LEDC_CH5_CHANNEL LEDC_CHANNEL_5

#define LEDC_TEST_CH_NUM (2)
#define LEDC_TEST_DUTY   (5000)

// Set LED Controller with previously prepared configuration

bool internal_led_init();

void internal_led_on(unsigned ch);

void internal_led_off(unsigned ch);

void internal_led_fade(unsigned ch, unsigned target_duty, unsigned fade_time_ms);

#endif
