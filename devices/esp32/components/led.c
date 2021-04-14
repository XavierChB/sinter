#include "led.h"

#include <esp_vfs_dev.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/ledc.h"
#include "esp_err.h"

const ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
    {.channel = LEDC_CH0_CHANNEL,
     .duty = 0,
     .gpio_num = LEDC_CH0_GPIO,
     .speed_mode = LEDC_MODE,
     .hpoint = 0,
     .timer_sel = LEDC_TIMER},
    {.channel = LEDC_CH1_CHANNEL,
     .duty = 0,
     .gpio_num = LEDC_CH1_GPIO,
     .speed_mode = LEDC_MODE,
     .hpoint = 0,
     .timer_sel = LEDC_TIMER},
    //{.channel = LEDC_CH2_CHANNEL,
    //.duty = 0,
    //.gpio_num = LEDC_CH2_GPIO,
    //.speed_mode = LEDC_MODE,
    //.hpoint = 0,
    //.timer_sel = LEDC_TIMER},
    //{.channel = LEDC_CH3_CHANNEL,
    //.duty = 0,
    //.gpio_num = LEDC_CH3_GPIO,
    //.speed_mode = LEDC_MODE,
    //.hpoint = 0,
    //.timer_sel = LEDC_TIMER},
    //{.channel = LEDC_CH4_CHANNEL,
    //.duty = 0,
    //.gpio_num = LEDC_CH4_GPIO,
    //.speed_mode = LEDC_MODE,
    //.hpoint = 0,
    //.timer_sel = LEDC_TIMER},
    //{.channel = LEDC_CH5_CHANNEL,
    //.duty = 0,
    //.gpio_num = LEDC_CH5_GPIO,
    //.speed_mode = LEDC_MODE,
    //.hpoint = 0,
    //.timer_sel = LEDC_TIMER},
};

const ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_13_BIT,  // resolution of PWM duty
    .freq_hz = 5000,                       // frequency of PWM signal
    .speed_mode = LEDC_MODE,               // timer mode
    .timer_num = LEDC_TIMER,               // timer index
    .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
};
// Set LED Controller with previously prepared configuration

bool internal_led_init()
{
    esp_err_t res0 = ledc_timer_config(&ledc_timer);
    for (int ch = 0; ch < LEDC_TEST_CH_NUM; ch++) { ledc_channel_config(&ledc_channel[ch]); }
    esp_err_t res1 = ledc_fade_func_install(0);
    return res0 == ESP_OK && res1 == ESP_OK;
}

void internal_led_on(unsigned ch)
{
    ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
    ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
}

void internal_led_off(unsigned ch)
{
    ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
    ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
}

void internal_led_fade(unsigned ch, unsigned target_duty, unsigned fade_time_ms)
{
    ledc_set_fade_with_time(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, target_duty,
                            fade_time_ms);
    ledc_fade_start(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
}
