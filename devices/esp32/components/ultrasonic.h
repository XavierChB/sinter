/**
 * @file ultrasonic.h
 * @defgroup ultrasonic ultrasonic
 * @{
 *
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and so on
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ESP_ERR_ULTRASONIC_PING         0x200
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202

#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL  portEXIT_CRITICAL(&mux)

#define TRIGGER_LOW_DELAY  4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT       6000
#define ROUNDTRIP          58

#define ULTRASONIC_TRIG_GPIO (33)
#define ULTRASONIC_ECHO_GPIO (32)

#define timeout_expired(start, len) ((esp_timer_get_time() - (start)) >= (len))

#define CHECK(x)                           \
    do {                                   \
        esp_err_t __;                      \
        if ((__ = x) != ESP_OK) return __; \
    } while (0)

#define RETURN_CRITICAL(RES) \
    do {                     \
        PORT_EXIT_CRITICAL;  \
        return RES;          \
    } while (0)

    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    /**
     * Device descriptor
     */
    typedef struct
    {
        gpio_num_t trigger_pin;  //!< GPIO output pin for trigger
        gpio_num_t echo_pin;     //!< GPIO input pin for echo
    } ultrasonic_sensor_t;

    static ultrasonic_sensor_t default_ultrasonic_config = {.trigger_pin = ULTRASONIC_TRIG_GPIO,
                                                            .echo_pin = ULTRASONIC_ECHO_GPIO};

    /**
     * @brief Init ranging module
     *
     * @param dev Pointer to the device descriptor
     * @return `ESP_OK` on success
     */
    esp_err_t internal_ultrasonic_init(const ultrasonic_sensor_t *dev)
    {
        CHECK(gpio_set_direction(dev->trigger_pin, GPIO_MODE_OUTPUT));
        CHECK(gpio_set_direction(dev->echo_pin, GPIO_MODE_INPUT));

        return gpio_set_level(dev->trigger_pin, 0);
    }

    /**
     * @brief Measure distance
     *
     * @param dev Pointer to the device descriptor
     * @param max_distance Maximal distance to measure, centimeters
     * @param[out] distance Distance in centimeters
     * @return `ESP_OK` on success, otherwise:
     *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
     *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
     */
    esp_err_t internal_ultrasonic_measure_cm(const ultrasonic_sensor_t *dev,
                                             uint32_t max_distance,
                                             uint32_t *distance)
    {
        PORT_ENTER_CRITICAL;

        // Ping: Low for 2..4 us, then high 10 us
        CHECK(gpio_set_level(dev->trigger_pin, 0));
        ets_delay_us(TRIGGER_LOW_DELAY);
        CHECK(gpio_set_level(dev->trigger_pin, 1));
        ets_delay_us(TRIGGER_HIGH_DELAY);
        CHECK(gpio_set_level(dev->trigger_pin, 0));

        // Previous ping isn't ended
        if (gpio_get_level(dev->echo_pin)) RETURN_CRITICAL(ESP_ERR_ULTRASONIC_PING);

        // Wait for echo
        int64_t start = esp_timer_get_time();
        while (!gpio_get_level(dev->echo_pin)) {
            if (timeout_expired(start, PING_TIMEOUT))
                RETURN_CRITICAL(ESP_ERR_ULTRASONIC_PING_TIMEOUT);
        }

        // got echo, measuring
        int64_t echo_start = esp_timer_get_time();
        int64_t time = echo_start;
        int64_t meas_timeout = max_distance * ROUNDTRIP;
        while (gpio_get_level(dev->echo_pin)) {
            time = esp_timer_get_time();
            if (timeout_expired(echo_start, meas_timeout))
                RETURN_CRITICAL(ESP_ERR_ULTRASONIC_ECHO_TIMEOUT);
        }
        PORT_EXIT_CRITICAL;

        *distance = (time - echo_start) / ROUNDTRIP;

        return ESP_OK;
    }

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ULTRASONIC_H__ */
