#include <sinter.h>
#include <sinter/heap.h>
#include <sinter/heap_obj.h>
#include <sinter/internal_fn.h>
#include <sinter/nanbox.h>
#include <sinter/vm.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gy33.h"
#include "led.h"
#include "motor.h"
#include "ultrasonic.h"

#define CHECK_ARGS(n)                                           \
    do {                                                        \
        if (argc < n) { sifault(sinter_fault_function_arity); } \
    } while (0)

#ifdef DEBUG
    #define DBG    (bool)true
    #define dbg(a) fprintf(stderr, a)
#else
    #define DBG    (bool)false
    #define dbg(a) void(0)
#endif

static inline sinanbox_t wrap_integer(int v)
{
    return v >= NANBOX_INTMIN && v <= NANBOX_INTMAX ? NANBOX_OFINT(v) : NANBOX_OFFLOAT(v);
}

static inline int nanboxToInt(sinanbox_t v)
{
    if (NANBOX_ISINT(v)) {
        return NANBOX_INT(v);
    } else if (NANBOX_ISFLOAT(v)) {
        return (int)(NANBOX_FLOAT(v));
    } else {
        sifault(sinter_fault_type);
    }

    return 0;
}

static inline bool nanboxToBool(sinanbox_t v)
{
    if (NANBOX_ISBOOL(v)) {
        return NANBOX_BOOL(v);
    } else {
        sifault(sinter_fault_type);
    }
}

static inline unsigned int nanboxToUint(sinanbox_t v)
{
    int r = nanboxToInt(v);
    if (r < 0) { sifault(sinter_fault_type); }
    return (unsigned int)(r);
}

static sinanbox_t init_led(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(0);
    bool result = internal_led_init();
    return NANBOX_OFBOOL(result);
}

static sinanbox_t led_on(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(1);
    int channel = nanboxToUint(argv[0]);
    if (DBG) fprintf(stderr, "led#%d: on\n", channel);
    internal_led_on(channel);
    return NANBOX_OFUNDEF();
}

static sinanbox_t led_off(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(1);
    int channel = nanboxToUint(argv[0]);
    if (DBG) fprintf(stderr, "led#%d: off\n", channel);
    internal_led_off(channel);
    return NANBOX_OFUNDEF();
}

static sinanbox_t led_fade(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(3);
    int channel = nanboxToUint(argv[0]);
    int target_duty = nanboxToUint(argv[1]);
    int fade_time = nanboxToUint(argv[2]);
    if (DBG)
        fprintf(stderr, "led#%d: fade; target_duty: %d; fade_time: %dms\n", channel, target_duty,
                fade_time);
    internal_led_fade(channel, target_duty, fade_time);
    return NANBOX_OFUNDEF();
}

static sinanbox_t color_sensor_init(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(1);
    int led_brightness = nanboxToUint(argv[0]);
    bool auto_white_balancing = nanboxToBool(argv[1]);
    internal_gy33_init(led_brightness, auto_white_balancing);
    return NANBOX_OFUNDEF();
}

static sinanbox_t color_get_rgbc_raw(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(0);
    rgbc_raw res;
    internal_get_rgbc_raw(&res);
    siheap_array_t *arr = siarray_new(sizeof(res.rgbc) / sizeof(*(res.rgbc)));
    for (int i = 0; i < sizeof(res.rgbc) / sizeof(*(res.rgbc)); ++i) {
        siarray_put(arr, i, wrap_integer(((uint16_t *)(&res))[i]));
    }
    return SIHEAP_PTRTONANBOX(arr);
}

static sinanbox_t color_get_rgb(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(0);
    rgb res;
    internal_get_rgb(&res);
    siheap_array_t *arr = siarray_new(sizeof(res.rgb) / sizeof(*(res.rgb)));
    for (int i = 0; i < sizeof(res.rgb) / sizeof(*(res.rgb)); ++i) {
        siarray_put(arr, i, wrap_integer(((uint8_t *)(&res))[i]));
    }
    return SIHEAP_PTRTONANBOX(arr);
}

static sinanbox_t color_get_lux(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(0);
    uint16_t res;
    internal_get_lux(&res);
    return wrap_integer(res);
}

static sinanbox_t color_get_color_temperature(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(0);
    uint16_t res;
    internal_get_color_temperature(&res);
    return wrap_integer(res);
}

static sinanbox_t motor_init(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(0);
    internal_motor_init(&default_left_motor_config);
    return NANBOX_OFUNDEF();
}

static sinanbox_t motor_full_steps(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(2);
    int steps = nanboxToInt(argv[0]);
    bool direction = nanboxToBool(argv[1]);
    internal_motor_full_steps(&default_left_motor_config, steps, direction);
    return NANBOX_OFUNDEF();
}

static sinanbox_t imu_init(uint8_t argc, sinanbox_t *argv)
{
    // TODO;
    return NANBOX_OFUNDEF();
}

static sinanbox_t imu_get_pitch(uint8_t argc, sinanbox_t *argv)
{
    // TODO;
    return NANBOX_OFUNDEF();
}

static sinanbox_t imu_get_yaw(uint8_t argc, sinanbox_t *argv)
{
    // TODO;
    return NANBOX_OFUNDEF();
}

static sinanbox_t imu_get_roll(uint8_t argc, sinanbox_t *argv)
{
    // TODO;
    return NANBOX_OFUNDEF();
}

static sinanbox_t imu_set_zero_angle(uint8_t argc, sinanbox_t *argv)
{
    // TODO;
    return NANBOX_OFUNDEF();
}

static sinanbox_t imu_get_angle(uint8_t argc, sinanbox_t *argv)
{
    // TODO;
    return NANBOX_OFUNDEF();
}

static sinanbox_t ultrasonic_init(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(0);
    internal_ultrasonic_init(&default_ultrasonic_config);
    return NANBOX_OFUNDEF();
}

static sinanbox_t ultrasonic_measure_cm(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(1);
    int max_distance = nanboxToInt(argv[0]);
    uint32_t res;
    internal_ultrasonic_measure_cm(&default_ultrasonic_config, max_distance, &res);
    return wrap_integer(res);
}

static sinanbox_t esp_wait(uint8_t argc, sinanbox_t *argv)
{
    CHECK_ARGS(1);
    int duration = nanboxToInt(argv[0]);
    vTaskDelay(duration / portTICK_PERIOD_MS);
    if (DBG) fprintf(stderr, "waited %dms\n", duration);
    return NANBOX_OFUNDEF();
}

const sivmfnptr_t internals[] = {init_led,
                                 led_on,
                                 led_off,
                                 led_fade,
                                 color_sensor_init,
                                 color_get_rgbc_raw,
                                 color_get_rgb,
                                 color_get_lux,
                                 color_get_color_temperature,
                                 motor_init,
                                 motor_full_steps,
                                 imu_init,
                                 imu_get_pitch,
                                 imu_get_yaw,
                                 imu_get_roll,
                                 imu_set_zero_angle,
                                 imu_get_angle,
                                 ultrasonic_init,
                                 ultrasonic_measure_cm,
                                 esp_wait};

void setupInternals()
{
    sivmfn_vminternals = internals;
    sivmfn_vminternal_count = sizeof(internals) / sizeof(*internals);
}
