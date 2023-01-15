/**
 * Motor controller for the Parallax 360 degree feedback servo.
 * Servo rotation rate is controlled by a PWM pin on the ESP32.
 * Servo position is encoded on a PWM signal read by an ADC on the ESP32.
 * An analog LPF smooths out the postion PWM signal before it gets to the ADC.
 *
 * @author Jim Van Donsel
 * @date 2023/01/14
 *
 */

#include <string>
#include <math.h>
#include "compass.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Pins used
const gpio_num_t SERVO_GPIO_PWM = GPIO_NUM_14;
const ledc_timer_t ledc_timer = LEDC_TIMER_0;
const ledc_channel_t ledc_channel = LEDC_CHANNEL_0;
const adc1_channel_t ADC_CHANNEL= ADC1_CHANNEL_4;

// PWM frequency
const int FREQ_HZ = 50;
// Period in US
const int PERIOD_US = 1.0/FREQ_HZ * 1000 * 1000;
const ledc_timer_bit_t resolution_bit = LEDC_TIMER_10_BIT;
const int resolution = (1 << resolution_bit) - 1;

// Servo PWM pulse widths, from the servo data sheet
const int MIN_WIDTH_US = 1280;
const int MAX_WIDTH_US = 1720;
// The servo has a dead band in this region
const int DEAD_BAND_LOWER_US = 1480;
const int DEAD_BAND_UPPER_US = 1520;
const int DEAD_BAND_CENTER_US = (DEAD_BAND_LOWER_US + DEAD_BAND_UPPER_US)/2;
// Total allowable PWM widths to the servo
const int RANGE_US = MAX_WIDTH_US - MIN_WIDTH_US;

// ADC for reading back position
const int  V_REF_MV = 1100; // mv
static esp_adc_cal_characteristics_t characteristics;
static float V_MAX = 3.0;

// Control loop
const float K = 0.005;
const float RATE_LIMIT = 0.25;

/**
 * Fetch a single voltage value from the ADC.
 */
static float servo_read_adc() {
    uint32_t v_mv;
    esp_adc_cal_get_voltage((adc_channel_t)ADC_CHANNEL, &characteristics, &v_mv);
    return v_mv / 1000.0;
}

/**
 * Initialize the servo ADC.
 */
static void servo_init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF_MV, &characteristics);
}


/**
 * Sets the servo rotation rate between (-1.0, 1.0)
 */
static void servo_set_rate(float rate) {
    if (rate < -RATE_LIMIT ) {
        rate = -RATE_LIMIT;
    } else if (rate > RATE_LIMIT) {
        rate = RATE_LIMIT;
    }

    // Computer the target pulse width in us.
    // Also flip the sign here so that a positive rate is clockwise.
    unsigned width_us = -rate * RANGE_US/2 + DEAD_BAND_CENTER_US;

    // Pulse width converted to a duty cycle
    float duty =  width_us / (float)PERIOD_US;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channel, duty * resolution);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channel);
}

/**
 * Returns the servo position estimate in degrees.
 */
float servo_get_position() {
    return servo_read_adc() * 360 / V_MAX;
}

/*
 * Attempt to slew the servo to the specified orientation.
 * @param target_degrees Target orientation in degrees.
 *
 * The result is a rate sent to the servo. This function
 * needs to be called periodically.
 *
 */
void servo_update_position(float target_degrees) {
    float pos = servo_get_position();
    float delta = target_degrees - pos;

    if (delta > 180) {
        delta = 360 - delta;
    } else if (delta < -180) {
        delta = 380 + delta;
    }

    float rate = K * delta;

    printf("%.2f\t\t%.2f\t\t%.2f\n", target_degrees, pos, rate);

    servo_set_rate(rate);
}


/**
 * Initialize GPIO for PWM for the servo.
 */
bool servo_init() {
    gpio_config_t config;
    config.intr_type = GPIO_INTR_DISABLE;
    config.mode = GPIO_MODE_OUTPUT;
    config.pin_bit_mask = (1 << SERVO_GPIO_PWM);
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&config);

    ledc_timer_config_t timer_cfg_;
    ledc_channel_config_t channel_config_;

    memset(&timer_cfg_, 0, sizeof(timer_cfg_));
    timer_cfg_.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_cfg_.duty_resolution = resolution_bit;
    timer_cfg_.timer_num = ledc_timer;
    timer_cfg_.freq_hz = FREQ_HZ;
    timer_cfg_.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_cfg_);

    channel_config_.gpio_num = SERVO_GPIO_PWM;
    channel_config_.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config_.channel = ledc_channel;
    channel_config_.intr_type = LEDC_INTR_DISABLE;
    channel_config_.timer_sel = ledc_timer;
    channel_config_.duty = 0;
    channel_config_.hpoint = 0; /// FIXME: what is this?

    ledc_channel_config(&channel_config_);

    // Init ADC
    servo_init_adc();

    return true;
}
