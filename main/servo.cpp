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
#include "servo.h"



// Pins used
const gpio_num_t SERVO_GPIO_PWM = GPIO_NUM_14;
const ledc_timer_t ledc_timer = LEDC_TIMER_0;
const ledc_channel_t ledc_channel = LEDC_CHANNEL_0;
const adc1_channel_t ADC_CHANNEL= ADC1_CHANNEL_4;

// PWM frequency
const int PWM_FREQ_HZ = 50;
const int PWM_PERIOD_US = 1.0/PWM_FREQ_HZ * 1000 * 1000;
const ledc_timer_bit_t RESOLUTION_BIT = LEDC_TIMER_10_BIT;
const int RESOLUTION = (1 << RESOLUTION_BIT) - 1;

// Allowable servo PWM pulse widths, from the servo data sheet
const int MIN_WIDTH_US = 1280;
const int MAX_WIDTH_US = 1720;
const int RANGE_US = MAX_WIDTH_US - MIN_WIDTH_US;

// The servo has a dead band in this region
const unsigned DEAD_BAND_LOWER_US = 1480;
const unsigned DEAD_BAND_UPPER_US = 1520;
const unsigned DEAD_BAND_CENTER_US = (DEAD_BAND_LOWER_US + DEAD_BAND_UPPER_US)/2;

// Minimum viable pulse width that causes movement
const unsigned MINIMUM_SPEED_US = DEAD_BAND_UPPER_US + 30;

// ADC for reading back position
const int  V_REF_MV = 1100; // mv
static esp_adc_cal_characteristics_t characteristics;
static float V_MAX = 3.0;

// Control loop
// Gain
const float K = 0.005;

// Fastest rate we will allow to be commanded
const rotation_rate_t RATE_LIMIT = 0.25;

/**
 * Fetch a single voltage value from the ADC.
 */
static float read_adc() {
    uint32_t v_mv;
    esp_adc_cal_get_voltage((adc_channel_t)ADC_CHANNEL, &characteristics, &v_mv);
    return v_mv / 1000.0;
}

/**
 * Initialize the servo ADC.
 */
static void init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF_MV, &characteristics);
}


/**
* Convert a rotation rate [-1.0, 1.0] to a pulse width in uS
*/
static unsigned rate_to_pulse_width(rotation_rate_t rate) {
    if (rate < -RATE_LIMIT ) {
        rate = -RATE_LIMIT;
    } else if (rate > RATE_LIMIT) {
        rate = RATE_LIMIT;
    }

    // Computer the target pulse width in us.
    // Also flip the sign here so that a positive rate is clockwise.
    return -rate * RANGE_US/2 + DEAD_BAND_CENTER_US;
}

/**
* Set the PWM pulse width in uS
*/
static void set_pulse_width(unsigned width_us) {
    // Pulse width converted to a duty cycle
    float duty =  width_us / (float)PWM_PERIOD_US;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_channel, duty * RESOLUTION);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_channel);
}

/**
 * Sets the servo rotation rate between (-1.0, 1.0)
 */
static void set_rate(rotation_rate_t rate) {
    set_pulse_width(rate_to_pulse_width(rate));
}


/*
-------------------- Public Functions ----------------------------------
*/

/**
 * Returns the servo position estimate in degrees.
 */
relative_degrees_t servo_get_position() {
    return read_adc() * 360 / V_MAX;
}

/**
 * Drive the servo continually at the lowest rate
 */
void servo_drive_slow() {
    set_pulse_width(MINIMUM_SPEED_US);
}

/**
 * Attempt to slew the servo to the specified orientation.
 * The result is a rate sent to the servo. This function
 * needs to be called periodically, or else the servo
 * will go right past the target.
 *
 * @param target_degrees Target orientation in degrees.
 * @return Position and rate
 *

 */
servo_status_t servo_update(relative_degrees_t target_degrees) {
    relative_degrees_t pos = servo_get_position();
    relative_degrees_t delta =  target_degrees - pos;

    if (delta > 180) {
        delta = 360 - delta;
    } else if (delta < -180) {
        delta = 380 + delta;
    }

    rotation_rate_t rate = K * delta;
    
    set_rate(rate);
    
    servo_status_t status;
    status.rate = rate;
    status.relative_pos = pos;

    return status;
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
    timer_cfg_.duty_resolution = RESOLUTION_BIT;
    timer_cfg_.timer_num = ledc_timer;
    timer_cfg_.freq_hz = PWM_FREQ_HZ;
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
    init_adc();

    return true;
}
