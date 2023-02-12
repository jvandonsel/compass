/*
 * LSM303AGR Magnetometer for Magic Compass.
 * Interface is I2C
 *
 * My LSM303AGR unit arrived calibrated and the x and y magnetic components seemed to be working
 * well. Then at some point during my testing it went completely wonky and I needed to determine
 * some correction offsets on x and y. These are applied here as MAG_CAL_OFFSET_X and
 * MAG_CAL_OFFSET_Y. These values will certainly need to be changed for a different unit.
 *
 * Author: J Van Donsel
 * Date: 12/29/2022
 */
#include <stdio.h>

#include <string>
#include <math.h>
#include "compass.h"
#include "util.h"
#include "driver/i2c.h"

/*
  I2C
*/
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_23
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 200000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

// Device addresses
#define LSM303_ACCEL_SENSOR_ADDR 0x19
#define LSM303_MAG_SENSOR_ADDR 0x1E

/*
  Registers
*/
#define LSM303_CTRL_REG1_ACCEL 0x20

// Accelerometer
#define LSM303_WHO_AM_I_A_REG_ADDR 0x0F
#define LSM303_OUTX_L_REG_ACCEL 0x28
#define LSM303_OUTX_H_REG_ACCEL 0x29
#define LSM303_OUTY_L_REG_ACCEL 0x2A
#define LSM303_OUTY_H_REG_ACCEL 0x2B
#define LSM303_OUTZ_L_REG_ACCEL 0x2C
#define LSM303_OUTZ_H_REG_ACCEL 0x2D

// Magneto
#define LSM303_OFFSETX_L_REG_MAG 0x45
#define LSM303_OFFSETX_H_REG_MAG 0x46
#define LSM303_OFFSETY_L_REG_MAG 0x47
#define LSM303_OFFSETY_H_REG_MAG 0x48
#define LSM303_WHO_AM_I_M_REG_ADDR 0x4F
#define LSM303_CFG_REG_A_MAG  0x60
#define LSM303_STATUS_REG_MAG 0x67
#define LSM303_OUTX_L_REG_MAG 0x68
#define LSM303_OUTX_H_REG_MAG 0x69
#define LSM303_OUTY_L_REG_MAG 0x6A
#define LSM303_OUTY_H_REG_MAG 0x6B
#define LSM303_OUTZ_L_REG_MAG 0x6C
#define LSM303_OUTZ_H_REG_MAG 0x6D


// Emperically determined calibration constants, specific to my (possibly uncalibrated) device. Calculated by determining the middle x
// and y raw values while rotating the device around the z axis in the absence of any extraneous magnetic fields.
// TODO: put this in NVS
const int16_t MAG_CAL_OFFSET_X = 200;
const int16_t MAG_CAL_OFFSET_Y = -130;

/**
 * @brief Read a sequence of bytes from a LSM303 MAG sensor registers
 */
static esp_err_t lsm303_mag_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, LSM303_MAG_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a LSM303 sensor MAG register
 */
static esp_err_t lsm303_mag_register_write_byte(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LSM303_MAG_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}


/**
 * @brief Read a sequence of bytes from a LSM303 ACC sensor registers
 */
static esp_err_t lsm303_accel_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, LSM303_ACCEL_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a LSM303 sensor ACC register
 */
static esp_err_t lsm303_accel_register_write_byte(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LSM303_ACCEL_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ},
        .clk_flags = 0};

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


bool magneto_init() {
    ESP_ERROR_CHECK(i2c_master_init());
    printf("I2C initialized successfully\n");


    // Enable continuous mode
    ESP_ERROR_CHECK(lsm303_mag_register_write_byte(LSM303_CFG_REG_A_MAG, 0x00));

    // Accelerometer
    ESP_ERROR_CHECK(lsm303_accel_register_write_byte(LSM303_CTRL_REG1_ACCEL, 0x57));

    // Sanity check
    uint8_t data[2];
    ESP_ERROR_CHECK(lsm303_mag_register_read(LSM303_WHO_AM_I_M_REG_ADDR, data, 1));
    printf("WHO_AM_I = %X\n", data[0]);
    return data[0] == 0x40;
}

/**
 *   Dump out statistics about the mag readings
 *   @param magneto_x X mag component
 *   @param magneto_y Y mag component
 *   @param heading Computed heading in degrees
 */
void print_mag_details(int16_t magneto_x, int16_t magneto_y, float heading) {
    // Device calibration stuff
    static int16_t max_x = INT16_MIN;
    static int16_t min_x = INT16_MAX;
    static int16_t max_y = INT16_MIN;
    static int16_t min_y = INT16_MAX;

    if (magneto_x > max_x) max_x = magneto_x;
    if (magneto_y > max_y) max_y = magneto_y;

    if (magneto_x < min_x) min_x = magneto_x;
    if (magneto_y < min_y) min_y = magneto_y;
    
    static int mid_x;
    mid_x  = (max_x + min_x)/2;
    static int mid_y;
    mid_y = (max_y + min_y)/2;

    printf("Read raw=(%d, %d) mid=(%d, %d) xrange=(%d, %d) xdelta=%d yrange=(%d, %d) ydelta=%d heading=%f\n",
           magneto_x, magneto_y,
           mid_x, mid_y,
           min_x, max_x,
           max_x - min_x,
           min_y, max_y,
           max_y - min_y,
           heading);
}

/**
 * Read the magnetometer and return the heading in degrees.
 * @return Heading in degrees
 */
compass_degrees_t magneto_read() {

    // Read x and y magnetic components
    int16_t magneto_x, magneto_y;
    ESP_ERROR_CHECK(lsm303_mag_register_read(LSM303_OUTX_L_REG_MAG, (uint8_t*)&magneto_x,  2));
    ESP_ERROR_CHECK(lsm303_mag_register_read(LSM303_OUTY_L_REG_MAG, (uint8_t*)&magneto_y,  2));
/*
  int16_t accel_x, accel_y, accel_z;
  ESP_ERROR_CHECK(lsm303_accel_register_read(LSM303_OUTX_L_REG_ACCEL, (uint8_t*)&accel_x,  2));
  ESP_ERROR_CHECK(lsm303_accel_register_read(LSM303_OUTY_L_REG_ACCEL, (uint8_t*)&accel_y,  2));
  ESP_ERROR_CHECK(lsm303_accel_register_read(LSM303_OUTZ_L_REG_ACCEL, (uint8_t*)&accel_z,  2));
*/

    // Apply our calibration offsets
    magneto_x += MAG_CAL_OFFSET_X;
    magneto_y += MAG_CAL_OFFSET_Y;

    // Calculate the angle of the x,y vector
    float heading = -(atan2((float)magneto_y, (float)magneto_x) * 180) / M_PI;

    if (heading < 0) {
        heading = 360 + heading;
    }

    // Uncomment this to see raw, min, and max magnetic readings, for calibration puposes
    ///print_mag_details(magneto_x, magneto_y, heading);
    
    return (compass_degrees_t)heading;

}
