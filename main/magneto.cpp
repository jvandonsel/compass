/*
 * LSM303AGR Magnetometer for magic compass
 *
 * Author: J Van Donsel
 * Date: 12/29/2022
 */
#include <stdio.h>

#include <string>
#include <math.h>
#include "compass.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_23
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 200000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define LSM303_SENSOR_ADDR 0x1E
#define LSM303_WHO_AM_I_REG_ADDR 0x4F

#define LSM303_CFG_REG_A_M  0x60
#define LSM303_STATUS_REG_M 0x67
#define LSM303_OUTX_L_REG_M 0x68
#define LSM303_OUTX_H_REG_M 0x69
#define LSM303_OUTY_L_REG_M 0x6A
#define LSM303 OUTY_H_REG_M 0x6B
#define LSM303_OUTZ_L_REG_M 0x6C
#define LSM303_OUTZ_H_REG_M 0x6D

/**
 * @brief Read a sequence of bytes from a LSM303 sensor registers
 */
static esp_err_t lsm303_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, LSM303_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a LSM303 sensor register
 */
static esp_err_t lsm303_register_write_byte(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LSM303_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

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


bool init_magneto() {
    ESP_ERROR_CHECK(i2c_master_init());
    printf("I2C initialized successfully\n");


    // Enable continuous mode
    // TODO: Look at filtering settings
    ESP_ERROR_CHECK(lsm303_register_write_byte(LSM303_CFG_REG_A_M, 0x00));

    // Sanity check
    uint8_t data[2];
    ESP_ERROR_CHECK(lsm303_register_read(LSM303_WHO_AM_I_REG_ADDR, data, 1));
    printf("WHO_AM_I = %X\n", data[0]);
    return data[0] == 0x40;
}

float get_heading() {
    int16_t x, y;
    ESP_ERROR_CHECK(lsm303_register_read(LSM303_OUTX_L_REG_M, (uint8_t*)&x,  2));
    ESP_ERROR_CHECK(lsm303_register_read(LSM303_OUTY_L_REG_M, (uint8_t*)&y,  2));

   // Calculate the angle of the vector y,x
    float heading = -(atan2(y, x) * 180) / M_PI;

    if (heading < 0) {
        heading = 360 + heading;
    }
    return heading;
}