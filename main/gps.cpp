/**
 * Magic Compass interface to Adafruit GPS Featherwing board with a MTK3339 GPS chipset.
 * Communication to GPS board is via a serial UART at 9600 baud.
 * @author Jim Van Donsel
 * @date 2023/01/14
 *           
*/

#include "gps.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

const int BUF_SIZE = 1024;
static char data[BUF_SIZE];
static const uart_port_t uart_num = UART_NUM_2;
static uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_APB
};
static const int intr_alloc_flags = 0;

/**
 *   Serial commands to the GPS board
 */
// Query firmware version
#define PMTK_Q_RELEASE "$PMTK605*31\r\n"

/**
 * Open serial port for GPS chip.
 * @return true on success
 */
bool gps_init() {
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));    
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    return true;
}

/**
 * Convert DDMM.MM or DDDMM.MM to degrees.
 * @return degrees
 */
static float dm_to_degrees(float dm) {
    const int degrees = dm / 100;
    const float minutes = dm - degrees * 100;
    return degrees + minutes/60.0;
}


/**
 * Parse the GPS chip serial output to get lat/long.
 * @return lat/long or 0.0/0.0 if no fix.
 * 
 *    GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx
 *    $GNGGA,200452.000,4221.6768,N,07114.8309,W,1,07,1.20,46.9,M,-33.7,M,,*7F
 *        hhmmss.ss = UTC of position
 *        llll.ll = latitude of position (DDMM.MM)
 *        a = N or S
 *        yyyyy.yy = Longitude of position (DDDMM.MM)
 *        a = E or W
 *        x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
 *        xx = number of satellites in use
 *        x.x = horizontal dilution of precision
 *        x.x = Antenna altitude above mean-sea-level
 *        M = units of antenna altitude, meters
 *        x.x = Geoidal separation
 *        M = units of geoidal separation, meters
 *        x.x = Age of Differential GPS data (seconds)
 *        xxxx = Differential reference station ID
 *
 */
static gps_location_degrees_t parse_lat_long(char* s) {
    gps_location_degrees_t result = {0.0,0.0};

    char* p = strstr(s, "GNGGA");
    if (p) {
        char* tok = strtok(p, ",");
        int n = 0;
        while (tok != nullptr) {
            // Extract latitude
            if (n == 2) {
                const float dm = atof(tok);
                result.latitude = dm_to_degrees(dm);
            } else if (n == 3) {
                // Defining S hemisphere as negative
                if (!strcmp(tok, "S")) {
                    result.latitude *= -1;
                }
            } else if (n == 4) {
                const float dm = atof(tok);
                result.longitude = dm_to_degrees(dm);
            } else if (n == 5) {
                // Defining W hemisphere as negative
                if (!strcmp(tok, "W")) {
                    result.longitude *= -1;
                }
                // done
                break;
            }
            tok = strtok(nullptr, ",");
            n++;
        }
    }

    return result;
}


/**
*   Read a blob of serial data from the GPS board, parse it, and return a lat/long structure.
*   Note that this function may not produce a valid lat/long value every time it's called - it
*   depends on the asynchronous data being sent by the GPS board.
*   
*   @return lat/long, or if no valid location data can be found, returns lat/long of 0.00,0.00
*/
gps_location_degrees_t gps_read() {
    gps_location_degrees_t result = {0.0, 0.0};

    const int TIMEOUT_MS = 20;
    const int ticks = pdMS_TO_TICKS(TIMEOUT_MS);

    // Read data from the UART
    const int len = uart_read_bytes(uart_num, data, BUF_SIZE - 1, ticks);
    if (len > 0) {
        return parse_lat_long(data);
    }
    return result;
}
