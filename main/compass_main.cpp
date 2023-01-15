/*
 * Magic Compass for ESP32
 *
 * Author: J Van Donsel
 * Date: 12/29/2022
 */

#include <math.h>
#include <stdio.h>

#include <iostream>
#include <string>

#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "gps.h"
#include "http.h"
#include "magneto.h"
#include "nvs_flash.h"
#include "servo.h"
#include "util.h"

bool do_logging = false;

// This is Brattleboro VT, chosen randomly.
lat_long_t target_position = {42.865807639823984, -72.58788475153355};

// Non-volatile storage keys
const char NVS_KEY_GPS[] = "gps";
const char NVS_KEY_HOME[] = "home";

/**
 * Control loop.
 */
void controllerTask() {
    // Top level loop period
    const int LOOP_SLEEP_MS = 250;
    const int LOOP_SLEEP_TICKS = pdMS_TO_TICKS(LOOP_SLEEP_MS);

    // Try to read any saved GPS position as a starting point
    lat_long_t last_known_good_gps = read_from_nvs(NVS_KEY_GPS);
    printf("Read saved GPS lat=%f long=%f from NVS\n", last_known_good_gps.latitude, last_known_good_gps.longitude);

    // How often to write the GPS position to NVS
    const unsigned FLASH_WRITE_INTERVAL_SECS = 300;
    const unsigned FLASH_WRITE_INTERVAL_TICKS = pdMS_TO_TICKS(FLASH_WRITE_INTERVAL_SECS * 1000);
    unsigned last_flash_write_ticks = 0;

    // jvd test
    // save_to_nvs(NVS_KEY_HOME, target_position);
    lat_long_t temp = read_from_nvs(NVS_KEY_HOME);
    printf("Read saved home lat=%f long=%f from NVS\n", temp.latitude, temp.longitude);

    while (true) {
        const int ticks = xTaskGetTickCount();

        // Read sensors
        const compass_degrees_t heading = magneto_read();
        lat_long_t lat_long = gps_read();

        if (!lat_long.isValid()) {
            // No good GPS reading this time. Use our last known good one.
            lat_long = last_known_good_gps;
        } else {
            // Save this GPS reading as our last known good one.
            last_known_good_gps = lat_long;

            // And save to non-volatile storage.
            // But we don't want to continually write to flash every loop, so
            // only write every N seconds.
            if (ticks - last_flash_write_ticks > FLASH_WRITE_INTERVAL_TICKS || last_flash_write_ticks == 0) {
                save_to_nvs(NVS_KEY_GPS, lat_long);
                last_flash_write_ticks = ticks;
            }
        }

        if (!lat_long.isValid()) {
            // No GPS position yet.
            // Drive the servo continuously at a low rate while we wait for GPS to sync.
            servo_drive_slow();
            relative_degrees_t servo_ps = servo_get_position();

            if (do_logging) {
                printf(
                    "[%u]\t"
                    "Lat: ?\t"
                    "Long: ?\t"
                    "Mag True: %d\t"
                    "Srv Pos: %d\t\n",
                    ticks,
                    heading,
                    servo_ps);
            }
        } else {
            // We have a valid GPS position (either live or saved)

            // Compute heading from our GPS position to the target lat/long
            const compass_degrees_t target_compass_bearing = compute_heading(lat_long, target_position);

            // Compute next servo position, based on heading
            relative_degrees_t target_servo_position = target_compass_bearing - heading;
            if (target_servo_position < 0) {
                target_servo_position += 360;
            }
            // Update servo
            const servo_status_t servo_status = servo_update(target_servo_position);

            // Not required, but interesting, maybe.
            const float dist_miles = compute_distance_miles(lat_long, target_position);

            if (do_logging) {
                printf(
                    "[%u]\t"
                    "GPS: (%.4f,%.4f)\t"
                    "Target: (%.4f,%.4f)\t"
                    "Mag True: %d\t"
                    "Mag Target: %d\t"
                    "Srv Target: %d\t"
                    "Srv Pos: %d\t"
                    "Srv Rate: %.3f\t\t"
                    "Dist: %.0f\t\n",
                    ticks,
                    lat_long.latitude, lat_long.longitude,
                    target_position.latitude, target_position.longitude,
                    heading,
                    target_compass_bearing, target_servo_position,
                    servo_status.relative_pos, servo_status.rate, dist_miles);
            }
        }

        vTaskDelay(LOOP_SLEEP_TICKS);

    }  // end while
}

static int set_home_location(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "Invalid command . Usage is 'home <latitude> <longitude>'" << std::endl;
        return 0;
    }

    lat_long_t loc;
    loc.latitude = atof(argv[1]);
    loc.longitude = atof(argv[2]);
    std::cout << "Setting home location to latitude=" << loc.latitude << " longitude=" << loc.longitude << std::endl;
    save_to_nvs(NVS_KEY_HOME, loc);
    return 0;
}

static int print_status(int argc, char **argv) {
    do_logging = false;
    std::cout << "Home location latitude " << target_position.latitude << std::endl;
    std::cout << "Home location longitude " << target_position.longitude << std::endl;
    std::cout << std::endl;
    return 0;
}

static int toggle_logging(int argc, char **argv) {
    do_logging = !do_logging;
    return 0;
}

/**
 * Main!
 */
extern "C" void app_main() {
    nvs_flash_init();
    // start_webserver();
    magneto_init();
    gps_init();
    servo_init();

    // Set up interactive console
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "compass>";
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    const esp_console_cmd_t status_cmd = {
        .command = "status",
        .help = "Show status",
        .hint = NULL,
        .func = &print_status,
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&status_cmd));

    const esp_console_cmd_t logging_cmd = {
        .command = "logging",
        .help = "Toggle logging",
        .hint = NULL,
        .func = &toggle_logging,
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&logging_cmd));

    
    const esp_console_cmd_t home_loc_cmd = {
        .command = "home",
        .help = "Set home latitude, longitude",
        .hint = NULL,
        .func = &set_home_location,
        .argtable = nullptr
        };
    ESP_ERROR_CHECK(esp_console_cmd_register(&home_loc_cmd));

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // Does not return
    controllerTask();
}
