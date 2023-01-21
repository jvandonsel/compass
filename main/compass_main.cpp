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
#include "freertos/task.h"
#include "gps.h"
#include "magneto.h"
#include "nvs_flash.h"
#include "servo.h"
#include "util.h"


// GPS location to point to
gps_location_degrees_t home_location = {0.0, 0.0};


// Some canned locations, for convenience of testing
const gps_location_degrees_t WALTHAM_LOC = { 42.36113692296913, -71.24715639837349};
const gps_location_degrees_t BOSTON_LOC  = { 42.35435968945194, -71.0655565086987};
const gps_location_degrees_t NORTH_POLE_LOC = {90.0, 0.0};
const gps_location_degrees_t SOUTH_POLE_LOC = {-90.0, 0.0};


// Non-volatile storage (NVS) keys
const char NVS_KEY_GPS[] = "gps";
const char NVS_KEY_HOME[] = "home";

// How often to write the GPS position to NVS
const unsigned FLASH_WRITE_INTERVAL_SECS = 300;
const unsigned FLASH_WRITE_INTERVAL_TICKS = pdMS_TO_TICKS(FLASH_WRITE_INTERVAL_SECS * 1000);

// Top level loop period
const int LOOP_SLEEP_MS = 250;
const int LOOP_SLEEP_TICKS = pdMS_TO_TICKS(LOOP_SLEEP_MS);

// To log or not to log
bool do_logging = false;
// Last known good GPS location (may be one saved to flash)
gps_location_degrees_t last_known_good_gps_location;
// Current GPS location in use
gps_location_degrees_t current_gps_location;
// Compass heading of this device
compass_degrees_t heading;

/**
 * Top-level loop.
 */
void controllerTask() {
    // Try to read any saved GPS position as a starting point. This allows for a
    // faster startup if the user powers it up in the previous location. Of course,
    // if the device has moved since the last GPS lock, then the position and thus
    // the pointer direction will be wrong for a while.

    // last_known_good_gps_location = read_from_nvs(NVS_KEY_GPS);
    // std::cout << "Read saved GPS " << last_known_good_gps_location.toString() << "from NVS" << std::endl;

    // Read our home (target) location from NVS
    home_location = read_from_nvs(NVS_KEY_HOME);
    std::cout << "Read saved home " << home_location.toString() << "from NVS" << std::endl;

    unsigned last_flash_write_ticks = 0;

    while (true) {
        const unsigned ticks = xTaskGetTickCount();

        // Read sensors
        heading = magneto_read();
        current_gps_location = gps_read();

        if (!current_gps_location.isValid()) {
            // No good GPS reading this time. Use our last known good one.
            current_gps_location = last_known_good_gps_location;
        } else {
            // Save this GPS reading as our last known good one.
            last_known_good_gps_location = current_gps_location;

            // And save to non-volatile storage.
            // But we don't want to continually write to flash every loop, so
            // only write every N seconds.
            if (ticks - last_flash_write_ticks > FLASH_WRITE_INTERVAL_TICKS || last_flash_write_ticks == 0) {
                save_to_nvs(NVS_KEY_GPS, current_gps_location);
                last_flash_write_ticks = ticks;
            }
        }

        if (!current_gps_location.isValid()) {
            // No GPS position yet.
            // Drive the servo continuously at a low rate while we wait for GPS to sync.
            servo_drive_slow();
            relative_degrees_t servo_ps = servo_get_position();

            if (do_logging) {
                printf(
                    "[%u]\t"
                    "Mag True: %d\t"
                    "Srv Pos: %d\t\n",
                    ticks,
                    heading,
                    servo_ps);
            }
        } else {
            // We have a valid GPS position (either live or saved)

            // Compute heading from our GPS position to the target lat/long
            const compass_degrees_t target_compass_bearing = compute_bearing(current_gps_location, home_location);

            // Compute next servo position, based on heading
            relative_degrees_t target_servo_position = target_compass_bearing - heading;
            if (target_servo_position < 0) {
                target_servo_position += 360;
            }
            // Update servo
            const servo_status_t servo_status = servo_update(target_servo_position);

            // Not required, but interesting, maybe.
            const float dist_miles = compute_distance_miles(current_gps_location, home_location);

            if (do_logging) {
                printf(
                    "[%u]\t"
                    "GPS: %18s  "
                    "Target: %18s   "
                    "Mag True: %3d   "
                    "Bearing: %3d   "
                    "Srv Target: %3d   "
                    "Srv Pos: %3d "
                    "Srv Rate: %8.3f    "
                    "Dist: %.0f mi\n",
                    ticks,
                    current_gps_location.toString().c_str(),
                    home_location.toString().c_str(),
                    heading,
                    target_compass_bearing, target_servo_position,
                    servo_status.relative_pos, servo_status.rate, dist_miles);
            }
        }

        vTaskDelay(LOOP_SLEEP_TICKS);
    }  // end while
}  // end controller Task

/**
 * Set the location to the given value
 */
static int set_canned_location(gps_location_degrees_t loc) {

    home_location = loc;
    std::cout << "Setting home location to: " << home_location.toString() << std::endl;

    // Save new location to flash.
    save_to_nvs(NVS_KEY_HOME, home_location);
    return 0;
}

/**
 * Console command handler for reading GPS location from NVS
 */
static int read_gps_location(int argc, char **argv) {
    last_known_good_gps_location = read_from_nvs(NVS_KEY_GPS);
    std::cout << "Read saved GPS " << last_known_good_gps_location.toString() << "from NVS" << std::endl;
    return 0;
}

/**
 * Console command handler for saving GPS location to NVS
 */
static int save_gps_location(int argc, char **argv) {
    save_to_nvs(NVS_KEY_GPS, current_gps_location);
    std::cout << "Saving GPS " << current_gps_location.toString() << "to NVS" << std::endl;
    return 0;
}

/**
 * Console command handler for setting the home location.
 */
static int set_home_location(int argc, char **argv) {
    if (argc != 3) {
        std::cout << "Invalid command . Usage is 'home <latitude> <longitude>'" << std::endl;
        std::cout << "(where latitude is positive in the N hemisphere, and longitude is negative in the W hemisphere)" << std::endl;
        return 0;
    }

    home_location.latitude = atof(argv[1]);
    home_location.longitude = atof(argv[1]);
    std::cout << "Setting home location to: " << home_location.toString() << std::endl;

    // Save new location to flash.
    save_to_nvs(NVS_KEY_HOME, home_location);
    return 0;
}

/**
 * Console command handler for printing status
 */
static int print_status(int argc, char **argv) {
    do_logging = false;
    std::cout << "Home location: " << home_location.toString() << std::endl;
    std::cout << "Current GPS lat/long: " << current_gps_location.toString() << std::endl;
    std::cout << "Last known good GPS lat/long: " << last_known_good_gps_location.toString() << std::endl;
    std::cout << "Magnetic Heading: " << heading << " degrees" << std::endl;
    return 0;
}

/**
 * Console command handler for toggling logging on/off
 */
static int toggle_logging(int argc, char **argv) {
    do_logging = !do_logging;
    return 0;
}

/**
 * Console command handler for printing the about mesage
 */
static int about(int argc, char **argv) {
    std::cout << "Magic Compass" << std::endl;
    std::cout << "Designed and built by Jim Van Donsel, January 2023." << std::endl;
    return 0;
}

/**
 * Main
 */
extern "C" void app_main() {
    nvs_flash_init();
    magneto_init();
    gps_init();
    servo_init();

    // Set up interactive console
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "compass>";
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    // Add console commands
    const esp_console_cmd_t status_cmd = {
        .command = "status",
        .help = "Show status",
        .hint = NULL,
        .func = &print_status,
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&status_cmd));

    const esp_console_cmd_t logging_cmd = {
        .command = "log",
        .help = "Toggle logging",
        .hint = NULL,
        .func = &toggle_logging,
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&logging_cmd));

    const esp_console_cmd_t about_cmd = {
        .command = "about",
        .help = "About this device",
        .hint = NULL,
        .func = &about,
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&about_cmd));

    const esp_console_cmd_t read_gps_cmd = {
        .command = "read",
        .help = "Read saved GPS position from NVS",
        .hint = NULL,
        .func = &read_gps_location,
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&read_gps_cmd));
    
    const esp_console_cmd_t save_gps_cmd = {
        .command = "save",
        .help = "Save GPS position to NVS",
        .hint = NULL,
        .func = &save_gps_location,
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&save_gps_cmd));

    const esp_console_cmd_t home_loc_cmd = {
        .command = "home",
        .help = "Set home latitude, longitude",
        .hint = NULL,
        .func = &set_home_location,
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&home_loc_cmd));

    const esp_console_cmd_t boston_loc_cmd = {
        .command = "boston",
        .help = "Set home location to Boston, MA",
        .hint = NULL,
        .func = ([](int argc, char** argv){set_canned_location(BOSTON_LOC); return 0;}),
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&boston_loc_cmd));

    const esp_console_cmd_t waltham_loc_cmd = {
        .command = "waltham",
        .help = "Set home location to Waltham, MA",
        .hint = NULL,
        .func = ([](int argc, char** argv){set_canned_location(WALTHAM_LOC); return 0;}),
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&waltham_loc_cmd));

    const esp_console_cmd_t north_loc_cmd = {
        .command = "north",
        .help = "Set home location to the North Pole",
        .hint = NULL,
        .func = ([](int argc, char** argv){set_canned_location(NORTH_POLE_LOC); return 0;}),
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&north_loc_cmd));

    const esp_console_cmd_t south_loc_cmd = {
        .command = "south",
        .help = "Set home location to the South Pole",
        .hint = NULL,
        .func = ([](int argc, char** argv){set_canned_location(SOUTH_POLE_LOC); return 0;}),
        .argtable = nullptr};
    ESP_ERROR_CHECK(esp_console_cmd_register(&south_loc_cmd));

    // Start the console
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // Run the controller loop
    // Does not return
    controllerTask();
}
