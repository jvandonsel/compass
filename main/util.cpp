/**
 * Misc utility functions for the Magic Compass.
 * @author: J Van Donsel
 * @date: 2023/01/16
 */


#include "compass.h"
#include "util.h"
#include <math.h>
#include "nvs_flash.h"
#include <iostream>

// Flash storage namespace
static const char NAMESPACE[] = "storage";

/**
 * Compute a bearing in degrees between two points.
 * @param here Start position (lat/long, degrees)
 * @param there End position (lat/long, degrees)
 * @return Integer compass degrees for a bearing between here and there
 *
 * See http://www.movable-type.co.uk/scripts/latlong.html
 */
compass_degrees_t compute_bearing(const gps_location_degrees_t here, const gps_location_degrees_t there) {

    const gps_location_radians_t here_rad = here.asRadians();
    const gps_location_radians_t there_rad = there.asRadians();

    const float delta_long = there_rad.longitude - here_rad.longitude;
    const float y = cos(there_rad.latitude) * sin(delta_long);
    const float x = cos(here_rad.latitude) * sin(there_rad.latitude) - sin(here_rad.latitude) * cos(there_rad.latitude) * cos(delta_long);
    const float bearing = atan2(y, x);
    compass_degrees_t bearing_degrees  = rad_to_deg(bearing);
    if (bearing_degrees  < 0) bearing_degrees += 360;
    return bearing_degrees;
}

/**
 * Compute the distance in miles between two points.
 * @param here Lat/long, degrees
 * @param there Lat/long, degrees
 * @return Distance in miles
 * 
 * See http://www.movable-type.co.uk/scripts/latlong.html
 */
float compute_distance_miles(const gps_location_degrees_t here, const gps_location_degrees_t there) {
    const gps_location_radians_t here_rad = here.asRadians();
    const gps_location_radians_t there_rad = there.asRadians();

    const float delta_lat = there_rad.latitude - here_rad.latitude;
    const float delta_long = there_rad.longitude - here_rad.longitude;
    
    float a = sqr(sin(delta_lat/2.0)) + cos(here_rad.latitude) * cos(there_rad.latitude)*sqr(sin(delta_long/2.0));
    float c = 2 * atan2(sqrt(a), sqrt(1-a));

    const float EARTH_RADIUS_MILES = 3961;
    return EARTH_RADIUS_MILES * c;
}

/**
 * Save an integer to non-volatile flash with the given key.
 */
void save_int_to_nvs(const char* key, int32_t value) {
    printf("Saving %s=%d\n", key, value);
    nvs_handle_t h;
    int err = nvs_open(NAMESPACE, NVS_READWRITE, &h);
    err = nvs_set_i32(h, key, value);
    if (err != ESP_OK) {
        fprintf(stderr, "Failed to write key %s\n", key);
    }
    nvs_commit(h);
    nvs_close(h);
}

/**
 * Read an integer from non-volatile flash using the given key.
 */
int32_t read_int_from_nvs(const char* key) {
    nvs_handle_t h;
    int err = nvs_open(NAMESPACE, NVS_READWRITE, &h);
    int32_t value = 0;
    err = nvs_get_i32(h, key, &value);
    if (err != ESP_OK) {
        fprintf(stderr, "Failed to read key %s\n", key);
    }
    return value;
}

/**
 * Save a latitude/longitude structure to non-volatile flash, with the given key.
 */
void save_lat_long_to_nvs(const char* key, gps_location_degrees_t lat_long) {
    printf("Saving lat=%f long=%f to NVS\n", lat_long.latitude, lat_long.longitude);
    nvs_handle_t h;
    int err = nvs_open(NAMESPACE, NVS_READWRITE, &h);
    err = nvs_set_blob(h, key, &lat_long, sizeof(lat_long));
    if (err != ESP_OK) {
        fprintf(stderr, "Failed to write lat/long blob!, error=%s\n", esp_err_to_name(err));
    }
    nvs_commit(h);
    nvs_close(h);
}


/**
 * Read a latitude/longitude structure from non-volatile flash with the given key.
 * @return lat/long structure, which contains zeros if nothing could be read.
 */
gps_location_degrees_t read_lat_long_from_nvs(const char* key) {
    nvs_handle_t h;
    int err = nvs_open(NAMESPACE, NVS_READONLY, &h);
    gps_location_degrees_t lat_long = {0.0, 0.0};
    size_t size =sizeof(gps_location_degrees_t);
    err = nvs_get_blob(h, key, &lat_long, &size);
    if (err != ESP_OK) {
        fprintf(stderr, "Failed to read lat/long blob!, error=%s\n", esp_err_to_name(err));
    }
    nvs_close(h);
    return lat_long;
}

