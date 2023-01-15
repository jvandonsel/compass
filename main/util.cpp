/**
 * Misc utility functions for the Magic Compass.
 * @author: J Van Donsel
 * @date: 2023/01/16
 */


#include "compass.h"
#include "util.h"
#include <math.h>
#include "nvs_flash.h"

/**
 * Compute a bearing between two points.
 * @param here Start position (lat/long, degrees)
 * @param there End position (lat/long, degrees)
 * @return Integer compass degress for a bearing between here and there
 * https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
 */
compass_degrees_t compute_heading(const lat_long_t here, const lat_long_t there) {

    const lat_long_t here_rad = here.asRadians();
    const lat_long_t there_rad = there.asRadians();

    const float delta_lat = there_rad.latitude - here_rad.latitude;
    const float x = cos(there_rad.longitude) * sin(delta_lat);
    const float y = cos(here_rad.longitude) * sin(there_rad.longitude) - sin(here_rad.longitude) * cos(there_rad.longitude) * cos(delta_lat);
    const float bearing = atan2(x, y);
    return (compass_degrees_t)rad_to_deg(bearing);
}

/**
 * Compute the distance in miles between two points.
 * @param here Lat/long, degrees
 * @param there Lat/long, degrees
 * @return Distance in miles
 */
float compute_distance_miles(const lat_long_t here, const lat_long_t there) {
    const lat_long_t here_rad = here.asRadians();
    const lat_long_t there_rad = there.asRadians();

    const float delta_lat = there_rad.latitude - here_rad.latitude;
    const float delta_long = there_rad.longitude - here_rad.longitude;
    
    float a = sqr(sin(delta_lat)) + cos(here_rad.latitude) * cos(there_rad.latitude)*sqr(sin(delta_long/2));
    float c = 2 * atan2(sqrt(a), sqrt(1-a));

    const float EARTH_RADIUS_MILES = 3961;
    return EARTH_RADIUS_MILES * c;
}

/**
 * Save a latitude/longitude structure to non-volatile flash
 */
void save_to_nvs(const char* key, lat_long_t lat_long) {
    printf("Saving lat=%f long=%f to NVS\n", lat_long.latitude, lat_long.longitude);
    nvs_handle_t h;
    int err = nvs_open("storage", NVS_READWRITE, &h);
    err = nvs_set_blob(h, key, &lat_long, sizeof(lat_long));
    if (err != ESP_OK) {
        fprintf(stderr, "Failed to write lat/long blob!, error=%s\n", esp_err_to_name(err));
    }
    nvs_commit(h);
    nvs_close(h);
}


/**
 * Read a latitude/longitude structure from non-volatile flash.
 * @return lat/long structure, which contains zeros if nothing could be read.
 */
lat_long_t read_from_nvs(const char* key) {
    nvs_handle_t h;
    int err = nvs_open("storage", NVS_READONLY, &h);
    lat_long_t lat_long = {0.0, 0.0};
    size_t size =sizeof(lat_long_t);
    err = nvs_get_blob(h, key, &lat_long, &size);
    if (err != ESP_OK) {
        fprintf(stderr, "Failed to read lat/long blob!, error=%s\n", esp_err_to_name(err));
    }
    nvs_close(h);
    return lat_long;
}

