#pragma once
#include "esp_system.h"
#include <string.h>
#include <string>

#define PI 3.1415927

// Absolute degrees 0-360 in the global compass frame
// (defined as signed so we can do wrap-arounds properly)
typedef int32_t compass_degrees_t;

// Relative degrees 0-360 in the device's frame
// (defined as signed so we an do wrap-arounds properly)
typedef int32_t relative_degrees_t;

inline float rad_to_deg(float rad) { return rad * 180 / PI; }
inline float deg_to_rad(float deg) { return deg * PI / 180; }

inline float sqr(float a) { return a * a; }

/**
 * GPS lat/long, with ambiguous units.
 */
struct lat_long_t {
    // Positive for N hemisphere, negative for S hemisphere
    float latitude;
    // Positive for E hemisphere, negative for W hemisphere
    float longitude;

    bool isValid() const { return (latitude != 0 || longitude != 0); }

};

/**
 * GPS lat/long, interpreted as radians.
 */
struct gps_location_radians_t : lat_long_t {};

/**
 * GPS lat/long, interpreted as degrees.
 */
struct gps_location_degrees_t : lat_long_t {

    gps_location_radians_t asRadians() const { return {deg_to_rad(latitude), deg_to_rad(longitude)}; }

    std::string toString() const {
        char buff[50];
        snprintf(buff, sizeof(buff), "(%.4f, %.4f)", latitude, longitude);
        return std::string(buff);
    }
};

extern compass_degrees_t compute_bearing(const gps_location_degrees_t here, const gps_location_degrees_t there);
extern float compute_distance_miles(const gps_location_degrees_t here, const gps_location_degrees_t there);
extern void save_lat_long_to_nvs(const char* key, gps_location_degrees_t lat_long);
extern gps_location_degrees_t read_lat_long_from_nvs(const char *key);
extern void save_int_to_nvs(const char* key, int32_t value);
extern int32_t read_int_from_nvs(const char* key);

