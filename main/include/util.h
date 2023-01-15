
#pragma once
#include "esp_system.h"


#define PI 3.1415927

// Absolute degrees 0-360 in the global compass frame
// (defined as signed so we an do wrap-arounds properly)
typedef int32_t compass_degrees_t;

// Relative degrees 0-360 in the device's frame
// (defined as signed so we an do wrap-arounds properly)
typedef int32_t relative_degrees_t;

inline float rad_to_deg(float rad) { return rad * 180 / PI; }

inline float deg_to_rad(float deg) { return deg * PI / 180; }

inline float sqr(float a) {return a * a;}
struct lat_long_t {
    // Positive for N hemisphere, negative for S hemisphere
    float latitude;
    // Positive for E hemisphere, negative for W hemisphere
    float longitude;

    bool isValid() const {return (latitude !=0 || longitude != 0);}

    lat_long_t asRadians() const {return {deg_to_rad(latitude), deg_to_rad(longitude)};}
        
};

extern compass_degrees_t compute_heading(const lat_long_t here, const lat_long_t there);
extern float compute_distance_miles(const lat_long_t here, const lat_long_t there);
extern void save_to_nvs(const char* key, lat_long_t lat_long);
extern lat_long_t read_from_nvs(const char* key);
