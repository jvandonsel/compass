#pragma once


/**
   TODO: Add support for E/W long, N/S lat
 */
struct lat_long_t {
    float latitude;
    float longitude;
};

bool gps_init();
lat_long_t gps_read();
