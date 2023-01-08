#pragma once


struct lat_long_t {
    float latitude;
    float longitude;
};

bool gps_init();
lat_long_t gps_read();