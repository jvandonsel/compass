
#pragma once

#include "esp_system.h"
#include "compass.h"
#include "util.h"


struct servo_status_t {
    // Measured servo position relative to the device
    relative_degrees_t relative_pos;
    // Rotation rate [-1.0 - 1.0]
    float rate;
};

bool servo_init();
servo_status_t servo_update(relative_degrees_t pos);
relative_degrees_t servo_get_position();
void servo_drive_slow();
