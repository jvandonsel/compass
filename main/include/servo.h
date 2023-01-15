
#pragma once

#include "esp_system.h"


void servo_set_rate(float rate);
bool servo_init();
void servo_update_position(float target_degrees);
