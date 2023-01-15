#pragma once

#include "util.h"

/**
 * Init the magnetometer
 * Returns true on success
 */
bool magneto_init();

/**
 * Return the heading in degrees.
*/
compass_degrees_t magneto_read();
