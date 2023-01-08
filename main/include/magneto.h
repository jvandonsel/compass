#pragma once

/**
 * Init the magnetometer
 * Returns true on success
 */
bool magneto_init();

/**
 * Return the heading in degrees.
*/
float magneto_get_heading();