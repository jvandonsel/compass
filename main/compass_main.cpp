/*
 * Magic compass for ESP32
 *
 * Author: J Van Donsel
 * Date: 12/29/2022
 */
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "http.h"
#include "nvs_flash.h"
#include "magneto.h"
#include "gps.h"

extern "C" void app_main() {
  printf("Hello.\n");

  nvs_flash_init();

  //start_webserver();

  magneto_init();
  gps_init();

  int ticks = pdMS_TO_TICKS(1000);
  while (true) {
    vTaskDelay(ticks);
    printf("Got heading: %f degrees\n",  magneto_get_heading());
    lat_long_t lat_long = gps_read();
    printf("Got fix %f, %f\n", lat_long.latitude, lat_long.longitude);

  }
}
