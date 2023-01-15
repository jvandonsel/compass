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
#include "servo.h"

extern "C" void app_main() {
  printf("Hello.\n");

  nvs_flash_init();

  //start_webserver();

  magneto_init();
  gps_init();
  servo_init();

  
  int ticks = pdMS_TO_TICKS(250);


  float target_direction = 0;
  int n = 0;
  while (true) {
/*
    printf("Got heading: %f degrees\n",  magneto_get_heading());
    lat_long_t lat_long = gps_read();
    printf("Got fix %f, %f\n", lat_long.latitude, lat_long.longitude);

*/
      
    servo_update_position(target_direction);

    /*
    if (n++ > 100) {
        n = 0;
        target_direction += 90;
        if (target_direction >= 360) {
            target_direction = 0;
        }
    }
    */
    
    
    vTaskDelay(ticks);

  }
}
