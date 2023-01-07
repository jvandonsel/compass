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

extern "C" void app_main() {
  printf("Hello.\n");

  nvs_flash_init();

  //start_webserver();

  init_magneto();

 int ticks = pdMS_TO_TICKS(1000);
  while (true) {
    vTaskDelay(ticks);
    printf("Got heading: %f degrees\n",  get_heading());
  }
}
