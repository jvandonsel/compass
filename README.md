
# Magic Compass for ESP32

This project isn't my original idea (precedents [here](https://hackaday.com/2016/11/05/personal-compass-points-to-your-spawn-point/) 
 and [here](https://danthegeek.com/2018/10/12/improved-personal-compass/)), but it's my own implementation. Basically it's a box with
a needle that always points home.  Well, it can point anywhere you want, but I've configured
mine to always point home.  Using GPS and a magnetometer it figures out where it is
and where home is, and moves a servo to put the needle in the right location. Ideally, it should
be indistinguishable from magic.

## Hardware
- ESP32 (I'm using a HUZZAH32 board from [AdaFruit](https://www.adafruit.com/), based on the Espressif WROOM32 chip)
- GPS Module (I'm using the MTK3339 GPS chipset from AdaFruit)
- Accelerometer/Magnetometer (I'm using the LSM303AGR from AdaFruit)
- Servo (I'm using the Parallax 360 degree feedback servo from AdaFruit)
- Wooden box, brass clock hand as a pointer, power supply, hardware, etc.



## Building
Assumes you have the [Espressive SDK](https://www.espressif.com) installed as well as [esptool](https://github.com/espressif/esptool)

Use `build.sh` to build.

## Flashing
Use `flash.sh` to flash the ESP32, changing the serial port in the script as necessary.

## Non-Volatile Storage
NVS is used to store:

* The last GPS coordinates found, for faster startup
* The home location coordinates
* Pointer position correction


## Serial Console
There's a rudimentary console program running in the background, allowing you to configure and monitor a few things assuming
you have a serial terminal connected to your ESP32.

Console commands:

* **status** - Display the status of the device
* **log** - Toggle detailed logging to the serial console on/offset
* **about** - Show info about this device
* **read** - Read saved GPS position from non-volatile storage
* **save** - Save current GPS position to non-volatile storage
* **offset** - Set pointer offset in degrees (to correct for a shifted pointer)
* **home** - Set the home location longitude and latitude, and save to non-volatile storage
* **boston** - Set the home location to Boston Common, and save to non-volatile storage
* **waltham** - Set the home location to Waltham, Massachusetts, and save to non-volatile storage
* **north** - Set the home location to the North Pole, and save to non-volatile storage
* **south** - Set the home location to the South Pole, and save to non-volatile storage

