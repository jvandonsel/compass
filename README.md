
# Magic Compass for ESP32

This project isn't my original idea, but it's my implementation. Basically it's a box with
a needle that always points home.  Well, it can point anywhere you want, but I've configured
mine to always point home.  Using GPS and a magnetometer it figures out where it is
and where home is, and moves a servo to put the needle in the right location. Ideally, it should
look like magic.

## Hardware
- ESP32 (I'm using an AdaFruit HUZZAH32 board, based on the WROOM32 chip)
- MTK3339 GPS chipset.
- LSM303AGR Accelerometer/Magnetometer
- Parallax 360 degree feedback servo
- Wooden box, brass needle, hardware, etc.


## Building
Use `build.sh` to build.

## Flashing
Use `flash.sh` to flash the ESP32, changing the serial port as necessaryx

## Non-Volatile Storage
NVS is used to store the last GPS coordinates found, for faster startup, as well as various other settings.

## Serial Console
There's a rudimentary console program running in the background, allowing you to configure and monitor a few things.

Console commands:

* **status** - Display the status of the device
* **log** - Toggle detailed logging to the serial console on/offset
* **about** - Show info about this device
* **read** - Read saved GPS position from non-volatile storage
* **save** - Save current GPS position to non-volatile storage
* **offset** - Set pointer offset in degrees (to correct for a shifted pointer)
* **home** - Set the home location longitude and latitude
* **boston** - Set the home location to Boston Common
* **waltham** - Set the home location to Waltham, Massachusetts
* **north** - Set the home location to the North Pole
* **south** - Set the home location to the Soutn Pole
