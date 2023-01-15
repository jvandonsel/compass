
# Magic Compass for ESP32

This project isn't my original idea, but it's my implementation. Basically it's a box with
a needle that always points home.  Well, it can point anywhere you want, but I've configured
mine to always point home.  Using GPS and a magnetometer it figures out where it is
and where home is, and moves a servo to put the needle in the right location.

## Hardware
- MTK3339 GPS chipset.
- LSM303AGR Accelerometer/Magnetometer
- ESP32 (I'm using an AdaFruit HUZZAH32 board, based on the WROOM32 chip)
- Parallax 360 degree feedback servo
- Wooden box, brass needle, etc.


## Building
Use `build.sh` to build.

## Flashing
Use `flash.sh` to flash the ESP32, changing the serial port as necessaryx

## Non-Volatile Storage
NVS is used to store the last GPS coordinates found, for faster startup.

## Configuration
TBD


