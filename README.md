
# Magic Compass for ESP32

This project isn't my original idea (precedents
 [here](https://hackaday.com/2016/11/05/personal-compass-points-to-your-spawn-point/) and
 [here](https://danthegeek.com/2018/10/12/improved-personal-compass/)), but it's my own
 implementation. Basically it's a box with a pointer that always points home. Well, it can point
 anywhere you want, but I've configured mine to always point home. Using GPS and a magnetometer it
 figures out where it is and where home is, and moves a servo to put the pointer in the right
 location. Ideally, it should be indistinguishable from magic.

## Hardware
- ESP32 (I'm using a HUZZAH32 board from [AdaFruit](https://www.adafruit.com/), based on the Espressif WROOM32 chip)
- GPS Module (I'm using the MTK3339 GPS chipset from AdaFruit)
- Accelerometer/Magnetometer (I'm using the LSM303AGR from AdaFruit)
- Servo (I'm using the Parallax 360 degree feedback servo from AdaFruit)
- [Sparkfun](https://www.sparkfun.com/) COM-15208 Buck/Boost Voltage Converter
- Power switch with LED from Sparkfun
- Wooden box, brass clock hand as a pointer, power supply, mounting hardware, etc.



## Building
Assumes you have the [Espressive SDK](https://www.espressif.com) installed as well as [esptool](https://github.com/espressif/esptool)

Use `build.sh` to build.

## Flashing
Use `flash.sh` to flash the ESP32, changing the serial port in the script as necessary.

## Non-Volatile Storage (NVS)
NVS is used to store:

* The last GPS coordinates found, for faster startup
* The home location coordinates
* Pointer position correction

## Power
I've chosen to use 4 AA batteries as the power source, hooked to a small power switch with an LED indicator. This 6v goes directly to the servo.  The 6v is also 
stepped down to 5v by a Sparkfun buck/boost board to supply the ESP32 via a microUSB connector. The GPS and magnetometer boards are powered by 3.3v coming off the ESP32 board.

## Servo Notes
The Parallax 360 is a nice device. A PWM signal from the ESP32 controls its rotation rate, and another PWM signal from the servo indicates its position. This feedback signal
is passed to an analog low-pass filter and then read by an ADC on the ESP32. In this application we want to drive the servo to a particular orientation, which means the servo
control loop needs to measure direction and command velocity. There is a large dead-band in the velocity control of this device, which might affect pointing accuracy.

## Magnetometer Notes
My LSM303AGR unit arrived calibrated and the x and y magnetic components seemed to be working well.
Then at some point during my testing it went completely wonky and I needed to determine some
correction offsets on x and y. These are hard-coded asf `MAG_CAL_OFFSET_X` and `MAG_CAL_OFFSET_Y`.
These values will certainly need to be changed for a different unit.

I made sure to mount the magnetometer as far away from the rest of the electronics (especially the servo) as possible.

There's a configuration parameter ("offset") to adjust the direction of the pointer hand with respect to the servo zero. If the pointer
hand is removed and reattached this may need to be tweaked.

## GPS Notes
The last known GPS position is read out of non-volatile storage (NVS) at startup. This allows the compass to work immediately after powering up even if
no GPS lock was acquired. Of course, if the compass has been moved significantly since its last GPS lock then its assumed position will be wrong until a new
position is acquired. GPS positions are periodically written back to NVS.

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
* **delmar** - Set the home location to Delmar, NY, and save to non-volatile storage
* **waltham** - Set the home location to Waltham, Massachusetts, and save to non-volatile storage
* **north** - Set the home location to the North Pole, and save to non-volatile storage
* **south** - Set the home location to the South Pole, and save to non-volatile storage

