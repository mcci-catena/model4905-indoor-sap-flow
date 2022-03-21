# model4905-indoor-sap-flow -- Capture pulses for indoor sap flow meter

## Assumptions

Two-wire flow meter, attached to A1 and GND.

A1 is JP5 pin 2.
GND is JP4 pin 4

![Pinout diagram of 4610 for reference](https://github.com/mcci-catena/HW-Designs/blob/master/Boards/Catena-4611_4612/Catena-4611_4612_4617_4618_4618-M201_Pinout.png)

## Cable entry

Drill out a suitable hole in the panel; use a slow drill, the plastic is fragile.

The dimples in the rear panel (around the USB connector) are intended for this purpose.

## Downlink commands

Downlink commands use the port number as the command index.

Port 1 controls the sample time. The message is either two or three bytes. If two bytes, the sample time (in seconds, big endian) is changed *temporarily* (for 30 uplinks). If three bytes, the third byte specifies how many samples should be sent at the new rate before reverting. If the third byte is zero, the sample time is set permanently (until reboot).

Port 2 controls rejoins. The message is two bytes, and specifies the number of seconds (big endian) to wait before rejoining. While waiting to rejoin, uplinks will not be initiated.

Port 3 controls reboots. The message is two bytes, and specifies the number of seconds (big endian) to wait before rejoining. While waiting to reboot, uplinks will not be initiated.

## Firmware update in field

- Open the enclosure.
- Attach a USB cable.
- Reset the device while holding the boot button
- Download code using DFU

## Meta

### Revision History

v1.0.0 is the first official release from this collection.

### Trademarks and copyright

MCCI and MCCI Catena are registered trademarks of MCCI Corporation. LoRa is a registered trademark of Semtech Corporation. LoRaWAN is a registered trademark of the LoRa Alliance.

This document and the contents of this repository are copyright 2021-2022, MCCI Corporation.

### License

This repository is released under the [MIT license](./LICENSE.md). Commercial licenses are also available from MCCI Corporation.

### Support Open Source Hardware and Software

MCCI invests time and resources providing this open source code, please support MCCI and open-source hardware by purchasing products from MCCI, Adafruit and other open-source hardware/software vendors!

For information about MCCI's products, please visit [store.mcci.com](https://store.mcci.com/).
