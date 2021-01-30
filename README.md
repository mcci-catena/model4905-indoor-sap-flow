# catena4612-pulse-4871 -- Capture pulses for Model 4871 fuel-oil meter

## Assumptions

3-wire Aichi [OF05ZAT](https://www.aichitokei.net/products/microflow-sensor-of-z/) meter attached to A1, with power from Vdd2.

## Downlink commands

Downlink commands use the port number as the command index.

Port 1 controls the sample time. The message is either two or three bytes. If two bytes, the sample time (in seconds, big endian) is changed *temporarily* (for 30 uplinks). If three bytes, the third byte specifies how many samples should be sent at the new rate before reverting. If the third byte is zero, the sample time is set permanently (until reboot).

Port 2 controls rejoins. The message is two bytes, and specifies the number of seconds (big endian) to wait before rejoining. While waiting to rejoin, uplinks will not be initiated.

Port 3 controls reboots. The message is two bytes, and specifies the number of seconds (big endian) to wait before rejoining. While waiting to reboot, uplinks will not be initiated.
