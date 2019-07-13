# door/window OPEN/SHUT sensor project

This project was derived from an incomplete project (LoFi) submitted by David Cook to hackaday.io. See https://hackaday.io/project/1552-lofi. He did not submitt software with his project.

My son ordered some boards and parts for me to play with. I decided to hack the original board to use an NRF24l01+ instead of the original 433 MHz transmitter.

## Features

- Draws about 6uA in the 'idle' mode.
- Supports two reed switches for door/window sensors.
- Uses Pin Change feature to report when a reed swith opens or closes.
- Uses the Watchdog feature to report switch state every N seconds. N is stored in eeprom.
- Can report the sensors battery voltage every M seconds. M is stored in eeprom.
- Can report the sensors temperature (rudamentary) every P seconds. P is stored in eeprom.
- Can report a 10-bit count that increments every R seconds. R is stored in eeprom. 
- Transmits a 3-byte packet. The packet contains the switch, Vcc, Temp, or Counter data.
- Can select transission bit rate (250kbps, 1Mbps, or 2Mbps) via eeprom setting.
- Can select to blink leds to indicate packet transmission and whether ACK was received or not.
- Can select channels 0..127 via eeprom.
- Can select Watchdog timeout period via eeprom.
- Can select RF gain setting via eeprom.
- Can enable Auto-Acknowledge and Dynamic Acknowledge via eeprom.

There is a companion program (lofi_mqtt) that receives these packets and converts them to MQTT topics and publishes them to an MQTT broker. This companion program runs on Odroid or Raspberry Pi. 
 
I am using parts of xprintf by ChaN (see xprintf.c for copyright)

I am using parts of nrf24 by <ihsan@ehribar.me> (see nrf24.c for license)

I have created/hacked together 19 nodes. These use the CR2032 or the CR2450 to power them. I have some nodes that use the LIR2450, but these nodes need to have an LDO installed to reduce the potential 4.2V to under 3.6V. The maximum voltage the nrf24 can support is 3.6V.

Using the LIR2450 was a mistake. It has much less capacity that a CR2450 and required the addition of the LDO to the sensor. I recommend using only the CR versions of the coin cell batteries.

## Flashing Sensor

I use a 'usbtiny' or 'usbasp' to program the sensor. The Makefile is my friend, since I have a hard time remembering things.

You first want to change the fuse settings so that the eeprom won't get erased every time you reflash the program onto the sensor.
```
$ make pgmfuses
```

Then you flash the program to the sensor:
```
$ make pgm
```

Then you modify the eeprom to enable features you want. I would like to create a GUI to help with this part but currently one has to look at lofi.h (the config_t structure). I use avrdude to gain access to the eeprom:
```
$ make dude
```

then use "d eep 0 20" to dump the contents of the eeprom.

then use "w eep <addr> <bytes> to write to the eeprom.



### File strutcture

The library basically consists of only three files: `nrf24.c `, `nrf24.h` and `nRF24L01.h` . Hardware spesific functions are defined at the `radioPinFunctions.c` file. This file can be different at each device / platform.

### Configuration


### Addressing


### Transmit

