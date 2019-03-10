AS5048A-Arduino
===============

A simple SPI library to interface with Austria Microsystem's AS5048A angle sensor with an Arduino.

The sensor should be connected to the hardware SPI pins (MISO, MOSI, SCK). The CS pin can be connected to any GPIO pin but should be passed to the constructor (in examples connected to pin 10).

# Modifications

Originally the .cpp and .h files were written by [ZoetropeLabs](https://github.com/ZoetropeLabs/AS5048A-Arduino). My current modifications include:
- you can now just copy AS5048-Arduino to the Arduino/libraries/, containts examples
- the set "getRawRotation()" method now returns the rotation from the currently set zero instead of some wierd half range signed stuff

