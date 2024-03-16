# T13I2C device configurator

Arduino utility sketch to configure T13I2C devices. T13I2C devices are I2C slave devices that use the T13I2C protocol for configuration.

## T13I2C protocol
Using the TinySoftWire library an ATtiny13A can receive I2C data. To make configuring ATtiny I2C devices easier, a common protocol can be implemented. When using this T13I2C protocol, data is preceeded by single-byte commands. Commands F0-FF are reserved for special common purposes. The reserved commands depend on the device, but may include the following:
  - 0xF1  - switch to data mode: no further command processing*
  - 0xF3  - get the I2C-address (can be used to verify T13I2C protocol support)
  - 0xF4  - change the I2C address and store it in EEPROM
  - 0xF6  - reset the I2C address to default device address
  - 0xFF  - get the T13I2C device type ID (4-bytes)
  - 
*) When in data mode sending the device type ID to the device switches back to command mode.

See the [library examples](/examples) for more information on how they implement the T13I2C protocol.

## Compatibility
This sketch was tested succesfully in various Arduino environments:
- Arduino Nano
- ESP8266
- LGT8F328P
- CH32V003
The I2C scanning feature can be used to scan generic I2C devices, but only T13I2C devices enable the extra menu items for configuration.

## Usage notes
- use a serial connection (115200 bps) to see the menu. Do NOT send line-endings (LF or CR).
- if needed add 10k pull-ups on the I2C lines
- use S to scan for up to 10 devices, then use 0-9 to select the device
- the 0xF3 and 0xFF commands are written to detect if an I2C device follows the T13I2C protocol
- most library examples implement T13I2C devices that return their 4-byte ID following the 0xFF command
- the sketch includes a hardcoded list of ID's to show a more descriptive label
- frequency testing generic devices may cause hangups or other unwanted effects
