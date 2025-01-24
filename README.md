# TinySoftWire
TinySoftWire for Arduino. A tiny I2C library to make peripherals from ATtiny13A and similar MCUs.

Make peripheral devices such as sensors using software I2C on ATtiny MCUs. Examples included.

DISCLAIMER - This library is work-in-progress. **USE AT YOUR OWN RISK!**

TinySoftWire is an I2C peripheral library for bitbanging I2C on the ATtiny13A and similar processors.
Turn the ATtiny13A into an I2C device that properly responds to an I2C controller.
Using this library the ATtiny13A at 9.6MHz can perform 100kbps [I2C communication](#i2c-protocol-simplified).
It can receive write commands and respond to read data-requests for the set I2C address.
While fully programmed in C++ the code was kept small to spare room for a useful applictions.

## Table of contents
- [Pinout ATtiny13A](#pinout-attiny13a)
- [Usage and examples](#usage-and-examples)
- [T13I2C protocol](#t13i2c-protocol)
- [Features & limitations](#features--limitations)
- [More information](#more-information)
- [Disclaimer](#disclaimer)

## Pinout ATtiny13A
```
                                +---v---+
   (PCINT5/RESET/ADC0/dW) PB5 --|1     8|-- VCC
       (PCINT3/CLKI/ADC3) PB3 --|2     7|-- PB2 (SCK/ADC1/T0/PCINT2)
            (PCINT4/ADC2) PB4 --|3     6|-- PB1 (MISO/AIN1/OC0B/INT0/PCINT1)
                          GND --|4     5|-- PB0 (MOSI/AIN0/OC0A/PCINT0)
                                +-------+
```
Following the pinout of 8-pin chips like the AT24C32 EEPROM, this library defaults to using pins PB0 and PB1 as SDA and SCL. These pins are defined in [TinySoftWire.h](src/TinySoftWire.h)

## Usage and examples

The sketch below shows the minimal code required to use this library. All it really does is calling the _process()_ method, which will put received data in the 4-byte internal buffer and return that very same data when it's read. The [I2C echo example](examples/I2C_echo/I2C_echo.ino) expands this minimal code with use of the watchdog to automatically reset when communication is blocked.
```
#include <TinySoftWire.h>
TinySoftWire myWire=TinySoftWire();

void setup() {
  myWire.begin(0x4D);      // initialize using address 0x4D
}

void loop() {
  myWire.process();        // process receiving and transmitting data from the buffer
}
```

To make the ATtiny I2C device more useful, it can be programmed to accept single byte commands, such as a command to change the I2C address. The internal EEPROM of the ATtiny can be used to store the changed address. Most [library examples](/examples) implement a [special I2C protocol](#t13i2c-protocol) for easy configuration of ATtiny I2C devices using the included [T13I2C device configurator](examples/mxT13_I2C_device_configurator) sketch.

This library uses polling to implement [I2C](#i2c-protocol-simplified). (Unfortunately interrupts seem to be too slow on the ATtiny13A @ 9.6MHz). For that reason continuous calls to _TinySoftWire::process()_ are required and the actual processing of received data/commands needs to be done quickly.

The _getLastStatus()_ method is used to check if data was written to the device. Various _getData_ and _setData_ methods are provided to see what was received and to set the reply data. (Refer to [TinySoftWire.h](src/TinySoftWire.h) for the complete list). Specific values can be used to implement commands:
```
byte nProcessed=myWire.process();
if(myWire.getLastStatus()==I2C_STATUS_WRITE && nProcessed)
{
  switch(myWire.getDataU8(0))
  {
  case 0xF0:  // blip the number of times specified
    blip(myWire.getDataU8(1));
    break;
  case 0xF3:  // get I2C address in next read, used to verify if this device is an T13I2C device.
    myWire.setDataU8(0, _i2cAddress);
    break;
  case 0xFF:  // get the device type id, used to identify the device type
    myWire.setDataU32(0, 0xC0DEDBAD);  // a 4-byte HEX-value can form a human-readable type ID 
    break;
   }
}
```
Note that the code above sets the reply data when a single byte command was written; requiring the controller to issue a subsequent I2C read to receive that reply.

See the [library examples](/examples) for more details on using this library. The [I2C_ADC example](examples/I2C_ADC) shows the various features used to implement a 4-channel ADC converter. The [TM1637](examples/I2C_gateway_TM1637) and [TM1650](examples/I2C_gateway_TM1650) gateway examples demonstrate how non-I2C devices can be given an I2C interface to make them addressable and connect to a shared bus. 

## T13I2C protocol
Using the TinySoftWire library an ATtiny13A can process [I2C data](#i2c-protocol-simplified). To make configuring ATtiny I2C devices easier, a common protocol can be implemented. When using this T13I2C protocol, data is preceeded by single byte commands. Commands F0-FF are reserved for special common purposes. The reserved commands depend on the device, but may include the following:
  - 0xF1  - switch to data mode: no further command processing*
  - 0xF3  - get the I2C-address (can be used to verify T13I2C protocol support
  - 0xF4  - change the I2C address and store it in EEPROM
  - 0xF6  - reset the I2C address to default device address
  - 0xFF  - get the T13I2C device type ID (4-bytes)

*) When in data mode sending the device type ID to the device switches back to command mode.

See the [library examples](/examples) for more information on how they implement the T13I2C protocol.
Use the [T13I2C device configurator](examples/mxT13_I2C_device_configurator) sketch on a regular Arduino board to configure T13I2C devices.

## Features & limitations
- I2C scanning is supported, but the periphiral device needs to respond quickly to prevent skipped addresses.
- Written fully in C++ as sufficient assembly skills were lacking. Better performance may be obtained using assembly.
- Unfortunately interrupts seem to be too slow on the ATtiny13A @ 9.6MHz, requiring continuous calls to TinySoftWire::process().
- Using this library the ATtiny13A at 9.6MHz can perform 100kHz I2C communication. Recognizing the address and acking after clock-pulse 8 now takes 4.5us. At 100Kbps each pulse is only 5us, so 400 kHz is much too fast.
- This library only implements 7-bit addressing. General call write/read at address 0x00 is not implemented.
- No clock-stretching was implemented (SCL is not kept low while prepping data).
- To keep things small and simple the internal buffer is only 4-bytes long. This size is defined in [TinySoftWire.h](src/TinySoftWire.h) and could be increased, but that would require implementation of supporting functions.

## More information

### Solving I2C communication problems
- The relatively slow processing of the ATtiny13A may cause lost clock pulses and data changes. Most critical is sending back the acknowledgement in time. All data processing should be done as quickly as possible. Sometimes adding/changing pull-up resistors on the I2C lines can help. When using processors such as the CH32V003 as controller these are essential, but on others they seem not required. In my experiments with an ESP8266 I found adding capacitors much more helpful. Place low value capacitors (<400pF, see table 10 of I2C specs) between SDA/SCL and ground. Having such a capacitor only on SCL (A5) may improve communication sufficiently. Effectively this delays the signals a bit, 47pF worked fine for me.
- Having long wires and loose breadboard connections may cause noise and weak signals. This is more problematic at higher speeds. Try to keep the length short and verify the connection.
- Alternatively one can change the speed of the I2C communication. Using Wire.setClock() may be limited to few supported speeds. On the Atmel chips such as the ATmega328P (and LGT8F328P) one can set the TWBR/TWPS registers. (Higher TWBR values result in lower speeds). See for more info [this fine post](http://www.gammon.com.au/forum/?id=10896) by Nick Gammon. Note that the time-outs set in the T13I2C library prevent support of very low speeds (below 10kHz).

### I2C protocol (simplified) 
```
 START CONDITION: 
     - SDA goes low while SCL is high.
 STOP CONDITION: 
     - SDA goes high after SCL goes high.

 DATA TRANSMISSIONS:
  >> I2C ADDRESS:
     - SCL goes up 8 times, Master sends address (7-bits) plus bit 0=WRITE/1=READ  
     - ACK: slave sends ack by pulling SDA low on 9th clock (NACK is keep SDA high)
 
  >> WRITE:
     - Master sends data byte 1 (8 bits + ack)
     - optional: Master sends subsequent data bytes (8 bits + ack)
 
  >> READ:
     - optional: Master sends data identification byte (8 bits + ack)
     - slave writes data on clock going up, sends no ack

 I2C ADDRESSES:
     - valid range 0x08 - 0x77  (8-119)
     - special addresses:  0x00 - R: general call, W: START byte
```

### Links & references
- For full documentation of the I2C protocol see the [NXP I<sup>2</sup>C specification and user manual](https://www.nxp.com/docs/en/user-guide/UM10204.pdf) [[mirror](documents/UM10204.pdf)]
- For more info and an I2C implementation in BascomAVR/assembly see [Elektor 2009-01 page 52-54](https://www.elektormagazine.nl/magazine/elektor-200901/15674/).

## Disclaimer
- All code on this GitHub account, including this library is provided to you on an as-is basis without guarantees and with all liability dismissed. It may be used at your own risk. Unfortunately I have no means to provide support.
