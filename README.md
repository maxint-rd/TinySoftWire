# TinySoftWire
TinySoftWire for Arduino. A tiny I2C library to make peripherals from ATtiny13A and similar MCUs.

Make peripheral devices such as sensors using software I2C on ATtiny MCUs. Examples included.
DISCLAIMER - This library is work-in-progress. USE AT YOUR OWN RISK!

TinySoftWare ATtiny13A I2C peripheral library for bitbanging I2C.
Turn the ATtiny13A into an I2C device that properly responds to an I2C controller.
Using this library the ATtiny13A at 9.6MHz can perform 100kbps [I2C communication](#i2c-protocol-simplified).
It can receive write commands and respond to read data-requests for the set I2C address.

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

A typical Arduino application uses setup() to initialize things:
```
  myWire.begin(0x2D);
```
The [library examples](/examples) show how to use the EEPROM of the ATtiny to store the address.

When setup() is done, the loop() function runs continuously to receive data and commands and to process them:
```
   byte nProcessed=myWire.process();
```

This library uses polling to implement I2C. Unfortunately interrupts seem to be too slow on the ATtiny13A @ 9.6MHz. For that reason continuous calls to TinySoftWire::process() are required and the actual processing of received data/commands needs to be done quickly.

The TinySoftWire::getLastStatus() method is used to check if data was received:
```
   if(myWire.getLastStatus()==I2C_STATUS_WRITE && nProcessed)   i2cAfterWrite(nProcessed);
```

Various getData and setData methods are provided to see what was received and to set the reply data. Specific values can be used to implement a [special I2C protocol](#t13i2c-protocol) for ATtiny devices:
```
  switch(myWire.getDataU8(0))
  {
  case 0xF0:  // blip n times
    // blip the number of times specified
    blip(myWire.getDataU8(1));
    break;
  case 0xF3:  // get I2C address in next read, used to verify if this device is an T13I2C device.
    myWire.setDataU8(0, _i2cAddress);
    break;
  case 0xFF:   // get the device type id
    myWire.setDataU32(0, DEVICETYPEID);
    break;
   }
```

See the [library examples](/examples) for more information on how to use this library.

## T13I2C protocol
Using the TinySoftWire library an ATtiny13A can receive [I2C data](#i2c-protocol-simplified). To make configuring ATtiny I2C devices easier, a common protocol can be implemented. When using this T13I2C protocol, data is preceeded by single-byte commands. Commands F0-FF are reserved for special common purposes. The reserved commands depend on the device, but may include the following:
  - 0xF1  - switch to data mode: no further command processing*
  - 0xF3  - get the I2C-address (can be used to verify T13I2C protocol support
  - 0xF4  - change the I2C address and store it in EEPROM
  - 0xF6  - reset the I2C address to default device address
  - 0xFF  - get the T13I2C device type ID (4-bytes)

*) When in data mode sending the device type ID to the device switches back to command mode.
See the [library examples](/examples) for more information on how they implement the T13I2C protocol.
An example configuration tool will be added soon!

## Features & limitations
- I2C scanning is supported, but the sketch needs to respond quickly to precent skipped addresses.
- Written fully in C++ as sufficient assembly skills were lacking. Better performance may be obtained using assembly.
- Unfortunately interrupts seem to be too slow on the ATtiny13A @ 9.6MHz, requiring continuous calls to TinySoftWire::process()
- Using this library the ATtiny13A at 9.6MHz can perform 100kHz I2C communication. Recognizing the address and acking after clock-pulse 8 now takes 4.5us. At 100Kbps each pulse is only 5us, so 400 kHz is much too fast. 
- No clock-stretching was implemented (SLC is not kept low while prepping data)
- General call write/read at address 0x00 is not implemented

## More information

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
For full documentation of the I2C protocol see https://www.nxp.com/docs/en/user-guide/UM10204.pdf
For more info and an I2C implementation in BascomAVR/assembly see Elektor 2009-01 page 52-54.
  (https://www.elektormagazine.nl/magazine/elektor-200901/15674/)

## Disclaimer
- All code on this GitHub account, including this library is provided to you on an as-is basis without guarantees and with all liability dismissed. It may be used at your own risk. Unfortunately I have no means to provide support.
