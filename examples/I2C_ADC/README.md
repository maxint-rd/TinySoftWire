# I2C_ADC - ATtiny13A I2C ADC example

Turn the ATtiny13A into an I2C device that properly responds to an I2C controller. This ADC implementation features 4 separate ADC channels. By connecting a TL431 to the reset pin, that pin can be used for both reset and for reading a [voltage reference](#tl431-25v-reference-voltage) on Channel 0.

Tested with ATtiny13A @ 9.6MHz, board MicroCore/ATtiny13 Arduino IDE 1.8.12: 10 bytes RAM, 1020 bytes Flash

## ATtiny13A ADC support
ATtiny13A offers the following support for ADC (see datasheet ch.14):
 - 10-bit Resolution
 - 0.5 LSB Integral Non-linearity
 - ± 2 LSB Absolute Accuracy
 - 13 - 260 μs Conversion Time
 - Up to 15 kSPS at Maximum Resolution
 - Four Multiplexed Single Ended Input Channels
 - Optional Left Adjustment for ADC Result Readout
 - 0 - VCC ADC Input Voltage Range
 - Selectable 1.1V ADC Reference Voltage
 - Free Running or Single Conversion Mode
 - ADC Start Conversion by Auto Triggering on Interrupt Sources
 - Interrupt on ADC Conversion Complete
 - Sleep Mode Noise Canceler      

## Pinout ATtiny13A
```
                                +---v---+
   (PCINT5/RESET/ADC0/dW) PB5 --|1     8|-- VCC
       (PCINT3/CLKI/ADC3) PB3 --|2     7|-- PB2 (SCK/ADC1/T0/PCINT2)
            (PCINT4/ADC2) PB4 --|3     6|-- PB1 (MISO/AIN1/OC0B/INT0/PCINT1)
                          GND --|4     5|-- PB0 (MOSI/AIN0/OC0A/PCINT0)
                                +-------+
```

## Pinout T13 I2C_ADC
Pinout ATtiny13A I2C_ADC implementation (top-view, following standard T13I2C pinout I2C_SDA=0, I2C_SCL=1)
```
                                +---v---+
              RESET/ADC0/AREF --|1     8|-- VCC
                         ADC3 --|2     7|-- ADC1
                         ADC2 --|3     6|-- SCL
                          GND --|4     5|-- SDA
                                +-------+
```

## I2C support (using TinySoftWire library):
 - scanning for ACK using write to address
 - writing up to 4 bytes (I2C_BUFFER_LENGTH 4), acknowledgment on every byte
 - reading up to 4 bytes (I2C_BUFFER_LENGTH 4), last byte not acknowledged by controller
 - detect (re)start/stop conditions
      
## T13I2C ADC commands:
 - 0x00 - read ADC channel 0 (pin 1, ADO0/RST), recommended to use TL431 for 2.5V reference.
 - 0x01 - read ADC channel 1 (pin 7, ADC1/PB2)
 - 0x02 - read ADC channel 2 (pin 3, ADC2/PB4)
 - 0x03 - read ADC channel 3 (pin 2, ADC3/PB3)
 - 0x04 - get VCC, based on reading voltage reference on ADC0
 - 0xF3 - get the I2C-address (can be used to verify T13I2C protocol support)
 - 0xF4 - change the I2C address and store it in EEPROM
 - 0xFB - set refence level in mV*100 to get calculated mV readings. 0xFB19=2500mV. Set to 0 for raw readings.
 - 0xFC - get reference level in mV*100. Default is 25 (0x19).
 - 0xFF - get the T13I2C device type ID (4-bytes)

Note: the reference value set using 0xFB is not stored in EEPROM (not enough FLASH)

## TL431 2.5V reference voltage
Using the TL431 as a 2.5V reference (bottom-view):
```
            /-----|
           /   o--+--(Cathode)--ADC0---+
          |    o--+---(Anode)---GND    +---[1K]--- VCC
           \   o--+----(Ref)----ADC0---+
            \-----|
```

Note: since !RESET has an internal pullup, the [1K] resistor is optional. Testing confirmed this.
Note2: when using a voltage divider to allow reading voltages higher than VCC in combination with the internal pullup (Rp), the pullup will cause higher readings than without.
```
           GND---R1--ADC--R2---Vm
                      +---Rp---Vcc
```

To allow proper readings the voltage divider should use resistors (R1+R2) that are low enough compared to Rp
Testing with a 1K-9.1K voltage divider indicated an internal pullup of 23.4 kOhm.
