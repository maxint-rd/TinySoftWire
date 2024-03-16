/*
    ATtiny13A I2C peripheral minimal example
    Turn the ATtiny13A into an I2C device that properly responds to an I2C controller. Supports:
      - scanning for ACK using write to address
      - writing up to 4 bytes (I2C_BUFFER_LENGTH 4), acknowledgment on every byte
      - reading up to 4 bytes (I2C_BUFFER_LENGTH 4), last byte not acknowlesdged by controller
      - detect (re)start/stop conditions

    Tested with ATtiny13A @ 9.6MHz, board MicroCore/ATtiny13 Arduino IDE 1.8.2: 8 bytes RAM, 512  bytes Flash

    For documentation of the I2C protocol see https://www.nxp.com/docs/en/user-guide/UM10204.pdf

    Made by MAXINT R&D 2020. See github.com/maxint-rd for more projects
*/

//   Pinout ATtiny13A
//                                +---v---+
//   (PCINT5/RESET/ADC0/dW) PB5 --|1     8|-- VCC
//       (PCINT3/CLKI/ADC3) PB3 --|2     7|-- PB2 (SCK/ADC1/T0/PCINT2)
//            (PCINT4/ADC2) PB4 --|3     6|-- PB1 (MISO/AIN1/OC0B/INT0/PCINT1)
//                          GND --|4     5|-- PB0 (MOSI/AIN0/OC0A/PCINT0)
//                                +-------+
//

// Pin definitions (see TinySoftWire.h)
// ====================================
// I2C_SDA 0
// I2C SCL 1
// 

//   Pinout AT24C32 I2C EEPROM (Used as model reference)
//                                +---v---+
//                           A0 --|1     8|-- VCC
//                           A1 --|2     7|-- WP
//                           A2 --|3     6|-- SCL
//                          GND --|4     5|-- SDA
//                                +-------+

#include <TinySoftWire.h>
TinySoftWire myWire=TinySoftWire();
#define I2C_ADDRESS 0x4D

// the setup function runs once when you press reset or power the board
void setup()
{
  myWire.begin(I2C_ADDRESS);

  // Initialize watchdog to autoreset in case we got stuck in some I2C waiting loop
  // Note: this will also cause a reset when while waiting for START without activity, but that's probably just fine
  wdt_enable(WDTO_1S); // Enable WDT with 1 second timeout
}

void loop()
{
  // Feed the watchdog while we're still running okay
  wdt_reset(); // Reset WDT so it doesn't reset the MCU

  // TinySoftWire::process() will wait for start condition and return once the transaction is done
  // Note that when no data is processed the watchdog will reset the MCU as specified
  myWire.process();

  //
  // Note: Whatever is done in the loop outside of process() should be done fast in order not to miss subsequent I2C transactions
  //
}
