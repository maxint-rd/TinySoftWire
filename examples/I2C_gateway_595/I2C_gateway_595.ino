/*
    ATtiny13A I2C-595 gateway device using TinySoftWire
    This T13I2C device will accept data and forward it as 595-compatible protocol
    for use with 74HC595 shiftregisters.

    Using the TinySoftWire library an ATtiny13A can receive I2C data.

    Commonly a T13I2C device accepts commands and data. The supported commands depend on the device, 
    but may include the following:
      0xF1  - switch to data mode: no further command processing*
      0xF3  - get the I2C-address (can be used to verify T13I2C protocol support
      0xF4  - change the I2C address and store it in EEPROM
      0xF6  - reset the I2C address to default device address
      0xFF  - get the T13I2C device type ID (4-bytes)
    
    *) When in data mode sending the device type ID to the device switches back to command mode.


    for documentation of the I2C protocol see https://www.nxp.com/docs/en/user-guide/UM10204.pdf

    Made by MAXINT R&D 2020. See github.com/maxint-rd for more projects


   Pinout ATtiny13A
                                +---v---+
   (PCINT5/RESET/ADC0/dW) PB5 --|1     8|-- VCC
       (PCINT3/CLKI/ADC3) PB3 --|2     7|-- PB2 (SCK/ADC1/T0/PCINT2)
            (PCINT4/ADC2) PB4 --|3     6|-- PB1 (MISO/AIN1/OC0B/INT0/PCINT1)
                          GND --|4     5|-- PB0 (MOSI/AIN0/OC0A/PCINT0)
                                +-------+
    

  Pinout 74HC595 shift register DIP and SMD model:

     +--v--+          PINS 1-8       PINS 9-16
  1 -+     +- 16      1: output Qb   16: VCC
  2 -+     +- 15      2: output Qc   15: output Qa   - expanded pin P0
  3 -+     +- 14      3: output Qd   14: Serial SER  - connect to SPI-MOSI
  4 -+     +- 13      4: output Qe   13: Enable OE   - active low, so connect to GND
  5 -+     +- 12      5: output Qf   12: Latch RCLK  - connect to SPI-SS
  6 -+     +- 11      6: output Qg   11: Clock SRCLK - connect to SPI-SCK
  7 -+     +- 10      7: output Qh   10: Clear SRCLR - active low, so connect to VCC
  8 -+     +- 9       8: GND          9: Cascade SER Qh' - to SER of next chip
     +-----+
 */

// Pin definitions (see TinySoftWire.h)
// ====================================
// I2C_SDA 0
// I2C SCL 1
// 

//   Pinout T13 I2C-595 gateway implementation (following standard T13I2C pinout I2C_SDA=0, I2C_SCL=1)
//                                +---v---+
//                        RESET --|1     8|-- VCC
//                    595_CLOCK --|2     7|-- 595_LATCH
//                     595_DATA --|3     6|-- SCL
//                          GND --|4     5|-- SDA
//                                +-------+
//

#include <TinySoftWire.h>
TinySoftWire myWire=TinySoftWire();

#include <EEPROM.h>
#define I2C_ADDRESS 0x73
byte _i2cAddress=I2C_ADDRESS;
#define DEVICETYPEID 0xFEED0595


//-------------------------------------------------------
//
// Software bitbanging the 595 chip
//

// Pin definitions for 595-forwarding. To keep SDA/SCL on pins 0/1 we don't use the SPI functions.
// As we use bitbanging, the used pins don't need to comply with the predfined (hardware) SPI pins.
#define PIN_DATA 4    // Pin connected to serial data input (DS) of 74HC595
#define PIN_LATCH 2   //Pin connected to shift register clock input (ST_CP) of 74HC595
#define PIN_CLOCK 3   //Pin connected to storage register clock input (SH_CP) of 74HC595

bool _fFast=true;      // switch between slow and fast mode 
#define DELAY_CLOCK_SLOW 160           // 16 uSec is enough for +/- 18+20 uSec clockcycle (25Khz), 5 times slower than fast mode

void delayWrite()
{ // just do nothing. Use nop instruction for very short delays
  // (two nops in a function may succeed when one isn't enough)
  //asm volatile ("nop");
  if(_fFast)
    return;    // enough for +/- 1.5+4.5 uSec clockcycle (100-150 kHz)
  delayMicroseconds(DELAY_CLOCK_SLOW);    // enough for +/- 18+20 uSec clockcycle (125)
}

void shiftOut595(byte mode, byte myDataOut)
{ // send a single byte to the 595 shiftregister using bitbanging
  boolean fMode=(mode==MSBFIRST);

  for (byte i=0; i<8; i++)
  {
    digitalWrite(PIN_DATA, myDataOut & (fMode?0x80:0x01));      // using _BV(i) or >>i takes more time each iteration
    if(fMode) myDataOut<<=1; else myDataOut>>=1;

    delayWrite();  // allow for proper shifting to second device on rising clock edge
    digitalWrite(PIN_CLOCK, HIGH);
    delayWrite();  // calling a 2-nop function provides sufficient delay on ATtiny13A@9.6Mhz

    digitalWrite(PIN_CLOCK, LOW); 
    delayWrite();  // calling a 2-nop function provides sufficient delay on ATtiny13A@9.6Mhz
  }

}

void sendBytes595(byte *pBuf, byte nBytes)
{ // send the specified number of bytes from the buffer

  // Set latch low. Storage register is filled when latch goes high
  digitalWrite(PIN_LATCH, LOW);
  
  // shift the data out
  for(byte n=0; n<nBytes; n++)
    shiftOut595(MSBFIRST, pBuf[n]);

  // set latch high. This moves data from the shift register to the storage register of the 595(s)
  digitalWrite(PIN_LATCH, HIGH);
}

void txByte(byte b)
{   // send one byte
  sendBytes595(&b, 1);
}

//-------------------------------------------------------
//
// Handling of T13I2C device commands
//

bool _fDataMode=false;

void i2cAfterWrite(byte nProcessed)
{ // This functions handles what is done after we received some data
  
  // In data mode the data is forwarded as is. Unless it's the 4-byte DEVICETYPEID, which is used to switch to command mode.
  if(_fDataMode)
  {   // just forward the data as serial
    if(myWire.getDataU32()==DEVICETYPEID)
    {
      _fDataMode=false;
      // store setting to EEPROM
      EEPROM.write(1, _fDataMode);
      return;
    }
    for(byte n=0; n<nProcessed; n++)
      txByte(myWire.getDataU8(n));
    return;
  }
  
  // Let's treat first byte as command
  switch(myWire.getDataU8(0))
  {
  case 0x00:  // forward subsequent data as serial
    for(byte n=1; n<nProcessed; n++)
      txByte(myWire.getDataU8(n));
    break;
  case 0xF1:  // switch to data mode
    // In data mode the data is send as is, no commandbyte required
    // To escape data mode the 4-byte device type id needs to be sent.
    _fDataMode=true;
    // store setting to EEPROM
    EEPROM.write(1, _fDataMode);
    break;
  case 0xF3:  // get I2C address in next read (can be used to verify device has command support)
    myWire.setDataU8(0, _i2cAddress);     // NOTE: using setDataU8 uses less flash than setDataU32
    break;
  case 0xF4:  // set I2C address
    _i2cAddress=myWire.getDataU8(1);
    myWire.setAddress(_i2cAddress);
    // store address to EEPROM
    EEPROM.write(0, _i2cAddress);
    break;
/* if not enough flash, comment this code out */
  case 0xF6:  // reset I2C address to default
    _i2cAddress=I2C_ADDRESS;
    myWire.setAddress(_i2cAddress);
    // store address to EEPROM
    EEPROM.write(0, _i2cAddress);
    break;
/**/
  case 0xFF:   // get the device type id
    myWire.setDataU32(0, DEVICETYPEID);
    break;
  }
}

//-------------------------------------------------------
//
// setup() and loop()
//
void setup()
{
  // Set 595-output pins
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_LATCH, HIGH);

  // read I2C address from EEPROM byte 0
  byte address = EEPROM.read(0);
  _i2cAddress=(address!=0xFF) ? address : I2C_ADDRESS;

  // get settings from EEPROM
  _fDataMode=true;
  byte setting=EEPROM.read(1);
  _fDataMode=(setting!=0xFF) ? setting : false;

  myWire.begin(_i2cAddress);

  // Initialize watchdog to autoreset in case we got stuck in some I2C waiting loop.
  // Note: the TinySoftWire library uses time-outcounters to prevent long loops while waiting for START without activity.
  // If any loop takes longer than the time specified below, the watchdog resets, but that's probably just fine.
  wdt_enable(WDTO_1S); // Enable WDT with 1 second timeout
}

// the loop function runs over and over again forever
void loop()
{
  // Feed the watchdog while we're still running okay
  wdt_reset(); // Reset WDT so it doesn't reset the MCU

  byte nProcessed=myWire.process();

  if(myWire.getLastStatus()==I2C_STATUS_WRITE)
    i2cAfterWrite(nProcessed);
}
