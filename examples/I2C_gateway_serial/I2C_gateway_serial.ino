/*
    ATtiny13A I2C-serial gateway device
    This T13I2C device will accept data and forward it as serial (N81, 115200 baud)

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

// Serial forwarding ports:
//   UART_Tx pin 3, UART_Rx pin 4
//
//
//

#include <TinySoftWire.h>
TinySoftWire myWire=TinySoftWire();

#include <EEPROM.h>
#define I2C_ADDRESS 0x6D
byte _i2cAddress=I2C_ADDRESS;

//-------------------------------------------------------
//
// Simple software-serial on UART_Tx pin 3, UART_Rx pin 4
//
#include <BasicSerial3.h>
#define SER 3

void txStr(const char* str)
{
  while (*str) TxByte (*str++);
}

void txStr_P(const char* str)
{
  // const __FlashStringHelper *ifsh
  // const char PROGMEM *p = (const char PROGMEM *)ifsh;
  char c;
  do
  {
    c = (char) pgm_read_byte(str++);
    if(c) TxByte(c);
  }
  while (c);
}
#define P(x) ((const char PROGMEM *)F(x))

void txInt(int num)
{
  char buf[7]; // 16 bit int is -32768 to 32767, global buf uses 10 bytes less flash, 7 bytes more RAM
  byte i=sizeof(buf);
  buf[--i]='\0';
  if(num<0) { TxByte('-'); num*=-1; }
  if(num==0) TxByte('0');
  while(num && i>0)
  {
    byte x=num%10;   // lsb
    num/=10;
    buf[--i]=x+'0';
  }
  txStr(buf+i);
}

void txHex(byte num)
{  // write a single byte in hex notation
    byte digit;

    digit=num>>4;
    TxByte(digit<10 ? '0'+digit : 'A'+digit-10);
    digit=num&0x0F;
    TxByte(digit<10 ? '0'+digit : 'A'+digit-10);
}

//-------------------------------------------------------
//
// Handling of T13I2C device commands
//

bool _fDataMode=false;
#define DEVICETYPEID 0xFEED0232

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
      TxByte(myWire.getDataU8(n));
    return;
  }
  
  // Let's treat first byte as command
  switch(myWire.getDataU8(0))
  {
  case 0x00:  // forward subsequent data as serial
    for(byte n=1; n<nProcessed; n++)
      TxByte(myWire.getDataU8(n));
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
  // read address from EEPROM byte 0
  byte address = EEPROM.read(0);
  _i2cAddress=(address!=0xFF) ? address : I2C_ADDRESS;

  // get settings from EEPROM
  _fDataMode=true;
  byte setting=EEPROM.read(1);
  _fDataMode=(setting!=0xFF) ? setting : false;

/* if not enough flash, comment this code out */
  txStr_P(P("Tiny13A I2C-device on 0x"));
  txHex(_i2cAddress);
/**/

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
