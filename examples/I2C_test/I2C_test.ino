/*
    ATtiny13A I2C peripheral test
    Turn the ATtiny13A into an I2C device that properly responds to an I2C controller

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

//   Pinout AT24C32 I2C EEPROM (Used as model reference)
//                                +---v---+
//                           A0 --|1     8|-- VCC
//                           A1 --|2     7|-- WP
//                           A2 --|3     6|-- SCL
//                          GND --|4     5|-- SDA
//                                +-------+
#include <TinySoftWire.h>
TinySoftWire myWire=TinySoftWire();

#include <EEPROM.h>
#define EEPROM_ADDRESS 0
#define I2C_ADDRESS 0x57
byte _i2cAddress=I2C_ADDRESS;
#define DEVICETYPEID 0xC0DEDBAD


// LED_BUILTIN is defined as pin 2 on MicroCore board ATtiny13
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

void blip(byte cnt=1)
{ // Blip to flash LED for debugging
  // LED_BUILTIN flash duration without delay is 0.19 us, doing function call about 0.25us
  while(cnt) // loop adds 0.5us to call 
  // do
  {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);     // LED_BUILTIN flash duration without delay is 0.2 us
    cnt--;
  } // while(--cnt); // loop adds 0.5us to call 
}

void bliplong(byte dur=0)
{ // Blip to flash LED for debugging
  // LED_BUILTIN flash duration without delay is 0.19 us, doing function call about 0.25us
  digitalWrite(LED_BUILTIN, HIGH);
  if(dur>0) delayMicroseconds(dur);
  digitalWrite(LED_BUILTIN, LOW);     // LED_BUILTIN flash duration without delay is 0.2 us
}


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


//char buf[7]; // 16 bit int is -32768 to 32767, global buf uses 10 bytes less flash, 7 bytes more RAM
void txInt(int num)
{
  char buf[7]; // 16 bit int is -32768 to 32767
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

void i2cAfterWrite(byte nProcessed)
{ // This functions handles what is done after we received some data
  // Let's treat first byte as command
  switch(myWire.getDataU8(0))
  {
  case 0x00:  // serial out the hex input
    TxByte(myWire.getDataU8(1));
    break;
/**/
  case 0xF0:  // blip n times
    // blip the number of times specified
    //for(int i=0; i<nProcessed; i++)
    //  txHex(myWire.getDataU8(i));
    blip(myWire.getDataU8(1));
    break;
/**/
  case 0xF3:  // get I2C address in next read, used to verify if this device is an T13I2C device. (Could also be usefull when responding to broadcast address 0)
    myWire.setDataU8(0, _i2cAddress);
    break;
  case 0xF4:  // set I2C address
    // verify parameters to prevent accidental setting wrong address (1=new adress, 2=old address, 3=new adress again)
    //if(myWire.getDataU8(1)==myWire.getDataU8(3) && _i2cAddress==myWire.getDataU8(2))
    if(_i2cAddress==myWire.getDataU8(2))      // only check old address to save some bytes
    {
      _i2cAddress=myWire.getDataU8(1);
      myWire.setAddress(_i2cAddress);
      EEPROM.write(EEPROM_ADDRESS, _i2cAddress);      // store address to EEPROM
    }
    break;
/**/
  case 0xF6:  // reset I2C address to default
    _i2cAddress=I2C_ADDRESS;
    myWire.setAddress(_i2cAddress);
    // store address to EEPROM
    EEPROM.write(EEPROM_ADDRESS, _i2cAddress);
    break;
/**/
  case 0xFF:   // get the device type id
    myWire.setDataU32(0, DEVICETYPEID);
    break;
  }
}

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // read address from EEPROM_ADDRESS
  byte address = EEPROM.read(EEPROM_ADDRESS);
  //_i2cAddress=(address!=0xFF) ? address : I2C_ADDRESS;
  _i2cAddress=(address!=0xFF && address!=0x00) ? address : I2C_ADDRESS;

  txStr_P(P("Tiny13A I2C-device. Serial on "));
  txInt(SER);

  txStr_P(P(". I2C on: "));
  txHex(_i2cAddress);

  myWire.begin(_i2cAddress);

  // Initialize watchdog to autoreset in case we got stuck in some I2C waiting loop
  // Note: this will also cause a reset when while waiting for START without activity, but that's probably just fine
  wdt_enable(WDTO_1S); // Enable WDT with 1 second timeout

  // Disable interrupts may speed up ACK-timing on first write (0.5us difference is a lot) 
  // Note: we're not using interrupts for millis() or other stuff)
  //cli();
}

// the loop function runs over and over again forever
void loop()
{
  // Feed the watchdog while we're still running okay
  wdt_reset(); // Reset WDT so it doesn't reset the MCU

  // Process the I2C data. Note: on I2C_STATUS_WRITE nProcessed can be zero when just scanning for devices
  byte nProcessed=myWire.process();

  // Note: blipping or writing serial requires controller to add time between subsequent write/read
  if(myWire.getLastStatus()==I2C_STATUS_NOSTART)
    //txStr_P(P("x"));
    txStr("x"); // using txStr costs 2 bytes RAM, but is 10 bytes FLASH smaller than txStr_P
  else if(myWire.getLastStatus()==I2C_STATUS_NOADDRESS)
    txStr("-");
  else if(myWire.getLastStatus()==I2C_STATUS_WRITE && nProcessed)
    i2cAfterWrite(nProcessed);
  //if(nProcessed)
  //{
/*
    // see if there's another write/read
    // common form is to write a command, then read the reasult, eg. W@50:123, R@50:??
    nReceived=myWire.process();
    if(nReceived)
    {
      blip(3);
      blip(3);

      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      fControllerRead=myWire.isRead();      // fControllerRead is true when controller wants to read
      txStr(fControllerRead?"R":"W");
      txStr(":");
      txHex(bW1);
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    }
*/
  //}
}
