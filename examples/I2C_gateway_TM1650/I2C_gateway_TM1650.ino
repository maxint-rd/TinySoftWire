/*
    ATtiny13A I2C peripheral for TM1650 8x4 LED display driver
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
#include <TinySoftWire.h>
TinySoftWire myWire=TinySoftWire();

#include <EEPROM.h>
#define I2C_ADDRESS 0x4D
byte _i2cAddress=I2C_ADDRESS;
#define DEVICETYPEID 0xFEED1650

//
// TM1650 forwarding
// Functions derived from TM1650 library (see github.com/maxint-rd/TM16xx)
//
#define TM1650_CMD_MODE  0x48
#define TM1650_CMD_DATA_READ  0x49

#define TM1650_PIN_DATA 3
#define TM1650_PIN_CLOCK 4

void TM1650_start()
{ // Datasheet p.3: "Start signal: keep SCL at "1" level, SDA jumps from "1" to "0", which is considered to be the start signal."
  // TM1650 expects start and stop like I2C: at start data is low, then clock changes from high to low.
  digitalWrite(TM1650_PIN_DATA, HIGH);
  digitalWrite(TM1650_PIN_CLOCK, HIGH);
  digitalWrite(TM1650_PIN_DATA, LOW);
  digitalWrite(TM1650_PIN_CLOCK, LOW);
}

void TM1650_stop()
{ // to stop TM1650 expects the clock to go high, when strobing DIO high
  // Datasheet p.3: "End signal: keep SCL at "1" level, SDA jumps from "0" to "1", which is considered to be the end signal."
  // TM1650 expects start and stop like I2C: at stop clock is high, then data changes from low to high.
  digitalWrite(TM1650_PIN_CLOCK, LOW);
  digitalWrite(TM1650_PIN_DATA, LOW);
  digitalWrite(TM1650_PIN_CLOCK, HIGH);
  digitalWrite(TM1650_PIN_DATA, HIGH);
}

void TM1650_send(byte data)
{ // send a byte to the chip the way the TM1650 likes it (MSB-first)
  // For the TM1650 the bit-order is MSB-first requiring different implementation

  for (byte i = 0; i < 8; i++)
  {
    digitalWrite(TM1650_PIN_CLOCK, LOW);

    digitalWrite(TM1650_PIN_DATA, data & 0x80 ? HIGH : LOW);    // in contrast to other TM16xx chips, the TM1650 expects MSB first
    data <<= 1;

    digitalWrite(TM1650_PIN_CLOCK, HIGH);
  }

  // unlike TM1638/TM1668 and TM1640, the TM1650 and TM1637 uses an ACK to confirm reception of command/data
  // read the acknowledgement
  // (method derived from https://github.com/avishorp/TM1637 but using pins in standard output mode when writing)
  digitalWrite(TM1650_PIN_CLOCK, LOW);
  pinMode(TM1650_PIN_DATA, INPUT);
  digitalWrite(TM1650_PIN_CLOCK, HIGH);
  uint8_t ack = digitalRead(TM1650_PIN_DATA);
  if (ack == 0)
    digitalWrite(TM1650_PIN_DATA, LOW);
  pinMode(TM1650_PIN_DATA, OUTPUT);
  // TODO: we don't do anything with the ACK, should we retry?
}

byte TM1650_receive()
{  // For the TM1650 the bit-order is MSB-first requiring different implementation
  byte btResult = 0;

  // Pull-up on
  digitalWrite(TM1650_PIN_CLOCK, LOW);
  pinMode(TM1650_PIN_DATA, INPUT);
  //digitalWrite(TM1650_PIN_DATA, HIGH);

  for (byte i = 0; i < 8; i++)
  {
    btResult <<= 1;  // MSB first on TM1650, so shift left
    digitalWrite(TM1650_PIN_CLOCK, HIGH);  // NOTE: on TM1637 reading keys should be slower than 250Khz (see datasheet p3)
    // Is the optimizer playing tricks on us?
    // Somehow using digitalRead here increases code size and gives faulty responds on I2C scanning!
    //bool fBit=digitalRead(TM1650_PIN_DATA);  // MSB first on TM1650, so set lowest bit
    bool fBit=((PINB&_BV(TM1650_PIN_DATA))==_BV(TM1650_PIN_DATA));
    btResult |= fBit;  // MSB first on TM1650, so set lowest bit
    digitalWrite(TM1650_PIN_CLOCK, LOW);
  }

  // According I2C specs, the last value received gives a NAK.
  // TODO: we don't do anything with the ACK, should we retry?
  // let's forget about that...
  digitalWrite(TM1650_PIN_CLOCK, HIGH);
  digitalWrite(TM1650_PIN_CLOCK, LOW);

/*
  // receive Ack
  // According I2C specs, the last value received gives a NAK.
  //digitalWrite(TM1650_PIN_CLOCK, LOW);
  pinMode(TM1650_PIN_DATA, INPUT);
  digitalWrite(TM1650_PIN_DATA, HIGH);
  digitalWrite(TM1650_PIN_CLOCK, HIGH);
  uint8_t ack = digitalRead(TM1650_PIN_DATA);
  digitalWrite(TM1650_PIN_CLOCK, LOW);
  if (ack == 0)
    digitalWrite(TM1650_PIN_DATA, LOW);
*/
  pinMode(TM1650_PIN_DATA, OUTPUT);

  return btResult;
}


//
// I2C handling
//

void i2cAfterWrite(byte nProcessed)
{ // This functions handles what is done after we received some data
  // TODO (IF WE HAD MORE FLASH): support two modes: datamode and command mode.
  // In datamode all data is assumed to be ascii data, wrapping after digit 4 or CR/LF
  // In command mode the device can process regular TM1650 commands (48=display command, 49=read command, 68-6E=set display)
  
  // Let's treat first byte as command
  byte btCmd=myWire.getDataU8(0);
  byte btData=0;
  switch(btCmd)
  {
  case 0x00:    // send intensity: 0-7 (0=off)
  case TM1650_CMD_MODE:    // set display command (originally 0x48 + 0bbbm00o where b=brightness 1-7+0, m=7/8seg mode, o=on/off)
    TM1650_start();
    TM1650_send(TM1650_CMD_MODE);   // set display command
    if(nProcessed==1)
      btData=0x01;      // intensity 8, 8-segment mode, active 1
    else
    {
      btData=myWire.getDataU8(1);
      if(btCmd==0 && btData<=0x07)
        btData=(btData<<4) | (btData>0 ? 0x01 : 0x00);      // intensity 1-7, 8-segment mode, active 1/0
    }
    TM1650_send(btData);
    TM1650_stop();
    break;

  case 0x68:    // send led data to address 0x68-0x6E
  case 0x6A:
  case 0x6C:
  case 0x6E:
    TM1650_start();
    TM1650_send(btCmd);                 // set address
    TM1650_send(myWire.getDataU8(1));   // send display data to address
    TM1650_stop();
    break;

  case TM1650_CMD_DATA_READ:     // read button data (DIG1-DIG4 x KI1xKI7, scan codes 0x44-0x77)
    {
      TM1650_start();
      TM1650_send(TM1650_CMD_DATA_READ);    // send read buttons command
      btData=TM1650_receive();
      TM1650_stop();
      myWire.setDataU8(0,btData);
    }
    break;

  //
  // T13I2C-device special commands
  //
  case 0xF3:  // get I2C address in next read
    myWire.setDataU8(0, _i2cAddress);
    break;
  case 0xF4:  // set I2C address
    _i2cAddress=myWire.getDataU8(1);
    myWire.setAddress(_i2cAddress);
    // store address to EEPROM
    EEPROM.write(0, _i2cAddress);
    break;
  case 0xF6:  // reset I2C address to default
    _i2cAddress=I2C_ADDRESS;
    myWire.setAddress(_i2cAddress);
    // store address to EEPROM
    EEPROM.write(0, _i2cAddress);
    break;
  case 0xFF:   // get the device type id
    myWire.setDataU32(0, DEVICETYPEID);
    break;
  }
}

// the setup function runs once when you press reset or power the board
void setup()
{
  pinMode(TM1650_PIN_DATA, OUTPUT);
  pinMode(TM1650_PIN_CLOCK, OUTPUT);

  // read address from EEPROM byte 0
  byte address = EEPROM.read(0);
  _i2cAddress=(address!=0xFF) ? address : I2C_ADDRESS;

  myWire.begin(_i2cAddress);

  // Initialize watchdog to autoreset in case we got stuck in some I2C waiting loop
  // Note: this will also cause a reset when while waiting for START without activity, but that's probably just fine
  wdt_enable(WDTO_1S); // Enable WDT with 1 second timeout
}

// the loop function runs over and over again forever
void loop()
{
  // Feed the watchdog while we're still running okay
  wdt_reset(); // Reset WDT so it doesn't reset the MCU

  byte nProcessed=myWire.process();

  // Note: nProcessed is zero when the address is scanned.
  if(myWire.getLastStatus()==I2C_STATUS_WRITE && nProcessed)
    i2cAfterWrite(nProcessed);
}
