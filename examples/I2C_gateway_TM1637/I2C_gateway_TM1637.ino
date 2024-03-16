/*
    ATtiny13A I2C peripheral for TM1637 8x6 LED display driver
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
#define DEVICETYPEID 0xFEED1637

//
// TM1637 forwarding
// Functions derived from TM1637 library (see github.com/maxint-rd/TM16xx)
//
#define TM1637_CMD_DISPLAY  0x80
#define TM1637_CMD_DATA  0x40
#define TM1637_CMD_DATA_FIXED  0x44
#define TM1637_CMD_DATA_READ  0x42

#define TM1637_PIN_DATA 3
#define TM1637_PIN_CLOCK 4

void TM1637_start()
{ // Datasheet p.3: "When CLK is a high level and DIO changes from high to low level, data input starts."
  // TM1637 expects start and stop like I2C: at start data is low, then clock changes from high to low.
  digitalWrite(TM1637_PIN_DATA, HIGH);
  digitalWrite(TM1637_PIN_CLOCK, HIGH);
  digitalWrite(TM1637_PIN_DATA, LOW);
  digitalWrite(TM1637_PIN_CLOCK, LOW);
}

void TM1637_stop()
{ // to stop TM1637 expects the clock to go high, when strobing DIO high
  //  Datasheet p.3: "When CLK is a high level and DIO changes from low level to high level, data input ends."
  // TM1637 expects start and stop like I2C: at stop clock is high, then data changes from low to high.
  digitalWrite(TM1637_PIN_CLOCK, LOW);
  digitalWrite(TM1637_PIN_DATA, LOW);
  digitalWrite(TM1637_PIN_CLOCK, HIGH);
  digitalWrite(TM1637_PIN_DATA, HIGH);
}


void TM1637_send(byte data)
{ // send a byte to the chip the way the TM1637 likes it (LSB-first)

  for (byte i = 0; i < 8; i++)
  {
    digitalWrite(TM1637_PIN_CLOCK, LOW);

    digitalWrite(TM1637_PIN_DATA, data & 0x01 ? HIGH : LOW);    // like most TM16xx chips, the TM1637 expects LSB first
    data >>= 1;

    digitalWrite(TM1637_PIN_CLOCK, HIGH);
  }

  // unlike TM1638/TM1668 and TM1640, the TM1650 and TM1637 uses an ACK to confirm reception of command/data
  // read the acknowledgement
  // (method derived from https://github.com/avishorp/TM1637 but using pins in standard output mode when writing)
  digitalWrite(TM1637_PIN_CLOCK, LOW);
  pinMode(TM1637_PIN_DATA, INPUT);
  digitalWrite(TM1637_PIN_CLOCK, HIGH);
  uint8_t ack = digitalRead(TM1637_PIN_DATA);
  if (ack == 0)
    digitalWrite(TM1637_PIN_DATA, LOW);
  pinMode(TM1637_PIN_DATA, OUTPUT);
  // TODO: we don't do anything with the ACK, should we retry?
}

byte TM1637_receive()
{  // For the TM1637 the bit-order is MSB-first requiring different implementation than in base class.
  byte btResult = 0;

  // Pull-up on
  digitalWrite(TM1637_PIN_CLOCK, LOW);
  pinMode(TM1637_PIN_DATA, INPUT);
  //digitalWrite(TM1637_PIN_DATA, HIGH);

  for (byte i = 0; i < 8; i++)
  {
    btResult >>= 1;  // LSB first on TM1637, so shift right
    digitalWrite(TM1637_PIN_CLOCK, HIGH);  // NOTE: on TM1637 reading keys should be slower than 250Khz (see datasheet p3)
    // Is the optimizer playing tricks on us?
    // Somehow using digitalRead here increases code size and gives faulty responds on I2C scanning!
    //bool fBit=digitalRead(TM1637_PIN_DATA);
    bool fBit=((PINB&_BV(TM1637_PIN_DATA))==_BV(TM1637_PIN_DATA));
    if(fBit) btResult |= 0x80;  // LSB first on TM1637, so set highest bit
    digitalWrite(TM1637_PIN_CLOCK, LOW);
    //bitDelay();    // NOTE: on TM1637 reading keys should be slower than 250Khz (see datasheet p3)
  }

  // According I2C specs, the last value received gives a NAK.
  // TODO: we don't do anything with the ACK, should we retry?
  // let's forget about that...
  digitalWrite(TM1637_PIN_CLOCK, HIGH);
  digitalWrite(TM1637_PIN_CLOCK, LOW);

/*
  // receive Ack
  // According I2C specs, the last value received gives a NAK.
  //digitalWrite(TM1637_PIN_CLOCK, LOW);
  pinMode(TM1637_PIN_DATA, INPUT);
  digitalWrite(TM1637_PIN_DATA, HIGH);
  digitalWrite(TM1637_PIN_CLOCK, HIGH);
  uint8_t ack = digitalRead(TM1637_PIN_DATA);
  digitalWrite(TM1637_PIN_CLOCK, LOW);
  if (ack == 0)
    digitalWrite(TM1637_PIN_DATA, LOW);
*/
  pinMode(TM1637_PIN_DATA, OUTPUT);

  return btResult;
}


//
// I2C handling
//

void i2cAfterWrite(byte nProcessed)
{ // This functions handles what is done after we received some data
  // TODO (IF WE HAD MORE FLASH): support two modes: datamode and command mode.
  // In datamode all data is assumed to be ascii data, wrapping after digit 4 or CR/LF
  // In command mode the device can process regular TM1637 commands (40=display command, 42=read command, C0-C5=set display)
  
  // Let's treat first byte as command
  byte btCmd=myWire.getDataU8(0);
  byte btData=0;

  // some TM1637 commands are sent as is
  if(
    (btCmd>=TM1637_CMD_DISPLAY && btCmd<=TM1637_CMD_DISPLAY+0x0F)    // 0x80-0x8F: the display command includes display settings
//    || btCmd==TM1637_CMD_DATA      // Mode 0x40 (auto address adding) is not supported
    )
  {
    TM1637_start();
    TM1637_send(btCmd);
    TM1637_stop();
    return;
  }

  switch(btCmd)
  {
  case 0x00:    // send intensity: 0-7 (0=off)
    TM1637_start();
    btData=0x8F;                 //intensity 8, active 1
    if(nProcessed==2)
    {
      btData=myWire.getDataU8(1);
      if(btCmd==0 && btData<=0x08)
        btData=0x80 | (btData>0 ? 0x08 : 0x00) | (btData);      // intensity 0-7, active 1/0
    }
    TM1637_send(btData);
    TM1637_stop();
    break;

  // certain TM1637-commands are forwarded as a main command followed by a subsequent command with data
  case TM1637_CMD_DATA_FIXED:   // 0x44: write fixed (eg. 0x44C0FF for first digit all segments on)
    TM1637_start();
    TM1637_send(TM1637_CMD_DATA_FIXED);                 // set addressing mode
    TM1637_stop();
    TM1637_start();
    TM1637_send(myWire.getDataU8(1));   // send address command (should be 0xC0-0xC5)
    TM1637_send(myWire.getDataU8(2));   // send display data to address
    TM1637_stop();
    break;

  case TM1637_CMD_DATA_READ:     // 0x42: read button data (K1/K2 x SG1-SG8, scan codes 0x0F - 0xFF)
    {
      TM1637_start();
      TM1637_send(TM1637_CMD_DATA_READ);    // send read buttons command
      btData=TM1637_receive();
      TM1637_stop();
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
  pinMode(TM1637_PIN_DATA, OUTPUT);
  pinMode(TM1637_PIN_CLOCK, OUTPUT);

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