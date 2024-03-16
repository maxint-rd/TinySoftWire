/*
    Arduino configuration tool for ATtiny I2C devices (T13I2C)
    Using the TinySoftWire library an ATtiny13A can receive I2C data.

    Commonly a T13I2C device accepts commands and data. The supported commands depend on the device, 
    but may include the following:
      0xF1  - switch to data mode: no further command processing*
      0xF3  - get the I2C-address (can be used to verify T13I2C protocol support
      0xF4  - change the I2C address and store it in EEPROM
      0xF6  - reset the I2C address to default device address
      0xFF  - get the T13I2C device type ID (4-bytes)
    *) When in data mode sending the device type ID to the device switches back to command mode.

    For documentation of the I2C protocol see https://www.nxp.com/docs/en/user-guide/UM10204.pdf

    Made by MAXINT R&D 2020. See github.com/maxint-rd for more projects
*/

// Solving I2C communication problems
// - The relatively slow processing of the ATtiny13A may cause lost clock pulses and data changes.
//   Most critical is sending back the acknowledgement in time. 
//   Sometimes pull-up resistors on the I2C lines can help, but in my experiments I found adding capacitors
//   much more helpful. Place low value capacitors (<400pF, see table 10 of I2C specs) between SDA/SCL and ground.
//   Having such a capacitor only on SCL (A5) may improve communication sufficiently.
//   Effectively this delays the signals a bit, 47pF worked fine for me.
// - Alternatively one can change the speed of the I2C communication. Using Wire.setClock() may be limited to few supported speeds.
//   On the Atmel chips such as the ATmega328P (and LGT8F328P) one can set the TWBR/TWPS registers. (Higher TWBR values result in lower speeds).
//   See for more info this fine post by Nick Gammon: http://www.gammon.com.au/forum/?id=10896 
//   Note that the time-outs set in the T13I2C library prevent support of very low speeds (below 10k)

#include <Wire.h>

#if defined(CH32V00x)
  #define LED_BUILTIN PA2       // PC0 for CH32V003-F4P6 TSSOP20, PA2 for CH32V003-J4M6 SOP8
  #define OPT_I2C_FREQTEST 0    // no room on CH32V003: FLASH: 15288/16384  RAM: 752/2048 (ld.exe: region `FLASH' overflowed by 2416 bytes)
#else
  #define OPT_I2C_FREQTEST 1
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 4
  #endif
#endif

void blink(byte cnt=1, byte dur=1)
{ // flash LED for debugging
  // LED_BUILTIN flash duration without delay is 0.19 us, doing function call about 0.25us
  for(byte n=0; n<cnt; n++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if(dur>0) delay(1+dur/10);
    digitalWrite(LED_BUILTIN, LOW);     // LED_BUILTIN flash duration without delay is 0.2 us
    if(dur>0) delay(dur-dur/10);
  }
}

bool i2cWriteU8(byte addr, uint8_t data)
{
  Wire.beginTransmission(addr);
  Wire.write(data);
  return(Wire.endTransmission()==0);      // ==0 means ack received, !=0 means nack or other error
  //blink(1);
}

bool i2cWriteU16(byte addr, uint16_t data)
{ // Note that AVR memory is Big-Endian: 0xDEAD is sent as AD DE
  Wire.beginTransmission(addr);
  Wire.write((uint8_t*)&data, 2);
  return(Wire.endTransmission()==0);      // ==0 means ack received, !=0 means nack or other error
  //blink(1);
}


bool i2cWriteU32(byte addr, uint32_t data)
{ // Note that AVR memory is Big-Endian: 0xDEADBEEF is sent as EF BE AD DE
  Wire.beginTransmission(addr);
  Wire.write((uint8_t*)&data, 4);
  return(Wire.endTransmission()==0);      // ==0 means ack received, !=0 means nack or other error
  //blink(1);
}


bool i2cReadTest(byte addr)
{
  uint8_t data=0;
  Wire.requestFrom(addr, (byte)0); //retrieve nBytes bytes
  return(data);
}

uint8_t i2cReadU8(byte addr)
{ // Note that AVR memory is Big-Endian: 0xDEADBEEF is sent as EF BE AD DE
  // when we reveive the data, it needs to be in proper order
  // Note that Wire.requestFrom() performs the actual transmission, Wire.available() and Wire.read() only work on the I2C buffer
  uint8_t data=0;
  Wire.requestFrom(addr, (byte)1); //retrieve nBytes bytes
  if(Wire.available())
    data=Wire.read();
  return(data);
}

uint16_t i2cReadU16(byte addr)
{ // Note that AVR memory is Big-Endian: 0xDEADBEEF is sent as EF BE AD DE
  // when we reveive the data, it needs to be in proper order
  uint16_t data=0;
  Wire.requestFrom(addr, (byte)2); //retrieve nBytes bytes
  while(Wire.available())
  {
    data>>=8;
    data|=((uint16_t)Wire.read())<<8;
  }
  //blink(1);
  return(data);
}

uint32_t i2cReadU32(byte addr)
{ // Note that AVR memory is Big-Endian: 0xDEADBEEF is sent as EF BE AD DE
  // when we receive the data, it needs to be in proper order
  uint32_t data=0;
  Wire.requestFrom(addr, (byte)4); //retrieve nBytes bytes
  while(Wire.available())
  {
    data>>=8;
    data|=((uint32_t)Wire.read())<<24;
  }
  //blink(1);
  return(data);
}

// ================================================

#define I2C_ADDRESS 0x00
byte _i2cAddressSelected=I2C_ADDRESS;   // selected address
uint32_t _uDeviceTypeID=0;     // last device type ID read


#define MAX_ADDRESSES 10
byte _arrAddresses[MAX_ADDRESSES];
bool _arrIsT13I2C[MAX_ADDRESSES];
uint32_t _arrDeviceTypeIDs[MAX_ADDRESSES];
byte _nCntAddresses=0;

const char _szUnknown[] PROGMEM = "-unknown-";
const char _szKnownID0[] PROGMEM = "I2C test";
const char _szKnownID1[] PROGMEM = "595 Gateway";
const char _szKnownID2[] PROGMEM = "Serial Gateway";
const char _szKnownID3[] PROGMEM = "TM1650 Gateway";
const char _szKnownID4[] PROGMEM = "TM1637 Gateway";
const char _szKnownID5[] PROGMEM = "4-channel ADC";
const char _szKnownID6[] PROGMEM = "3-channel ADC";
const char _szKnownID7[] PROGMEM = "12-14V DC-DC converter";
const char _szKnownID8[] PROGMEM = "12-17V DC-DC converter";
uint32_t _arrKnownDeviceTypeIDs[MAX_ADDRESSES]={0xC0DEDBAD, 0xFEED0595, 0xFEED0232, 0xFEED1650, 0xFEED1637, 0xADC4C005, 0xADC3C005, 0xDCDC1214, 0xDCDC1217};
const char * _arrKnownDeviceTypeDescriptions[MAX_ADDRESSES]={_szKnownID0, _szKnownID1, _szKnownID2, _szKnownID3, _szKnownID4, _szKnownID5, _szKnownID6, _szKnownID7, _szKnownID8};
/*
#define CF(x) ((const PROGMEM __FlashStringHelper *)(x))
#define PCF(x) ((const char *)((const PROGMEM __FlashStringHelper *)(x)))
const __FlashStringHelper * _arrKnownDeviceTypeDescriptions[MAX_ADDRESSES]={CF("I2C test"), CF("595 Gateway"), CF("Serial Gateway"), CF("TM1650 Gateway"), CF("TM1637 Gateway")};
const char * _arrUnknownDeviceTypeDescriptions[]={PCF("-unknown-")};
*/
// macro to print flash strings
//#define P(x) (reinterpret_cast<const __FlashStringHelper *> pgm_read_word(x))
#define P(x) ((const __FlashStringHelper *)(x))
//#define PP(x) ((const __FlashStringHelper *)( pgm_read_word(x)))


void printMenu()
{ // print the menu
  Serial.println(F("\n\n======================================"));
  Serial.println(F("Maxint R&D T13 I2C device configurator"));
  Serial.println(F("============================== v231014"));
  Serial.println(F(" S  - Scan for devices"));
  Serial.print(F("0-9 - Select device"));

  bool fIsT13I2C=false;
  if(_i2cAddressSelected==0)
    Serial.println(F(" (no device selected)"));
  else
  {
    for(byte i=0; i<MAX_ADDRESSES; i++)
    {
      if(_arrAddresses[i]==_i2cAddressSelected)
      {
        fIsT13I2C=_arrIsT13I2C[i];
        _uDeviceTypeID=_arrDeviceTypeIDs[i];
      }
    }
    Serial.print(F("(0x"));
    Serial.print(_i2cAddressSelected, HEX);
    Serial.println(F(" selected)"));
    if(fIsT13I2C)
    {
      Serial.println(F(" A  - Set Address (eg.: A4D=set address to 0x4D)"));
      Serial.println(F(" I  - Get device type id hex code"));
      Serial.print(F(" C  - Switch to command mode ["));
      Serial.print(_uDeviceTypeID, HEX);
      Serial.println(F("]"));
      Serial.println(F(" D  - Switch to data mode"));
    }
#if OPT_I2C_FREQTEST
Serial.println(F(" F  - Frequency-test selected device"));
#endif // #if OPT_I2C_FREQTEST
    Serial.println(F(" Rn - Read n single bytes"));
    Serial.println(F(" T  - Send text data"));
    Serial.println(F(" V  - Write HEX data (N 1-byte values)"));
    Serial.println(F(" W  - Write HEX data (1/2/4-byte value)"));
    Serial.println(F(" X  - Send HEX data (1/2/4-byte value), read 4-byte reply"));
    Serial.println(F(" ?  - Read a 4-byte value"));
    Serial.println(F("NOTE: do not send line-endings when entering data!"));
  }
}

/**/
const char PROGMEM * getKnownDeviceType(uint32_t uID)
{
  for(byte n=0; n<MAX_ADDRESSES; n++)
  {
    if(_arrKnownDeviceTypeIDs[n]==uID)
      return(_arrKnownDeviceTypeDescriptions[n]);
  }
  return(_szUnknown);
}
/**/
/*
const __FlashStringHelper * getKnownDeviceType(uint32_t uID)
{
  for(byte n=0; n<MAX_ADDRESSES; n++)
  {
    if(_arrKnownDeviceTypeIDs[n]==uID)
      return(_arrKnownDeviceTypeDescriptions[n]);
  }
  return(_arrUnknownDeviceTypeDescriptions[0]);   // unknown
}
*/

uint32_t i2cCalcFrequencyFromTWBR(byte nTWBR)
{ // Wire.setClock(frequency) supposedly does TWBR = ((F_CPU / frequency) - 16) / 2;
  // but that results in a frquency 18kHz too high, compared to what the logical analyzer says.
  // Most likely the clock of the used LGT8F328P isn't exactly F_CPU.
  // Experiments showed differences up to 5% from day to day.
  //uint32_t uFreqency=F_CPU / ((nTWBR*2L)+16L);
  uint32_t uFreqency=F_CPU / ((nTWBR*2L)+50L);
  return(uFreqency);
}

#if OPT_I2C_FREQTEST
void i2cTestFrequency(byte i2cAdressTest)
{ // Test to see what the highest frequency is that the device supports.
  // The frequency is tested by repeatedly writing to the device at different frequencies and checking if the device properly acknowledges
#if defined(TWBR)
  Serial.println("Testing frequency by setting TWBR:");
  byte nTWBRorg=TWBR;
  byte nFreqMax=0;
  //TWBR = 134;    // http://www.gammon.com.au/forum/?id=10896 (on LGT8F328P: 12=400k, 24=300k, 48=200k, 96=130k, 132=100k, 144=94k, 200=70k, 255=57k)
  // Fastest speed measured: TWBR=1 (615 kHz) on Atmel 24C02 Eeprom. Best for T13I2C: TWBR=106 (123kHz) on I2C_Test example T13@9.6MHz
  //TWSR |= bit (TWPS0);    // set prescaler to 4/16/64 for lower speeds (speeds lower than 10k need different timeout settings in TinySoftWire.h)
  //TWSR |= bit (TWPS1);

  TWBR=nTWBRorg;
  delay(100);
  Wire.beginTransmission(i2cAdressTest);
  Serial.print("Test normal: ");
  Serial.print(i2cCalcFrequencyFromTWBR(TWBR)/1000L);
  Serial.print(" kHz, TWBR=");
  Serial.print(TWBR);
  if (Wire.endTransmission() == 0)
  { // ack received      
    Serial.println(" OK, ");
  }
  else
  { // no ACK
    Serial.println(" NOK!");
  }



  // test various frequencies
  for(byte nFreq=255; nFreq>0; nFreq--)
  {
    TWBR=nFreq;  
    delayMicroseconds(500);
    Wire.beginTransmission(i2cAdressTest);
    Serial.print(nFreq);
    if (Wire.endTransmission() == 0)
    { // ack received      
      Serial.print(" ");
      nFreqMax=nFreq;
    }
    else
    { // no ACK
      // Missing a bit may result in incomplete transmission, timeouts and possibly the device resetting (eg. 1sec WDT on T13I2C)...
      // ...unless we write something subsequently. So give it a test on normal frequency.
      TWBR=nTWBRorg;
      delay(50);
      Wire.beginTransmission(i2cAdressTest);
      if (Wire.endTransmission() == 0)
        Serial.print(" +");
      else
        Serial.print(" -");
      Serial.println(" NOK!");
      break;
    }
    if((nFreq%16)==0)
      Serial.println("");
  }

  // retest with normal frequency
  TWBR=nTWBRorg;
  delay(100);
  Wire.beginTransmission(i2cAdressTest);
  Serial.print("Retest normal: ");
  Serial.print(i2cCalcFrequencyFromTWBR(TWBR)/1000L);
  Serial.print(" kHz, TWBR=");
  Serial.print(TWBR);
  if (Wire.endTransmission() == 0)
  { // ack received      
    Serial.println(" OK, ");
  }
  else
  { // no ACK
    Serial.println(" NOK!");
  }
  delay(100);

  // retest maximum frequency found and a few levels lower
  #define I2C_NUMTESTFOROK 25
  for(byte nFreq=nFreqMax; nFreq<nFreqMax+10; nFreq++)
  {
    TWBR=nFreq;
    byte nTimesOK=0;
    for(byte n=1; n<=I2C_NUMTESTFOROK; n++)
    {
      Wire.beginTransmission(i2cAdressTest);
      if (Wire.endTransmission() == 0)
        nTimesOK++;
      delayMicroseconds(100);
    }
    //Serial.println("");
    Serial.print("Frequency ");
    Serial.print(i2cCalcFrequencyFromTWBR(nFreq)/1000L);
    Serial.print(" kHz (TWBR=");
    Serial.print(nFreq);
    Serial.print(") tested ");
    Serial.print(nTimesOK);
    Serial.print("/");
    Serial.print(I2C_NUMTESTFOROK);
    Serial.println(" times OK.");
    delay(50);
    if(nTimesOK==I2C_NUMTESTFOROK)
      break;
  }

  // restore original frquency
  TWBR=nTWBRorg;
#else
  Serial.println("Testing frequency using Wire.setClock().");
  Serial.print("CPU Freq: ");
  Serial.print(F_CPU/1000000L);
  Serial.println("MHz.");
#ifdef ESP8266
#if(F_CPU==80000000)
  Serial.println("WARNING: ESP8266 must be set to 160MHz to test speeds > 400Khz!\n");
#endif
#endif
  byte nFreqMax=0;
  //TWBR = 134;    // http://www.gammon.com.au/forum/?id=10896 (on LGT8F328P: 12=400k, 24=300k, 48=200k, 96=130k, 132=100k, 144=94k, 200=70k, 255=57k)
  // Fastest speed measured: TWBR=1 (615 kHz) on Atmel 24C02 Eeprom. Best for T13I2C: TWBR=106 (123kHz) on I2C_Test example T13@9.6MHz
  //TWSR |= bit (TWPS0);    // set prescaler to 4/16/64 for lower speeds (speeds lower than 10k need different timeout settings in TinySoftWire.h)
  //TWSR |= bit (TWPS1);

  delay(100);
  Wire.setClock(100000L);
  Wire.beginTransmission(i2cAdressTest);
  Serial.print("Test normal 100K: ");
  if (Wire.endTransmission() == 0)
  { // ack received      
    Serial.println(" OK, ");
  }
  else
  { // no ACK
    Serial.println(" NOK!");
  }

  // Test various frequencies
  byte nMcuMax=40;
#ifdef ESP8266
  // NOTE: For ESP8266 the set speed is maximized depending the CPU clock.
  // At lower settings the set speed deviates more from actual speed.
  // Eg. at 20k the actual speed is only 14kHz (30% less!)
  #if(F_CPU==80000000)
    nMcuMax=40; //  for ESP8266 @80MHz the max. is  400=390 kHz.
  #else
    nMcuMax=76; // for ESP8266 @160MHz the max. is 760=727-762 kHz.
  #endif
#endif
  
  for(byte nFreq=1; nFreq<nMcuMax; nFreq++)
  {
    Wire.setClock(nFreq*10000L);
    delayMicroseconds(500);
    Wire.beginTransmission(i2cAdressTest);
    Serial.print(nFreq*10);
    Serial.print("K");
    if (Wire.endTransmission() == 0)
    { // ack received      
      Serial.print(" ");
      nFreqMax=nFreq;
    }
    else
    { // no ACK
      // Missing a bit may result in incomplete transmission, timeouts and possibly the device resetting (eg. 1sec WDT on T13I2C)...
      // ...unless we write something subsequently. So give it a test on normal frequency.
      Wire.setClock(100000L);
      delay(500);
      Wire.beginTransmission(i2cAdressTest);
      if (Wire.endTransmission() == 0)
        Serial.print(" +");
      else
        Serial.print(" -");
      Serial.println(" NOK!");
      break;
    }
    if((nFreq%16)==0)
      Serial.println("");
  }

  // retest with normal frequency
  Wire.setClock(100000L);
  delay(100);
  Wire.beginTransmission(i2cAdressTest);
  Serial.print("Retest normal 100K:");
  if (Wire.endTransmission() == 0)
  { // ack received      
    Serial.println(" OK, ");
  }
  else
  { // no ACK
    Serial.println(" NOK!");
#ifdef ESP8266
    Serial.println("Rescan or reset may be needed on ESP8266! Exit now...");
    printMenu();
    return;
#endif
  }
  delay(100);

  // retest maximum frequency found and a few levels lower
  #define I2C_NUMTESTFOROK 25
  for(byte nFreq=nFreqMax; nFreq>nFreqMax-20; nFreq--)
  {
    Wire.setClock(nFreq*10000L);
    delay(50);
    byte nTimesOK=0;
    for(byte n=1; n<=I2C_NUMTESTFOROK; n++)
    {
      Wire.beginTransmission(i2cAdressTest);
      if (Wire.endTransmission() == 0)
        nTimesOK++;
      delayMicroseconds(100);
    }
    //Serial.println("");
    Serial.print("Frequency ");
    Serial.print(nFreq);
    Serial.print("0K (~ ");
    Serial.print(nFreq-(int)((nMcuMax-nFreq)*0.08));
    Serial.print("0 kHz) tested ");
    Serial.print(nTimesOK);
    Serial.print("/");
    Serial.print(I2C_NUMTESTFOROK);
    Serial.println(" times OK.");
    delay(50);
    if(nTimesOK==I2C_NUMTESTFOROK)
      break;
  }

  // restore original frequency
  Wire.setClock(100000L);
  Serial.println("NOTE: actual frequency may be less and may be limited by MCU core.");
#endif
  printMenu();
}
#endif // #if OPT_I2C_FREQTEST

void i2cScan()
{
#if defined(CH32V00x)
  Serial.print("Re-");
  Wire.end();      // sometimes the bus is stuck. SCL is then kept low, SDA too on occasion
  Wire.begin();
#endif // #ifdef CH32V00x
  Serial.println("Scanning ...");
  byte nCount = 0;

  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      if(nCount<MAX_ADDRESSES)
        _arrAddresses[nCount]=i;
      if(_i2cAddressSelected==i)
        Serial.print(">> ");
      else
        Serial.print("   ");
      
      Serial.print(nCount, DEC);
      Serial.print(F(") 0x"));
      Serial.print(i, HEX);
      Serial.print(" (#");
      Serial.print(i, DEC);
      Serial.print(")");

      // Let's see if the device found supports reading DEVICETYPEID
      // First check if we can read the I2C address from the device to see if it understand commands
      // Then read the ID
      _arrDeviceTypeIDs[nCount]=0L;
      i2cWriteU8(i, 0xF3);  // command: READ_I2CADRRESS
      delayMicroseconds(100);       // allow some time
      byte nAddressRead=i2cReadU8(i);
      if(nAddressRead==i)
      {
        _arrIsT13I2C[nCount]=true;
        Serial.print("  @");
        i2cWriteU8(i, 0xFF);  // command: READ_DEVICETYPEID
        delayMicroseconds(100);       // allow some time
        uint32_t uID=i2cReadU32(i);
        if(uID)
        {
          _arrDeviceTypeIDs[nCount]=uID;
          Serial.print(uID, HEX);
          Serial.print(" -> ");
          //Serial.print(P(_szUnknown));
          Serial.print(P(getKnownDeviceType(uID)));
          //Serial.print(P(_arrUnknownDeviceTypeDescriptions[0]));
        }
        delay (1);  // maybe unneeded?
      }
      else
        _arrIsT13I2C[nCount]=false;
      
      nCount++;
      delay (1);  // maybe unneeded?
      Serial.println("");
    } // end of good response
    delayMicroseconds(200);   // For debug purposes we wait some time inbetween, to allow devices to blink a pin and give slow devices some room to return
  } // end of for loop
  Serial.print(F("Found "));
  Serial.print(nCount, DEC);
  Serial.println(F(" device(s)."));
  _nCntAddresses=nCount;

  // reset other addresses
  while(nCount<MAX_ADDRESSES-1)
  {
    _arrAddresses[nCount]=0;
    _arrIsT13I2C[nCount]=false;
    _arrDeviceTypeIDs[nCount]=0L;
    nCount++;
  }

}
uint8_t Serial2Hex()
{
  char ch;
  uint8_t b=0;
  if(Serial.available())
  {
    ch=Serial.read();
    if(ch>='0' && ch<='9')
      b+=(ch-'0');
    else if(ch>='A' && ch<='F')
      b+=(ch-'A'+10);
    else if(ch>='a' && ch<='f')
      b+=(ch-'a'+10);
    return(b);
  } 
  return(0);  
}

uint8_t SerialReadHex()
{ // read serial as a one-by hexadecimal value
  uint8_t b=0;
  if(Serial.available())
    b=Serial2Hex();
  if(Serial.available())
  {
    b<<=4;
    b+=Serial2Hex();
    return(b);
  }
//Serial.print("#");
//Serial.println(b, DEC);
//Serial.print("0x");
//Serial.println(b, HEX);
  return(0);
}

uint32_t SerialReadNum()
{ // read serial as numeric value, ignore non decimal characters
  char ch;
  uint32_t n=0;
  while(Serial.available())
  {
    ch=Serial.read();
    if(ch>='0' && ch<='9')
    {
      n*=10;
      n+=(ch-'0');
    }
  }
  return(n);
}

void SerialPrintHex(byte b)
{   // print a hexadecimal value as two digits
  if(b<16)
    Serial.print("0");
  Serial.print(b, HEX);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  blink(5, 100);
  printMenu();

//  Wire.begin(2,0);   // SDA, SCL => use alternative pins for I2C
  Wire.begin();
  // Default speed setting on LGT8F328P (@32MHz clock) for 100kpbs gives about 90k.
  // Wire.setClock only allows choice between supported speeds (100k and 400k)
  // To tune the speed we can set the TWBR (two-wire-bit-rate) register.
  Wire.setClock(100000L);     // NOTE: when setting a speed that's not supported 400k is used. Only 100k and 400k seem to be supported on LGT8F328P
#if defined(TWBR)
  TWBR = 134;    // http://www.gammon.com.au/forum/?id=10896 (on LGT8F328P: 12=400k, 24=300k, 48=200k, 96=130k, 132=100k, 144=94k, 200=70k, 255=57k)
  //TWSR |= bit (TWPS0);    // set prescaler to 4/16/64 for lower speeds (speeds lower than 10k need different timeout settings in TinySoftWire.h)
  //TWSR |= bit (TWPS1);
#endif
  delay(100);
  //Serial.println("setup done");
  blink(1, 200);
}

void loop()
{
  static uint32_t ulTimer=millis();
  uint32_t uVal=0;
  
  blink(1, 0);
//#if defined(CH32V00x)
//#else
  while(!Serial.available())      // requires fixed Serial.available on CH32V00x
  {
    // while we're waiting let's show some action
    if(millis()-ulTimer>5000)
    {
      if(_nCntAddresses>0)
        blink(_nCntAddresses,200);
      else
        blink(1,10);
      
      ulTimer=millis();
    }
  }
//#endif
  delay(200);   // to allow multiple character input, the serialport needs time to buffer remainder of input before reading the first character

  // handle serial commands  
  char ch=Serial.read();
//Serial.print("ch: 0x");
//Serial.println(ch, HEX);
  switch(ch)
  {
  case 's': case 'S':   // scan addresses
    Serial.println(F("\nS - Scan for devices"));
    printMenu();
    i2cScan();
    break;
  case '0':   // set active address
  case '1': case '2': case '3':
  case '4': case '5': case '6':
  case '7': case '8': case '9':
    Serial.print(F("\nSet active address to 0x"));
    Serial.println(_arrAddresses[ch-'0'], HEX);
    _i2cAddressSelected=_arrAddresses[ch-'0'];
    printMenu();
    i2cScan();
    break;

#if OPT_I2C_FREQTEST
  case 'f': case 'F':   // test frequency
    {
      i2cTestFrequency(_i2cAddressSelected);
    }
    break;
#endif // #if OPT_I2C_FREQTEST

  case 'a': case 'A':   // set address, for verification purposes we send 0=F4, 1=new adress, 2=old address, 3=new adress again.
    {
      Serial.print(F("\nA  - Set Address: "));
      uint8_t b=SerialReadHex();
      Serial.print(b, HEX);
      i2cWriteU32(_i2cAddressSelected, (uint32_t)(((uint32_t)b<<24)|((uint32_t)_i2cAddressSelected<<16)|((uint32_t)b<<8)|(uint32_t)0xF4));    // NOTE: uint32_t is written BIG-endian, so command should be in last byte
      _i2cAddressSelected=b;
      printMenu();
      i2cScan();
    }
    break;
  case '\n': case '\0': case '/':   // menu
    printMenu();
    break;
  case 'd': case 'D':   // data mode
    Serial.println(F("\nD - Set data mode"));
    i2cWriteU8(_i2cAddressSelected, 0xF1);
    break;
  case 'c': case 'C':   // command mode
    Serial.println(F("\nC - Set command mode"));
    i2cWriteU32(_i2cAddressSelected, _uDeviceTypeID);
    break;
  
  case 'i': case 'I':   // get type id
    Serial.print(F("\nI - Get device type id hex code: "));
    i2cWriteU8(_i2cAddressSelected, 0xFF);
    // allow some time
    delayMicroseconds(100);
    _uDeviceTypeID=i2cReadU32(_i2cAddressSelected);
    Serial.print("0x");
    Serial.println(_uDeviceTypeID, HEX);
    break;

  case '?':   // read four bytes
    Serial.print(F("\n?  - Read 4-byte value: "));
    Serial.print("0x");
    uVal=i2cReadU32(_i2cAddressSelected);
    Serial.print(uVal, HEX);
    Serial.print(F(" [d"));
    Serial.print(uVal, DEC);
    Serial.println(F("]"));
    break;    
  case 'r': case 'R':   // read n bytes
    {
      uint32_t n=SerialReadNum();
      Serial.print(F("\nR  - Read "));
      Serial.print(n, DEC);
      Serial.println(F(" bytes: "));
      for(int i=0; i<n; i++)
      {
        if((i%16)==0) Serial.print("\n0x ");
        SerialPrintHex(i2cReadU8(_i2cAddressSelected));
        Serial.print(" ");
        delayMicroseconds(100);  // allow some time
      }
      Serial.println(".");
    }
    break;
  case 't': case 'T':   // write text bytes
    {
      Serial.print(F("\nT  - Write text-byte(s): "));
      while(Serial.available())
      {
        ch=Serial.read();
        Serial.write(ch);
        i2cWriteU8(_i2cAddressSelected, ch);
        delayMicroseconds(200);  // allow some time for the device to forward the data
      }
      Serial.println("");
    }
    break;
  case 'v': case 'V':   // write a series of hex byte(s)
    {
      Serial.print(F("\n"));
      Serial.print(F("V")); 
      int nCnt=0;
      bool fAckReceived=true;   // check ACK/NACK when sending data
      Wire.beginTransmission(_i2cAddressSelected);
      while(Serial.available())
      {
        nCnt++;
        uint8_t b=SerialReadHex();
        SerialPrintHex(b);
        Wire.write(b);      // Note: Arduino I2C buffer is max 32 bytes

        // allow some time
        //delayMicroseconds(100);
      }
      fAckReceived=(Wire.endTransmission()==0);      // ==0 means ack received, !=0 means nack or other error
      if(!fAckReceived)
      {
        Serial.print(F(" failed"));
        blink(10,10);     // signal error to easily spot in logic analyzer
      }
    }
    break;
  case 'w': case 'W':   // write 1/2/4 hex byte(s)
  case 'x': case 'X':   // write 1/2/4 hex byte(s) and read reply
    {
      Serial.print(F("\n"));
      if(ch=='w' || ch=='W')
        Serial.print(F("W")); 
      else 
        Serial.print(F("X"));
      Serial.print(F("  - Write 1/2/4 hex-byte(s): "));
      //Serial.print("0x");
      int nCnt=0;
      bool fAckReceived=true;   // check ACK/NACK when sending data
      while(Serial.available())
      {
        nCnt++;
        //Serial.print(nCnt, DEC);
        //Serial.println(":");
        uint8_t b=SerialReadHex();
        SerialPrintHex(b);
        if(!Serial.available())
          fAckReceived=i2cWriteU8(_i2cAddressSelected, b);
        else
        {
          uint8_t b2=SerialReadHex();
          SerialPrintHex(b2);
          if(!Serial.available())
            fAckReceived=i2cWriteU16(_i2cAddressSelected, ((uint16_t)b2<<8)|b);
          else
          {
            uint8_t b3=SerialReadHex();
            SerialPrintHex(b3);
            if(!Serial.available())
              fAckReceived=i2cWriteU32(_i2cAddressSelected, ((((uint32_t)b3<<8)|b2)<<8)|b);
            else
            {
              uint8_t b4=SerialReadHex();
              SerialPrintHex(b4);
              fAckReceived=i2cWriteU32(_i2cAddressSelected, ((((((uint32_t)b4<<8)|b3)<<8)|b2)<<8)|b);
            }
          }
        }
        if(!fAckReceived)
        {
          Serial.print(F(" failed"));
          blink(10,10);     // signal error to easily spot in logic analyzer
        }
        // allow some time
        delayMicroseconds(100);
      }
      if(ch=='x' || ch=='X')
      { // read back 4-byte result after sending data
        if(!fAckReceived)
          Serial.print(F(" (no ACK received)"));
        else
        {
          delay(100);    // give the I2C device some time to collect data (NOTE: 50 mSec could be too short)
          Serial.print(F("  => 0x"));
          uVal=i2cReadU32(_i2cAddressSelected);
          Serial.print(uVal, HEX);
          Serial.print(F(" [d"));
          Serial.print(uVal, DEC);
          Serial.print(F("]"));
        }
      }
      Serial.println("");
    }
    break;
  default:
    blink(1,200);
    delay(800);
    printMenu();
    delay(1000);
    break;
  }
  
  // read unprocessed extra commands
#if defined(CH32V00x)
  while(Serial.available() >0)
#else
  while(Serial.available())
#endif
    Serial.read();
}
