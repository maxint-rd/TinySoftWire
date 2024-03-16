/*
    ATtiny13A I2C ADC example.
    Turn the ATtiny13A into an I2C device that properly responds to an I2C controller. 
    This ADC implementation features 4 separate ADC channels. 
    Channel 0 (on the reset pin) can be used as voltage reference.

    ATtiny13A offers the following support for ADC (see datasheet ch.14):
      * 10-bit Resolution
      * 0.5 LSB Integral Non-linearity
      * ± 2 LSB Absolute Accuracy
      * 13 - 260 μs Conversion Time
      * Up to 15 kSPS at Maximum Resolution
      * Four Multiplexed Single Ended Input Channels
      * Optional Left Adjustment for ADC Result Readout
      * 0 - VCC ADC Input Voltage Range
      * Selectable 1.1V ADC Reference Voltage
      * Free Running or Single Conversion Mode
      * ADC Start Conversion by Auto Triggering on Interrupt Sources
      * Interrupt on ADC Conversion Complete
      * Sleep Mode Noise Canceler      

    I2C support (using TinySoftWire library):
      - scanning for ACK using write to address
      - writing up to 4 bytes (I2C_BUFFER_LENGTH 4), acknowledgment on every byte
      - reading up to 4 bytes (I2C_BUFFER_LENGTH 4), last byte not acknowlesdged by controller
      - detect (re)start/stop conditions
    For documentation of the I2C protocol see https://www.nxp.com/docs/en/user-guide/UM10204.pdf
      
    T13I2C protocol:
      Using the TinySoftWire library an ATtiny13A can receive I2C data.
      Commonly a T13I2C device accepts commands and data. The supported commands depend on the device, 
      but may include the following:
        0xF1  - switch to data mode: no further command processing*
        0xF3  - get the I2C-address (can be used to verify T13I2C protocol support)
        0xF4  - change the I2C address and store it in EEPROM
        0xF6  - reset the I2C address to default device address
        0xFF  - get the T13I2C device type ID (4-bytes)
    *) When in data mode sending the device type ID to the device switches back to command mode.
    
    T13I2C ADC commands:
      0x00 - read ADC channel 0 (pin 1, ADO0/RST), recommended to use TL431 for 2.5V reference.
      0x01 - read ADC channel 1 (pin 7, ADC1/PB2)
      0x02 - read ADC channel 2 (pin 3, ADC2/PB4)
      0x03 - read ADC channel 3 (pin 2, ADC3/PB3)
      0x04 - get VCC, based on reading voltage reference on ADC0
      0xF3 - get the I2C-address (can be used to verify T13I2C protocol support)
      0xF4 - change the I2C address and store it in EEPROM
      0xFB - set refence level in mV*100 to get calculated mV readings. 0xFB19=2500mV. Set to 0 for raw readings.
      0xFC - get reference level in mV*100. Default is 25 (0x19).
      0xFF - get the T13I2C device type ID (4-bytes)
    Note: the reference value set using 0xFB is not stored in EEPROM (not enough FLASH)

    Tested with ATtiny13A @ 9.6MHz, board MicroCore/ATtiny13 Arduino IDE 1.8.12: 10 bytes RAM, 1020 bytes Flash
    Made by MAXINT R&D 2023. See github.com/maxint-rd for more projects
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
// I2C_SCL 1
// 

// Pinout T13 ADC implementation (top-view, following standard T13I2C pinout I2C_SDA=0, I2C_SCL=1)
//                                +---v---+
//              RESET/ADC0/AREF --|1     8|-- VCC
//                         ADC3 --|2     7|-- ADC1
//                         ADC2 --|3     6|-- SCL
//                          GND --|4     5|-- SDA
//                                +-------+
//
// Using TL431 as 2.5V reference (bottom-view):
//            /-----|
//           /   o--+--(Cathode)--ADC0---+
//          |    o--+---(Anode)---GND    +---[1K]--- VCC
//           \   o--+----(Ref)----ADC0---+
//            \-----|
// Note: since !RESET has an internal pullup, the [1K] resistor is optional. Testing confirmed this.
//
// Note2: when using a voltage divider to allow reading voltages higher than VCC in combination with 
// the internal pullup (Rp), the pullup will cause higher readings than without.
//           GND---R1--ADC--R2---Vm
//                      +---Rp---Vcc
// To allow proper readings the voltage divider should use resistors (R1+R2) that are low enough compared to Rp
// Testing with a 1K-9.1K voltage divider indicated an internal pullup of 23.4 kOhm.

#include <TinySoftWire.h>
TinySoftWire myWire=TinySoftWire();

#include <EEPROM.h>
#define I2C_ADDRESS 0x3C
byte _i2cAddress=I2C_ADDRESS;
#define DEVICETYPEID 0xADC3C005

//-------------------------------------------------------
//
// Handling of T13I2C device commands
//

//byte _nActiveChannel=0;     // 0=ADC0/!Reset
byte _nVoltageReference=25;   // voltage level of reference on ADC0 [mV*100]. Use command FB to set level and read calculated voltages. Set to 0 to get raw readings.

#define digitalWriteHigh(P) {PORTB |= _BV(P);}

uint16_t getV(byte nChannel)
{ // measure voltage
  // note: MicroCore for ATtiny13A uses analog portnumbers 0-3 for analogRead(), these are different than the digital numbers
  uint16_t nOld;
  uint16_t nAna=1024;  //when we have a voltage reference, VCC can also be calculated by reading ADC0 and using the maximum ADC value (1024)

/* NOT ENOUGH FLASH
  if(nChannel>3)    // channels 0-3 for ATtiny13A
    return(0);
*/

/* NOT ENOUGH FLASH
  // First wait until the reading is stable
  nAna=analogRead(nChannel);
  do {
    nOld = nAna;
    nAna = analogRead(nChannel);
  }  while (abs(nAna - nOld) > 3);
*/

  // Due to lack of flash, we skip the first instead of waiting for stable reading
  // then read NUM_ANALOGREAD times to take the average
#define NUM_SKIP 5
#define NUM_ANALOGREAD 10
  if(nChannel<4)
  {
    nAna=0;
    for(byte n=0; n<NUM_ANALOGREAD+NUM_SKIP; n++)
    {
      if(n>=NUM_SKIP)
        nAna+=analogRead(nChannel);
      delay(1);
    }
    nAna=nAna/NUM_ANALOGREAD;   // avarage range 0-1023
  }

  if(_nVoltageReference>0)
  {   // calculate mV's using reference voltage on ADC0
    //analogRead(0);
    //delay(1);
    nOld=analogRead(0);
    uint32_t uVal=nAna;
    uVal=uVal*_nVoltageReference*100;
    uVal=uVal/nOld;
    nAna=(uint16_t)uVal;
    //nAna=(((uint32_t)nAna*(uint32_t)nOld)/_nVoltageReference);
  }
  
  return(nAna);
}

void i2cAfterWrite()
{ // This functions handles what is done after we received some data
  
  // Let's treat first byte as a command
  byte nVal=myWire.getDataU8(0);

  if(nVal<5)
  { // Commands 0x00, 0x01, 0x02 and 0x03 are used to read the specific ADC channel
    // When a voltage reference is used, the command 0x04 is used to calculate VCC
    myWire.setDataU32(0, getV(nVal));
    return;
  }

  switch(nVal)
  {
/*
  case 0x00:  // set active ADC-channel
  case 0x01:  // set active ADC-channel
  case 0x02:  // set active ADC-channel
  case 0x03:  // set active ADC-channel
    //_nActiveChannel=nVal;  // 5=VCC internal voltage
    //myWire.setDataU32(0, getV(_nActiveChannel));     // NOTE: using setDataU8 uses less flash than setDataU32
    myWire.setDataU32(0, getV(nVal));     // NOTE: using setDataU8 uses less flash than setDataU32
    break;
*/
  case 0xF3:  // get I2C address in next read (can be used to verify device has command support)
    myWire.setDataU8(0, _i2cAddress);     // NOTE: using setDataU8 uses less flash than setDataU32
    break;
  case 0xF4:  // set I2C address
    _i2cAddress=myWire.getDataU8(1);
    myWire.setAddress(_i2cAddress);
    // store address to EEPROM
    EEPROM.write(0, _i2cAddress);
    break;
/* if not enough flash, comment this code out 
  case 0xF6:  // reset I2C address to default
    _i2cAddress=I2C_ADDRESS;
    myWire.setAddress(_i2cAddress);
    // store address to EEPROM
    EEPROM.write(0, _i2cAddress);
    break;
*/

  case 0xFB:   // set value of reference voltage (25=2500mV), when set the ADC values are returned as mV
    _nVoltageReference=myWire.getDataU8(1);
    break;
  case 0xFC:   // get value of set reference voltage
    myWire.setDataU32(0, _nVoltageReference);
    break;
  case 0xFF:   // get the device type id
    myWire.setDataU32(0, DEVICETYPEID);
    break;
  }
}

void setup()
{
  // Enable pullup on ADC0/!RESET to get VCC on the pin connected to the voltage reference.
  // This allows using the TL431 as 2.5V reference without other components.
  // Note: the digitalWriteHigh() macro uses 2 bytes less flash per call than using pinMode(PB2, INPUT_PULLUP);
  digitalWriteHigh(5);     // ADC0/!RESET
/*
  digitalWriteHigh(2);     // ADC1
  digitalWriteHigh(3);     // ADC3
  digitalWriteHigh(4);     // ADC2
*/

  // read I2C address from EEPROM byte 0
  byte address = EEPROM.read(0);
  _i2cAddress=(address!=0xFF) ? address : I2C_ADDRESS;
  myWire.begin(_i2cAddress);

  // NOT ENOUGH FLASH to store this setting in EEPROM
  //_nVoltageReference = EEPROM.read(1);

  // Set voltage reference. On ATtiny13A in MicroCore INTERNAL is same as INTERNAL1V1 [1]
  // Not setting analogReference() or using EXTERNAL [0] implies readings are relative to VCC
  // ATtiny13A doesn't have a true external reference and Vcc/Vref cannot be selected as input for analogRead()
  // Recommended is use of TL431 voltage reference on ADC0, which also is the !RESET pin 
  //analogReference(INTERNAL);

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
  // Note: Whatever else is done in the loop besides process() should be done fast in order not to miss subsequent I2C transactions
  if(myWire.process()>0)
  {
    if(myWire.getLastStatus()==I2C_STATUS_WRITE)
      i2cAfterWrite();
  }
}
