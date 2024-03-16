/*
    TinySoftWare ATtiny13A I2C peripheral library for bitbanging I2C.
    Turn the ATtiny13A into an I2C device that properly responds to an I2C controller.
    Using this library the ATtiny13A at 9.6MHz can perform 100kbps I2C communication.
    It can receive write commands and respond to read data-requests for the set I2C address.

  	A description of the protocol (simplified) can be found below
    For full documentation of the I2C protocol see https://www.nxp.com/docs/en/user-guide/UM10204.pdf
    For more info and an I2C implementation in BascomAVR/assembly see Elektor 2009-01 page 52-54.
        (https://www.elektormagazine.nl/magazine/elektor-200901/15674/)

    Notes:
      - Written fully in C++ as sufficient assembly skills were lacking.
        Better performance may be obtained using assembly.
      - Unfortunately interrupts seem to be too slow on the ATtiny13A @ 9.6MHz, requiring continuous
        calls to TinySoftWire::process()
      - Using this library the ATtiny13A at 9.6MHz can perform 100kHz I2C communication.
        Recognizing the address and acking after clock-pulse 8 now takes 4.5us.
        At 100Kbps each pulse is only 5us, so 400 kHz is much too fast. 
      - No clock-stretching was implemented (SLC is not kept low while prepping data)
      - General call write/read at address 0x00 is not implemented

    Made by MAXINT R&D 2020-2024. See github.com/maxint-rd for more projects.
*/

//   Pinout ATtiny13A
//                                +---v---+
//   (PCINT5/RESET/ADC0/dW) PB5 --|1     8|-- VCC
//       (PCINT3/CLKI/ADC3) PB3 --|2     7|-- PB2 (SCK/ADC1/T0/PCINT2)
//            (PCINT4/ADC2) PB4 --|3     6|-- PB1 (MISO/AIN1/OC0B/INT0/PCINT1)
//                          GND --|4     5|-- PB0 (MOSI/AIN0/OC0A/PCINT0)
//                                +-------+
//

//   Pinout AT24C32 I2C EEPROM (Used as model reference)
//                                +---v---+
//                           A0 --|1     8|-- VCC
//                           A1 --|2     7|-- WP
//                           A2 --|3     6|-- SCL
//                          GND --|4     5|-- SDA
//                                +-------+

//
// Solving I2C communication problems
// - The relatively slow processing of the ATtiny13A may cause lost clock pulses and data changes.
//   Most critical is sending back the acknowledgement in time. 
//   Sometimes pull-up resistors on the I2C lines can help, but in my experiments I found adding capacitors
//   much more helpful. Place low value capacitors (<400pF, see table 10 of I2C specs) between SDA/SCL and ground.
//   Effectively this delays the signals a bit, 47pF worked fine for me.
//
// - Alternatively one can change the speed of the I2C communication. Using Wire.setClock() may be limited to few supported speeds.
//   On the Atmel chips such as the ATmega328P (and LGT8F328P) one can set the TWBR/TWPS registers. (Higher TWBR values result in lower speeds).
//   See for more info this fine post by Nick Gammon: http://www.gammon.com.au/forum/?id=10896 
//   Note that the time-outs set in the library prevent support of very low speeds (below 10k)



// I2C Slave Protocol (simplified)
// ===============================
// START CONDITION: 
//     - SDA goes low while SCL is high.
// STOP CONDITION: 
//     - SDA goes high after SCL goes high.
//
// DATA TRANSMISSIONS:
//  >> I2C ADDRESS:
//     - SCL goes up 8 times, Master sends address (7-bits) plus bit 0=WRITE/1=READ  
//     - ACK: slave sends ack by pulling SDA low on 9th clock (NACK is keep SDA high)
// 
//  >> WRITE:
//     - Master sends data byte 1 (8 bits + ack)
//     - optional: Master sends subsequent data bytes (8 bits + ack)
// 
//  >> READ:
//     - optional: Master sends data identification byte (8 bits + ack)
//     - slave writes data on clock going up, sends no ack
//
// I2C ADDRESSES:
//     - valid range 0x08 - 0x77  (8-119)
//     - special addresses:  0x00 - R: general call, W: START byte
//
//
//

#include "TinySoftWire.h"

/* Microcore 1.06 for ATtiny13A already includes fast reading and writing in digitalRead and digitalWrite and has optimization on/
   The macros below give same Flash usage and show equal timing, so we can assume the generated code to be identical.
// Macros to speed up things and to minimize flash usage.
// NOTE: used pins match port B, only tested with ATtiny13, could also work on other ATtiny's
#define digitalPinToPortReg(P) &PORTB
#define digitalPinToDDRReg(P) &DDRB
#define digitalPinToPINReg(P) &PINB
#define __digitalPinToBit(P) (P)
#define __digitalPinToTimer(P) (((P) ==  0) ? &TCCR0A : &TCCR0B
#define __digitalPinToTimerBit(P) (((P) ==  0) ? &COM0A1 : &COM0B1 
#include <digitalWriteFast.h>
//#define digitalReadFast2(P) ((PINB & _BV(P)) == _BV(P))
*/

/**/
#define pinModeInput(P) {DDRB &= ~_BV(P);}
#define pinModeOutput(P) {DDRB |= _BV(P);}
#define digitalWriteHigh(P) {PORTB |= _BV(P);}
#define digitalWriteLow(P) {PORTB &= ~_BV(P);}
#define pinModePullup(P) {DDRB &= ~_BV(P); PORTB |= _BV(P);}
#define digitalReadPin(P) ((PINB & _BV(P)) == _BV(P))
/**/

#if(OPT_DEBUG_BLIP)
// LED_BUILTIN is defined as pin 2 on MicroCore board ATtiny13
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#endif


TinySoftWire::TinySoftWire(void)
{	// constructor
}

void TinySoftWire::begin(byte address)
{	// begin method to be called in constructor
#if(OPT_DEBUG_BLIP)
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  //pinMode(I2C_SDA, INPUT);
  //pinMode(I2C_SCL, INPUT);
  pinModeInput(I2C_SDA);
  pinModeInput(I2C_SCL);

  //_i2cAddress=address;
  _i2cAddressWrite=address<<1;
  
#if OPT_ISR
  // Setup interrupt to detect change in SDA.
  // Experiments showed that using pin change interrupts is too slow to detect
  // pin changes and respond accordingly in time to catch the subsequent data stream.

  GENERAL_INTERRUPT_MASK |= 1 << PIN_CHANGE_INTERRUPT_ENABLE; // enable Pin Change Interrupt
  GENERAL_INTERRUPT_FLAGS  |= 1 << PIN_CHANGE_FLAG; // clear interrupt flag, by writing 1 to it
  PIN_CHANGE_MASK |= 1 << I2C_SDA; // enable pin change interrupt on PCINT0 (SDA line)
  // MCUCR ISC01 ISC00 = 0 1 
  // SEI()
#endif
}

#if OPT_ISR
// Interrupt Routine, triggered when SDA pin changes; for detecting Stop condition
bool _fStoppedISR=false;
bool _fRestartISR=false;

ISR( PCINT0_vect )
{
	byte bPins=PINB;
	if(bPins & ( 1 << I2C_SCL ))
	{	// stop or restart condition occured
		_fStoppedISR=(bPins & (1 << I2C_SDA));
		_fRestartISR=!(bPins & (1 << I2C_SDA));
		//blip();
  }
  else
  {
  	_fStoppedISR=false;
  	_fRestartISR=false;
  }
}
#endif

#if(OPT_CHANGE_ADDRESS)
void TinySoftWire::setAddress(byte address)
{	// change the I2C address
  //_i2cAddress=address;
  _i2cAddressWrite=address<<1;
}

uint8_t TinySoftWire::getAddress(void)
{
	//return(_i2cAddress);
	return(_i2cAddressWrite>>1);
}
#endif

#if(OPT_DEBUG_BLIP)
void TinySoftWire::blip(byte cnt)
{ // Blip to flash LED for debugging
  // LED_BUILTIN flash duration without delay is 0.19 us, doing function call about 0.25us
  while(cnt) // loop adds 0.5us to call 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);     // LED_BUILTIN flash duration without delay is 0.2 us
    cnt--;
  }
}
#else
	#define blip(x)
#endif

bool TinySoftWire::i2cWaitForStart(void)
{
  //
  // START CONDITION: SDA going from high to low while SCL is high
  //
#if OPT_ISR
	while((PINB&_BV(I2C_SCL))==_BV(I2C_SCL))
	{
		if(_fRestartISR)
		{
		  blip(3);
		  return;
		}
  }
#else
	uint16_t uTimeOut=I2C_START_TIMEOUT;

waitforstart:
  // wait for SCL&SDA=1
  while(uTimeOut && ((PINB & (_BV(I2C_SDA) | _BV(I2C_SCL))) != (_BV(I2C_SDA) | _BV(I2C_SCL))))      // simultaneous read of SDA=PINB0 and SCL=PINB1
  {
  	uTimeOut--;			// decreasing counter within loop optimizes smaller code than doing that as part of while conditon!  (542->512 = 30 bytes saved!)
  }

  // Wait for SCL=1,SDA=0
  // Go back to start when SCL goes low before SDA does
  // NOTE: for speed no instructions are allowed inbetween detection of 1-1 -> 1-0
  while(uTimeOut && ((PINB & (_BV(I2C_SDA) | _BV(I2C_SCL))) != _BV(I2C_SCL)))      // simultaneous read of SDA=PINB0 and SCL=PINB1
  {
    if(!(PINB & _BV(I2C_SCL))) goto waitforstart; // SLC goes low, this is no START but data
  	uTimeOut--;
  }
  // detection of START on ATtiny13A@9.6Mhz: 0.8-1.5 us after SDA going low
  
  // now wait for SLC=0 to allow subsequent receive to assume SCL is low
  while(uTimeOut && ((PINB & _BV(I2C_SCL))==_BV(I2C_SCL)))
  {
  	//blip();
  	uTimeOut--;
  }

  if(uTimeOut==0L)
  {
  	blip(5);
  	_i2cStatus=I2C_STATUS_NOSTART;
  }

  return(uTimeOut>0L);
#endif
}

bool TinySoftWire::i2cWaitForLowClock(void)
{
  // wait for SCL=0
#if OPT_ISR
	while((PINB&_BV(I2C_SCL))==_BV(I2C_SCL))
	{
		if(_fStoppedISR || _fRestartISR)
		{
			_fStoppedI2C=_fStoppedISR;
			_fRestartI2C=_fRestartISR;
			
			return(false);
		}
	}
	return(true);
#else	
	// While waiting for low SLC a STOP or START can occur. Eg. when no bytes follow the address.
	// For instance the I2C scanner sends address to check for ACK, and immidiately stops.
  // NOTE: detecting transitions of SDA (for repeated START or for STOP) takes about 4 us (requires at least two SDA reads)
  // This delay gives a later setting of the ACK, but still before the rising edge of the ninth SCL pulse (@100kbps I2C).
  _fStoppedI2C=false;
  _fRestartI2C=false;
  byte bPinsBefore=PINB;
  byte bPinsNow=bPinsBefore;

  while((bPinsNow&_BV(I2C_SCL))==_BV(I2C_SCL))			// while clock is high
  {
    // If SDA changes while SLC is still high, we may have a start/stop condition.
    // If SDA goes high while SLC is still high, we have a STOP.
    // If SDA goes low while SLC is still high, we have an unexpected repeated start
    // If this happens while receiving the address, it is unexpected.
    // If this happens while waiting for the first bit of a data byte, it means there is no further data
    // If this happens while waiting for a next bit of a data byte, it means an unexpected stop of data being sent
    if((bPinsNow&_BV(I2C_SDA))!=(bPinsBefore&_BV(I2C_SDA)))      // checking transition using PINB faster?
    { // We have SDA transition; now figure out what kind of transition we have
      if((bPinsBefore&_BV(I2C_SDA))!=_BV(I2C_SDA) && (bPinsNow&_BV(I2C_SDA))==_BV(I2C_SDA))
      { // SDA 0->1 STOP! (wait for next start)
        _fStoppedI2C=true;
/* blip only for debugging, requires controller to use delays of many microseconds in between write/read
        blip(2);
        blip(3);
*/
        return(false);    // false means while waiting stop or start was received
      }
      else if((bPinsBefore&_BV(I2C_SDA))==_BV(I2C_SDA) && (bPinsNow&_BV(I2C_SDA))!=_BV(I2C_SDA))
      { // SDA 1->0 START! (unexpected repeated start)
        _fRestartI2C=true;
        blip(3);
        blip(2);
        return(false);    // false means stop or start was received while waiting
      }
    }
    bPinsNow=PINB;
  }
  return(true);
#endif
}


bool TinySoftWire::i2cSendAck(void)
{
  // ACK: 
  //   Keep SDA low for the duration of the ninth SCL pulse
  //   pull SDA low until SCL is low after that pulse
  // To behave similar as other I2C devices, we could pull SDA low directly after reading databit 8, possibly while SCL is still high.
  // Unfortunately we're not fast enough to do that. Instead we assume previous receive/send waited for SCL=low after sending each bit.
  // 

  // Send ACK for the whole duration of the ninth SCL pulse, assume previous receive/send waited for SCL=low after sending each bit.
  //pinMode(I2C_SDA, OUTPUT);
  //digitalWrite(I2C_SDA, LOW); 

  pinModeOutput(I2C_SDA);
  digitalWriteLow(I2C_SDA); 


  // Wait for SCL to go high (beginning of ninth pulse)
  //while(!digitalRead(I2C_SCL));
  //while(!digitalRead(I2C_SCL))  blip();
  while(!digitalReadPin(I2C_SCL))  blip();

  // wait for SCL to go low (end of ninth pulse)
  // If SDA goes high while SLC is high, we have an unexpected stop
  // However, since we're pulling SDA low, this cannot be detected during the ninth pulse.
  // If we miscounted bits, we may end up acking the stop-pulse instead of pulse 9, while SCL stays up 
  byte nCntTimeout=0;
  //while(digitalRead(I2C_SCL))
  while(digitalReadPin(I2C_SCL))
  {
	  if(++nCntTimeout>I2C_CLOCK_TIMEOUT)		// using millis() for time-out is slow and uses more memory
	  {	// Time-out! If ack times out and SDA is released, SDA is pulled up in high clock, which stands for STOP condition
		  //pinMode(I2C_SDA, INPUT);
		  pinModeInput(I2C_SDA);
		  _fStoppedI2C=true;
		  blip(5);
		  blip(5);
		  blip(5);
		  return(false);		// ACK kind-of failed
	  }
  }

  // end our ACK
  //pinMode(I2C_SDA, INPUT);
  pinModeInput(I2C_SDA);
  //pinModePullup(sda);

  return(true);		// ACK succeeded!
}

bool TinySoftWire::i2cReceiveAck(void)
{  // See if controller has acknowledged our send.
  // Note that the controller may not send an ACK on the last byte (see I2C specs 3.1.10 "Immediate read").
  // Acknowledge means controller is expecting more data; beware of STOP/RESTART conditions.
  // Wait for SCL to go high (ninth pulse)
  byte nPins;
  do
  {
  	blip();
  	nPins=PINB;
 	} while((nPins&_BV(I2C_SCL))!=_BV(I2C_SCL));
  
  // Read SDA, should be low for ack, if high it may be the last read in the sequence
  // To properly read the ACK we need to read both pins and check SDA at time of SLA high
  bool fAck=nPins&_BV(I2C_SDA);

	// wait for SCL low complete ACK/NACK and be ready for next i2cSendByte
  if(!i2cWaitForLowClock())		// after sending ACK SCL should already be low
    return(true);    // true means no ack or stopped/restart

  return(fAck);
}

/*
// i2cReceiveByte optionally with ack is somewhat slower when ack is not used, but faster when fSendAck==true
// 4.25us-4.38 for address, 2.50-2.75 for cmd  (time between end eight pulse and start of ack)
// sketch is also larger i2c_test: 1014 instead of 1004.
byte TinySoftWire::i2cReceiveByte(bool fSendAck)
{ // Function to receive a byte from the master
  // Assume startbit was already sent and SCL became low from the START pulse, or from the ACK we sent after receiving a byte previously.
  byte nByteReceived=0;
  byte nBit=8;
  
  do
  //for(byte nBit=0; nBit<8; nBit++)
  {
    // wait for SCL=1
    while(!digitalRead(I2C_SCL));

    // read databit as soon as SCL=1
    nByteReceived<<=1;
    nByteReceived|=digitalRead(I2C_SDA);

		// wait for low to be ready for next bit
    if(!i2cWaitForLowClock())		// after sending ACK SCL should already be low
      return(0);    // 0 is only really 0 if _fRestartI2C and _fStoppedI2C are false

  }
  while(--nBit);    // faster than for?

	if(fSendAck)
		if(!i2cSendAck())
		  return(0);    // 0 is only really 0 if _fRestartI2C and _fStoppedI2C are false

  return(nByteReceived);
}
*/

// making fSendAck separate function call instead of optional parameter is faster when ack is not used, but slower when fSendAck==true
// 3.63us-3.93us for address, 3.25us-3.5us for cmd (time between end eight pulse and start of ack)
// sketch is smaller without using parameter; i2c_test: 1004 instead of 1014.
byte TinySoftWire::i2cReceiveByte()
{ // Function to receive a byte from the master
  // Assume startbit was already sent and SCL became low from the START pulse, or from the ACK we sent after receiving a byte previously.
  byte nByteReceived=0;
  byte nBit=8;
  
  do
  //for(byte nBit=0; nBit<8; nBit++)
  {
    nByteReceived<<=1;

    // wait for SCL=1
    while(!digitalRead(I2C_SCL));

    // read databit as soon as SCL=1
    //nByteReceived<<=1;
    nByteReceived|=digitalRead(I2C_SDA);

		// wait for low to be ready for next bit
    if(!i2cWaitForLowClock())		// after sending ACK SCL should already be low
      return(0);    // 0 is only really 0 if _fRestartI2C and _fStoppedI2C are false

  }
  while(--nBit);    // faster than for?

  return(nByteReceived);
}


byte TinySoftWire::i2cReceiveByteSendAck()
{
	byte nByteReceived=i2cReceiveByte();
	if(!i2cSendAck())
	  return(0);    // 0 is only really 0 if _fRestartI2C and _fStoppedI2C are false
  return(nByteReceived);
}
/**/

/*
// Trying to speed up first ack by combining things wasn't succesful at first attempt.
// TODO: try again later...
bool TinySoftWire::i2cReceiveAddress(void)
{ // Function to receive a byte from the master
  // Assume startbit was already sent and SCL became low from the START pulse, or from the ACK we sent after receiving a byte previously.
	byte btAddressWrite=_i2cAddress<<1;		// to speed-up ack-time
  byte nByteReceived=0;
  byte nBit=8;
  
  do
  //for(byte nBit=0; nBit<8; nBit++)
  {
    // wait for SCL=1
    while(!digitalRead(I2C_SCL));

    // read databit as soon as SCL=1
    nByteReceived<<=1;
    nByteReceived|=digitalRead(I2C_SDA);

		// wait for low to be ready for next bit
    if(!i2cWaitForLowClock())		// after sending ACK SCL should already be low
      return(0);    // 0 is only really 0 if _fRestartI2C and _fStoppedI2C are false

  }
  while(--nBit);    // faster than for?

	if((nByteReceived&0xFE)!=btAddressWrite)
		return(0);		// not our address

	if(!i2cSendAck())
	  return(0);    // 0 is only really 0 if _fRestartI2C and _fStoppedI2C are false

  return(nByteReceived);
}
*/

bool TinySoftWire::i2cSendByte(byte bData)
{ // Send data after receiving a read request; it should always follow an ACK
  // Assume SCL to have gone low after the previously sent ACK
  // Set each bit when SCL is low. Keep the bit set as long as SCL remains high
  byte nBit=8;

  // set SCL to output low and thus stretch it a bit if needed, we're now in control. (TODO: check if bus is free?)
  //digitalWrite(I2C_SCL, LOW);
  //pinMode(I2C_SCL, OUTPUT);

  // set SDA to output, we're now in control. (TODO: check if bus is free?)
  pinModeOutput(I2C_SDA);
  //pinMode(I2C_SDA, OUTPUT);
  //pinMode(I2C_SCL, INPUT);

  do
  {
    // set databit as soon as SCL=0, high bit first
    digitalWrite(I2C_SDA, bData&_BV(7));
    bData<<=1;

    // wait for SCL=1
    while(!digitalRead(I2C_SCL));

    // wait for SCL=0
    // If SDA goes high while SLC is high, we have an unexpected stop
    // When SDA goes from high to low is a an unexpected repeated start
    // However, since we're controlling SDA, this can't happen.
    while(digitalRead(I2C_SCL));
  }
  while(--nBit);    // faster than using a for-loop?

  // set SDA back to input
  //pinMode(I2C_SDA, INPUT);
  pinModeInput(I2C_SDA);

	return(i2cReceiveAck());
}

//
// Public methods
//

//uint32_t fnCallBack(bool fRead, byte nBytesProcessed, uint32_t uData);

byte TinySoftWire::process()
{	// receive some bytes or send a response
	// return the number of bytes received/responded
	//byte btAddressWrite=_i2cAddress<<1;		// to speed-up ack-time
	
  //*_i2cBuffer=0L;    // set buffer empty
  if(!i2cWaitForStart())
  	return(0);

	// Receive the first byte, this should be the address, so we check _i2cAddress before sending ACK
	// We need to send our ACK as soon as we can. 
	// The current implementation has only 1us or less (0.875-1us) between actually setting the ack and getting the rising edge of the ninth pulse
	// When less than 0.5us the ACK will be set to late. In that case a 47pF cap between SCL and GND may help to get just a bit more room...
  byte _nBufferIndex=0;
  byte b=i2cReceiveByte();
  //byte b=i2cReceiveByte(false);
  //byte b=i2cReceiveAddress();
  //if(b>>1 == _i2cAddress)
  //if((b&0xFE) == btAddressWrite)		// using AND seems 0.5 us faster than using >>1
  if((b&0xFE) == _i2cAddressWrite)		// using AND seems 0.5 us faster than using >>1
  //if(b)
  {
    i2cSendAck();		// only send ack for our own address

    // if action is read, then send requested data, otherwise read subsequent bytes until STOP
    bool fControllerRead=b&0x01;      // fControllerRead is true when controller wants to read
    if(fControllerRead)
    {
      do
      {
	      if(i2cSendByte(_i2cBuffer[_nBufferIndex++]))
	      	break;		// stop reading when not acknowledged
	    }
	    while(_nBufferIndex < I2C_BUFFER_LENGTH);
	    _i2cStatus=I2C_STATUS_READ;
	    //fnCallBack(fControllerRead, _nBufferIndex, 0x4D6D);		// TODO?
      return(_nBufferIndex);
    }
    else
    { // Read subsequent bytes until until STOP or buffer full.
    	// Note that the controller may send a STOP immediately, eg. when scanning for devices.
      do
      {
	      //if(_fStoppedI2C || _fRestartI2C)
	      //	break;		// we read until stop/restart or full
	      //b=i2cReceiveByte(true);			// receiving byte first waits for low SCL, then reveices byte and sends ACL ending low.
	      //b=i2cReceiveByteSendAck();			// receiving byte first waits for low SCL, then reveices byte and sends ACL ending low.
	      b=i2cReceiveByte();			// receiving byte first waits for low SCL, then reveices byte and sends ACL ending low.
	      if(_fStoppedI2C || _fRestartI2C)
	      	break;		// we read until stop/restart or full
				if(!i2cSendAck())
				  break;    // we read until stop/restart or full

				_i2cBuffer[_nBufferIndex++]=b;
	    }
	    while(_nBufferIndex < I2C_BUFFER_LENGTH);
	    _i2cStatus=I2C_STATUS_WRITE;
	    //fnCallBack(fControllerRead, _nBufferIndex, 0x4D6D);		// TODO?
      return(_nBufferIndex);
    }
  }
  _i2cStatus=I2C_STATUS_NOADDRESS;
  return(0);
}

byte TinySoftWire::getLastStatus(void)
{	// see what the status of the latest transmission is
	return(_i2cStatus);      // indicates last status: read/write/none/etc
}

uint8_t TinySoftWire::getDataU8(byte nIndex)
{	// get a byte from the buffer
	return((uint8_t)_i2cBuffer[nIndex]);
}

uint16_t TinySoftWire::getDataU16(byte nIndex)
{	// get a byte from the buffer
	return((uint16_t)*((uint16_t*)(_i2cBuffer+nIndex*2)));
}

uint32_t TinySoftWire::getDataU32(byte nIndex)
{	// get a byte from the buffer
	return((uint32_t)*((uint32_t*)(_i2cBuffer+nIndex*4)));
}

void TinySoftWire::setDataU8(byte nIndex, uint8_t uData)
{
	_i2cBuffer[nIndex]=uData;
}

void TinySoftWire::setDataU16(byte nIndex, uint16_t uData)
{
	*((uint16_t*)(_i2cBuffer+nIndex*2))=uData;
}

void TinySoftWire::setDataU32(byte nIndex, uint32_t uData)
{
	*((uint32_t*)(_i2cBuffer+nIndex*4))=uData;
}