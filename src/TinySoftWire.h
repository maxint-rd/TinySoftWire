#ifndef _TINYSOFTWIRE_H
#define _TINYSOFTWIRE_H
#include <Arduino.h>

// define I2C pins
#define I2C_SDA 0          // NOTE: fixed to pin 0 as for testing PINB0
#define I2C_SCL 1          // NOTE: fixed to pin 1 as for testing PINB1

#define I2C_BUFFER_LENGTH 4		// size of buffer used for read()

#define OPT_DEBUG_BLIP 0
#define OPT_ISR 0
#define OPT_CHANGE_ADDRESS 1


#if(OPT_ISR)
#define PIN_CHANGE_INTERRUPT_ENABLE PCIE
#define GENERAL_INTERRUPT_MASK GIMSK
#define GENERAL_INTERRUPT_FLAGS GIFR
#define PIN_CHANGE_FLAG     PCIF
#define PIN_CHANGE_MASK     PCMSK
#endif

//
// Timeout when waiting for START condition.
// Using a timeout ensures that the process() method returns when the I2C bus is quiet.
// Note that this only works when the bus isn't floating.
// To keep code small, a simple 16-bit timeout-counter is used, not a timer.
// The actual time depends on which loop timed out
// 500 is about as long as 0.5 ms, 1000 is about 1 ms (when running at 9.6 Mhz)
#define I2C_START_TIMEOUT			1000L

//
// 8-bit Timeout counter when waiting for clock to go low.
// If this happens during the device sending the ACK, this results in a STOP condition (_fStoppedI2C==true).
#define I2C_CLOCK_TIMEOUT			100


//
// Status definitons. Use getLastStatus() to see in what status process() returned
//
#define I2C_STATUS_NONE				0			// nothing done yet [**]
#define I2C_STATUS_START			1			// transaction was started properly [*]
#define I2C_STATUS_NOSTART		5			// time-out during waiting for start
#define I2C_STATUS_NOADDRESS	10		// address received for other device
#define I2C_STATUS_READ				20		// read-request by controller was processed
#define I2C_STATUS_NOREAD			25		// read-request by controller failed [*]
#define I2C_STATUS_WRITE			30		// data-write by controller was processed
#define I2C_STATUS_NOWRITE		35		// data-write by controller failed [*]

class TinySoftWire
{
 public:
	TinySoftWire(void);
	void begin(byte address=0x4D);
	byte process();
	byte getLastStatus(void);
	uint8_t getDataU8(byte nIndex=0);
	uint16_t getDataU16(byte nIndex=0);
	uint32_t getDataU32(byte nIndex=0);
	void setDataU8(byte nIndex=0, uint8_t uData=0);
	void setDataU16(byte nIndex=0, uint16_t uData=0);
	void setDataU32(byte nIndex=0, uint32_t uData=0L);
	
#if(OPT_CHANGE_ADDRESS)
	void setAddress(byte address);
	uint8_t getAddress(void);
#endif


 private:	
#if(OPT_DEBUG_BLIP)
	void blip(byte cnt=1);
#endif
	bool i2cWaitForStart(void);
	bool i2cWaitForLowClock(void);
	bool i2cSendAck(void);
	bool i2cReceiveAck(void);
	//byte i2cReceiveByte(bool fSendAck=true);
	byte i2cReceiveByte();
	byte i2cReceiveByteSendAck();
	//bool i2cReceiveAddress(void);
	bool i2cSendByte(byte bData);

	bool _fStoppedI2C=false;
	bool _fRestartI2C=false;
	byte _i2cBuffer[I2C_BUFFER_LENGTH];
	//byte _i2cAddress=0;
	byte _i2cAddressWrite=0;
	byte _i2cStatus=0;
};

#endif