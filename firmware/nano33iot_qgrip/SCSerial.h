#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SCS.h"

class SCSerial : public SCS
{
public:
	SCSerial();
	SCSerial(u8 End);
	SCSerial(u8 End, u8 Level);

protected:
	virtual int writeSCS(unsigned char *nDat, int nLen);
	virtual int readSCS(unsigned char *nDat, int nLen);
	virtual int writeSCS(unsigned char bDat);
	virtual void rFlushSCS();
	virtual void wFlushSCS();
public:
	unsigned long int IOTimeOut;
	HardwareSerial *pSerial;
	int Err;
public:
	virtual int getErr(){  return Err;  }
};

#endif
