/*
 *  ReliefSerial.h
 *  Relief
 *
 *  Created by Daniel Leithinger on 2/27/10.
 *  Copyright 2010 MIT Media Lab. All rights reserved.
 *
 */

#ifndef _RELIEF_SERIAL
#define _RELIEF_SERIAL

#include "ofxThread.h"
#include "ofMain.h"
#include "ofxThread.h"
#include "constants.h"

class ReliefSerial: public ofxThread{
private:
	ofSerial mSerial;
	unsigned char mFromArduinosPinHeight [MAX_NUM_ARDUINOS_PER_CONNECTION][NUM_PINS_ARDUINO];
	unsigned char mToArduinosPinHeight [MAX_NUM_ARDUINOS_PER_CONNECTION][NUM_PINS_ARDUINO];
	unsigned char mToArduinosPIDTerms [MAX_NUM_ARDUINOS_PER_CONNECTION][4];
	bool mWritePIDTerms;
	void start();
	void stop();
	void threadedFunction();
	void serialWriteHeight(int pID);
	int serialReceiveHeight();
	void serialWritePIDTerms(int pID);
	int getArduinoArrayIndex(int pID);
	
public:
	int mFirstArduinoID;
	int mNumberOfArduinos;
	ReliefSerial(string pPortName, int pBaudrate, int pFirstArduinoID, int pNumerOfArduinos);
	~ReliefSerial();
	void getFromArduinoPinHeight(int pID, unsigned char pFromArduinoPinHeight[NUM_PINS_ARDUINO]);
	void writeToArduinoPinHeight(int pID, unsigned char pToArduinoPinHeight[NUM_PINS_ARDUINO]);
	void writeToArduinoPIDTerms(int pID, unsigned char pDeadZone, unsigned char pGain_P, unsigned char pGain_I, unsigned char pMax_I);
	
};

#endif