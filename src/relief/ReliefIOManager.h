/*
 *  ReliefIOManager.h
 *  Relief
 *
 *  Created by Daniel Leithinger on 3/3/10.
 *  Copyright 2010 MIT Media Lab. All rights reserved.
 *
 */

#ifndef _RELIEF_IO_MANAGER
#define _RELIEF_IO_MANAGER

#include "ofMain.h"
#include "ReliefSerial.h"
#include "Constants.h"
#include "ofxXmlSettings.h"

class ReliefIOManager {
private:
	ReliefSerial * mSerialConnections [NUM_SERIAL_CONNECTIONS];
	int mPinMaskTable [RELIEF_SIZE_X][RELIEF_SIZE_Y];
	int mPinLookupTable [NUM_ARDUINOS][NUM_PINS_ARDUINO][2];
	ofxXmlSettings mXML;
	
public:
	ReliefIOManager();
	~ReliefIOManager();
	void getPinHeightFromRelief(unsigned char pPinHeightFromRelief [RELIEF_SIZE_X][RELIEF_SIZE_Y]);
	void sendPinHeightToRelief(unsigned char pPinHeightToRelief [RELIEF_SIZE_X][RELIEF_SIZE_Y]);
	void sendPidTermsToRelief(unsigned char pDeadZone, unsigned char pGain_P, unsigned char pGain_I, unsigned char pMax_I);
	void loadSettings(string pFileName);
	void saveSettings(string pFileName);
};


#endif
