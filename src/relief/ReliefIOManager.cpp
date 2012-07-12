/*
 *  ReliefIOManager.cpp
 *  Relief
 *
 *  Created by Daniel Leithinger on 3/3/10.
 *  Copyright 2010 MIT Media Lab. All rights reserved.
 *
 */

#include "ReliefIOManager.h"

ReliefIOManager::ReliefIOManager() {
	loadSettings(RELIEFSETTINGS);
	
	mSerialConnections[0] = new ReliefSerial(SERIAL_PORT_0, SERIAL_BAUD_RATE, SERIAL_PORT_0_FIRST_ID, SERIAL_PORT_0_NUMEROFARDUINOS);
	mSerialConnections[1] = new ReliefSerial(SERIAL_PORT_1, SERIAL_BAUD_RATE, SERIAL_PORT_1_FIRST_ID, SERIAL_PORT_1_NUMEROFARDUINOS);
	mSerialConnections[2] = new ReliefSerial(SERIAL_PORT_2, SERIAL_BAUD_RATE, SERIAL_PORT_2_FIRST_ID, SERIAL_PORT_2_NUMEROFARDUINOS);
}

ReliefIOManager::~ReliefIOManager() {
	for (int i = 0; i < NUM_SERIAL_CONNECTIONS; i++) {
		delete mSerialConnections[i];
	}
}


void ReliefIOManager::getPinHeightFromRelief(unsigned char pPinHeightFromRelief [RELIEF_SIZE_X][RELIEF_SIZE_Y]) {
	unsigned char fromArduinoPinHeight [NUM_PINS_ARDUINO];
	memset(fromArduinoPinHeight, 0, NUM_PINS_ARDUINO * sizeof(unsigned char));
	
	for (int i = 0; i < NUM_SERIAL_CONNECTIONS; i ++) {
		for (int j = mSerialConnections[i]->mFirstArduinoID; j < mSerialConnections[i]->mFirstArduinoID + mSerialConnections[i]->mNumberOfArduinos; j++) {
			mSerialConnections[i]->getFromArduinoPinHeight(j, fromArduinoPinHeight);
			for (int k = 0; k < NUM_PINS_ARDUINO; k++) {
				pPinHeightFromRelief[mPinLookupTable[j][k][0]][mPinLookupTable[j][k][1]] = fromArduinoPinHeight[k];
			}
		}
	}
}

void ReliefIOManager::sendPinHeightToRelief(unsigned char pPinHeightToRelief [RELIEF_SIZE_X][RELIEF_SIZE_Y]) {
	unsigned char toArduinoPinHeight [NUM_PINS_ARDUINO];
	memset(toArduinoPinHeight, 0, NUM_PINS_ARDUINO * sizeof(unsigned char));
	
	for (int i = 0; i < NUM_SERIAL_CONNECTIONS; i ++) {
		for (int j = mSerialConnections[i]->mFirstArduinoID; j < mSerialConnections[i]->mFirstArduinoID + mSerialConnections[i]->mNumberOfArduinos; j++) {
			for (int k = 0; k < NUM_PINS_ARDUINO; k++) {
				toArduinoPinHeight[k] = pPinHeightToRelief[mPinLookupTable[j][k][0]][mPinLookupTable[j][k][1]];
				if (toArduinoPinHeight[k] > 120)
					toArduinoPinHeight[k] = 120;
				
			}
			mSerialConnections[i]->writeToArduinoPinHeight(j, toArduinoPinHeight);
		}
	}
}

void ReliefIOManager::sendPidTermsToRelief(unsigned char pDeadZone, unsigned char pGain_P, unsigned char pGain_I, unsigned char pMax_I) {
	for (int i = 0; i < NUM_SERIAL_CONNECTIONS; i ++) {
		for (int j = mSerialConnections[i]->mFirstArduinoID; j < mSerialConnections[i]->mFirstArduinoID + mSerialConnections[i]->mNumberOfArduinos; j++) {
			mSerialConnections[i]->writeToArduinoPIDTerms(j, pDeadZone, pGain_P, pGain_I, pMax_I);
		}
	}
}

void ReliefIOManager::loadSettings(string pFileName) {
	
	mXML.loadFile(pFileName);
	
	// load pin lookup table
	int numArduinos = mXML.getNumTags("BOX");
	if (numArduinos == NUM_ARDUINOS) {
		for (int i = 0; i < numArduinos; i ++) {
			mXML.pushTag("BOX", i);
			int ArduinoID = mXML.getValue("ID", 0);
			for (int j = 0; j < NUM_PINS_ARDUINO; j++) {
				int x = mXML.getValue("PIN:X", 0, j);
				int y = mXML.getValue("PIN:Y", 0, j);
				mPinLookupTable[ArduinoID][j][0] = x;
				mPinLookupTable[ArduinoID][j][1] = y;
			}
			mXML.popTag();
		}
	}
	
	// print out the loaded pin lookup table for debugging
	for (int i = 0; i < NUM_ARDUINOS; i++) {
		for (int j = 0; j < NUM_PINS_ARDUINO; j++) {
			printf(" %d %d, ", mPinLookupTable[i][j][0], mPinLookupTable[i][j][1]);
		}
		printf("\n");
	}
}

void ReliefIOManager::saveSettings(string pFileName) {
	mXML.clear();
	for (int i = 0; i < NUM_ARDUINOS; i++) {
		int currentTagNumber = mXML.addTag("BOX");
		mXML.setValue("BOX:ID", i, currentTagNumber);
		if( mXML.pushTag("BOX", currentTagNumber) ){			
			for (int j = 0; j < 4; j++) {
				int tagNum = mXML.addTag("PIN");
				mXML.setValue("PIN:ID", j, tagNum);
				mXML.setValue("PIN:X", mPinLookupTable[i][j][0], tagNum);
				mXML.setValue("PIN:Y", mPinLookupTable[i][j][1], tagNum);
			}
			mXML.popTag();
		}
	}
	mXML.saveFile(pFileName);
}