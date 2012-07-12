/*
 *  ReliefSerial.cpp
 *  Relief
 *
 *  Created by Daniel Leithinger on 2/27/10.
 *  Copyright 2010 MIT Media Lab. All rights reserved.
 *
 */

#include "ReliefSerial.h"

ReliefSerial::ReliefSerial(string pPortName, int pBaudrate, int pFirstArduinoID, int pNumerOfArduinos) {
	mFirstArduinoID = pFirstArduinoID;
	mNumberOfArduinos = pNumerOfArduinos;
	
	for (int i = 0; i < MAX_NUM_ARDUINOS_PER_CONNECTION; i ++) {
		for (int j = 0; j < NUM_PINS_ARDUINO; j++) {
			/*mFromArduinosPinHeight [i][j] = (unsigned char) 106;
			mToArduinosPinHeight [i][j] = (unsigned char) 106;
			mToArduinosPIDTerms [i][j] = (unsigned char) 106;*/
			mFromArduinosPinHeight [i][j] = (unsigned char) 100;
			mToArduinosPinHeight [i][j] = (unsigned char) 100;
			mToArduinosPIDTerms [i][j] = (unsigned char) 100;
		}
	}
	
	mSerial.enumerateDevices();
	mSerial.setup(pPortName, pBaudrate);
	mWritePIDTerms = false;
	start();
}

ReliefSerial::~ReliefSerial(){
	this->stop();
	mSerial.close();
}

void ReliefSerial::start(){
	startThread(true, false);   // blocking, verbose
}

void ReliefSerial::stop(){
	stopThread();
}

void ReliefSerial::threadedFunction(){ 
	ofSleepMillis(1000);
	
	while (mSerial.available()) {
		mSerial.readByte();
	}
	
	while( isThreadRunning() != 0 ){
		lock();
		bool writePIDTerms = mWritePIDTerms;
		unlock();
		for (int i = mFirstArduinoID; i < mFirstArduinoID + mNumberOfArduinos; i++) {
			if(writePIDTerms) {
				printf("write pid to %d. \n", i);
				serialWritePIDTerms(i);
				ofSleepMillis(2);
				serialReceiveHeight();
				ofSleepMillis(2);
			}
			else {
				serialWriteHeight(i);
				ofSleepMillis(2);
				serialReceiveHeight();
				ofSleepMillis(2);
			}
		}
		if(writePIDTerms) {
			lock();
			mWritePIDTerms = false;
			unlock();
		}
	}
}

void ReliefSerial::getFromArduinoPinHeight(int pID, unsigned char pFromArduinoPinHeight[NUM_PINS_ARDUINO]) {
	int index = getArduinoArrayIndex(pID);
	lock();
	for (int j = 0; j < NUM_PINS_ARDUINO; j ++) {
		pFromArduinoPinHeight[j] = mFromArduinosPinHeight[index][j];
	}
	unlock();
}

void ReliefSerial::writeToArduinoPinHeight(int pID, unsigned char pToArduinoPinHeight[NUM_PINS_ARDUINO]) {
	int index = getArduinoArrayIndex(pID);
	lock();
	for (int j = 0; j < NUM_PINS_ARDUINO; j ++) {
		mToArduinosPinHeight[index][j] = pToArduinoPinHeight[j];
	}
	unlock();
}

void ReliefSerial::serialWriteHeight(int pID) {
	int index = getArduinoArrayIndex(pID);
	
	unsigned char heightMessage[6];
	heightMessage[0] = (unsigned char)pID;
	heightMessage[1] = 0x00;
	lock();
	for(int j = 0; j < NUM_PINS_ARDUINO; j++) {
		heightMessage[2 + j] = mToArduinosPinHeight[index][j];
	}
	unlock();
	mSerial.writeBytes( heightMessage, 6);
}

int ReliefSerial::serialReceiveHeight() {
	/*bool read = true;
	unsigned char receiveHeightMessage[5];
	memset(receiveHeightMessage, 0, 5);
	
	mSerial.readBytes(receiveHeightMessage, 5);
	
	int index = getArduinoArrayIndex((int)receiveHeightMessage[0]);
	lock();
	for (int i = 0; i < NUM_PINS_ARDUINO; i++)
		mFromArduinosPinHeight[index][i] = receiveHeightMessage[1+i];
	unlock();
	//printf("Got message: %d \n", (int)receiveHeightMessage[0]);
	return (int)receiveHeightMessage[0];
	*/
	
	int counter = 0;
	int messageByte = 0;
	int id = -1;
	while (counter < 6) {
		messageByte = mSerial.readByte();
		if (messageByte == 255)
			counter = 6;
		switch (counter) {
			case 0:
				id = getArduinoArrayIndex(messageByte);
				break;
			case 1:
			case 2:
			case 3:
			case 4:
				mFromArduinosPinHeight[id][counter - 1] = (unsigned char)messageByte;
			case 5:
			default:
				break;
		}
		counter ++;
	}
	return id;
	
}

int ReliefSerial::getArduinoArrayIndex(int pID) {
	int index = pID  - mFirstArduinoID;
	if(index >=0 && index < mNumberOfArduinos) {
		return index;
	}
	else {
		printf("Wrong Arduino ID: %d \n", pID);
		return -1;
	}
}

void ReliefSerial::writeToArduinoPIDTerms(int pID, unsigned char pDeadZone, unsigned char pGain_P, unsigned char pGain_I, unsigned char pMax_I) {
	int index = getArduinoArrayIndex(pID);
	lock();
	mToArduinosPIDTerms[index][0] = pDeadZone;
	mToArduinosPIDTerms[index][1] = pGain_P;
	mToArduinosPIDTerms[index][2] = pGain_I;
	mToArduinosPIDTerms[index][3] = pMax_I;
	mWritePIDTerms = true;
	unlock();
}

void ReliefSerial::serialWritePIDTerms(int pID) {
	int index = getArduinoArrayIndex(pID);
	
	unsigned char pidMessage[6];
	pidMessage[0] = (unsigned char)pID;
	pidMessage[1] = (unsigned char) 1;
	lock();
	for(int j = 0; j < NUM_PINS_ARDUINO; j++) {
		pidMessage[2 + j] = mToArduinosPIDTerms[index][j];
	}
	unlock();
	mSerial.writeBytes( pidMessage, 6);
}

