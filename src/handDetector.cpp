#include "handDetector.h"

handDetector::handDetector(){
	bFingerPointRuns = new bool[MAX_CONTOUR_LENGTH];
	angleThreshold = FINGER_ANGLE_THRESHOLD;
}

void handDetector::init() {
	// Init hands
	for (int i = 0; i < NUMBER_OF_HANDS; i++) {
		handBlobs[i].averagePt.x = 0;
		handBlobs[i].averagePt.y = 0;
	}
	cutoff = FINGER_Y_CUTOFF;
}

void handDetector::setROI(int x, int y, int width, int height) {
	roiRect.x = x;
	roiRect.y = y;
	roiRect.width = width;
	roiRect.height = height;
}

void handDetector::findFeatures(ofxCvGrayscaleImage grayImage, unsigned char * pixels) {
	// Constrain to ROI so we ignore extraneous data
	grayImage.setROI(roiRect);
	
	contourFinder.findContours(grayImage, BLOB_SIZE, (grayImage.width * grayImage.height)/8, 4, true);
	
	numHoles = 0;
	numHands = 0;
	averageHandHeight = 0;
	
	for(int i = 0; i < contourFinder.nBlobs; i++) {
				
		// blob.hole is true when we have a hand (not a pinch)
		if(contourFinder.blobs[i].hole){
			if (numHands < NUMBER_OF_HANDS) {
				handBlobs[numHands].myBlob = contourFinder.blobs[i];
				//findFingers(handBlobs[numHands]);
				
				// Can only call one of the functions below as they both set averagePt
				//averageFinger(handBlobs[numHands]);
				findHandPoint(handBlobs[numHands]);
				
				numHands++;
			}
		}
		else {
			numHoles++;
		}
	}
		
	if (numHoles > 0) {
		for (int j = 0; j < numHands; j++) {
			handBlobs[j].pinch = true;
		}
	}
	else {
		for (int j = 0; j < numHands; j++) {
			handBlobs[j].pinch = false;
		}
	}
		
	for (int h = 0; h < numHands; h++) {
		averageHandHeight += pixels[(int)((handBlobs[h].averagePt.x + roiRect.x) + (handBlobs[h].averagePt.y + roiRect.y) * grayImage.width)];
	}
	
	averageHandHeight /= numHands;
}

void handDetector::findHandPoint(handBlob &aHand) {
	int maxPtIndex = -1;
	int maxPtValue = -1;
	for (int i = 0; i < aHand.myBlob.nPts-1; i++) {
		if (aHand.myBlob.pts[i].y > maxPtValue) {
			maxPtValue = aHand.myBlob.pts[i].y;
			maxPtIndex = i;
		}
	}
	if (maxPtIndex > -1) {
		aHand.averagePt = aHand.myBlob.pts[maxPtIndex];
	}
}

//------------------------------------------------------------
void handDetector::findFingers 	(handBlob &blob){

	memset (bFingerPointRuns, 0, sizeof(bool) * MAX_CONTOUR_LENGTH);

	//--------------------------- blur the contour pts alot ----------
	for (int k=0; k < 3; k++){
		for (int j=1; j< blob.myBlob.nPts-1; j++){
			
			blob.myBlob.pts[j].x =  0.6f * blob.myBlob.pts[j].x +
								  0.2f * blob.myBlob.pts[j-1].x +
								  0.2f * blob.myBlob.pts[j+1].x;
			blob.myBlob.pts[j].y =  0.6f * blob.myBlob.pts[j].y +
								  0.2f * blob.myBlob.pts[j-1].y +
								  0.2f * blob.myBlob.pts[j+1].y;
			
		}
	}

	//--------------------------- calc angles ----------
	// this can be optimized, please
	
	// int kConstant = (int)(blob.myBlob.nPts/20.0f); 
	// usually, this made sense (adjust angle calc based on n points), but with CV_CHAIN_APPROX...  
	// with all the points, and hands pretty big, I found this is pretty good:
	
	int kConstant = 25; //default 30
	
	// 	you can try adjusting and see what works well:
	//	int kConstant = 33 + 30 * sin(ofGetElapsedTimef());
	//	printf("%i \n", (int)kConstant);
	
	for (int j=0; j<blob.myBlob.nPts; j++){
		int pt0 = (j - kConstant + blob.myBlob.nPts) % blob.myBlob.nPts;
		int pt1 = j;
		int pt2 = (j + kConstant) % blob.myBlob.nPts;
		
		ofVec3f lineab;
		lineab.x = blob.myBlob.pts[pt1].x - blob.myBlob.pts[pt0].x;
		lineab.y = blob.myBlob.pts[pt1].y - blob.myBlob.pts[pt0].y;
		
		ofVec3f linecb;
		linecb.x = blob.myBlob.pts[pt2].x - blob.myBlob.pts[pt1].x;
		linecb.y = blob.myBlob.pts[pt2].y - blob.myBlob.pts[pt1].y;
		
		lineab.normalize();
		linecb.normalize();
		float dot = lineab.x * linecb.x + lineab.y * linecb.y;
		float cross = lineab.x * linecb.y - lineab.y * linecb.x;
		float theta = acos(dot);
		if (cross < 0) { theta = 0-theta; }
		blob.curvatureAtPt[j] = theta;
		
		bFingerPointRuns[j] = blob.curvatureAtPt[j] < angleThreshold;
	}

	//if (bFingerPointRuns[0] == true) do something else, smart!
	// ie search for first non, start there and do to there+nPts % nPts...!
	// that's smart :)

	bool bInRun = false;
	int startRun = 0;
	int fingersFound = 0;
	blob.nFingers = 0;
	for (int j=0; j<blob.myBlob.nPts; j++){
		
		if (!bInRun){
			if (bFingerPointRuns[j] == true){
				bInRun = true;
				startRun = j;
			}
		} else {
			if (bFingerPointRuns[j] == false){
				bInRun = false;
				//printf("run from %i to %i \n", startRun, j);
				float maximumMin = 0;
				int indexOfMaxMin = -1;
				
				if (j - startRun > 10){
				for (int k = startRun; k < j; k++){
					float c = blob.curvatureAtPt[k];
					if (c < maximumMin){
						maximumMin = c;
						indexOfMaxMin = k;
					}
				}
				}
				if (indexOfMaxMin != -1) {
					
					if (blob.myBlob.pts[indexOfMaxMin].y > cutoff) {
						blob.fingerPos[blob.nFingers] = blob.myBlob.pts[indexOfMaxMin];
						
						// try to fit a line here:
						fitLineThroughPts(blob, indexOfMaxMin, blob.nFingers);
						
						if (blob.nFingers < MAX_NUM_FINGERS) {
						 	blob.nFingers++;
						}
					}
				}
			}
		
		}
	}
	
}

//------------------------------------------------------------
void handDetector::fitLineThroughBlob (handBlob &aHand, int xSelection, int ySelection, int wSelection, int hSelection) {
	
	int numPtsInSelection = 0;
	ofPoint lineFitPts[aHand.myBlob.nPts];
	
	for (int j=0; j< aHand.myBlob.nPts; j++){
		if (aHand.myBlob.pts[j].x > (xSelection) && aHand.myBlob.pts[j].x < (xSelection + wSelection)
			&& aHand.myBlob.pts[j].y > (ySelection) && aHand.myBlob.pts[j].y < (ySelection + hSelection)) {
			lineFitPts[numPtsInSelection].x = aHand.myBlob.pts[j].x;
			lineFitPts[numPtsInSelection].y = aHand.myBlob.pts[j].y;
			numPtsInSelection++;
		}
	}
	
	if (numPtsInSelection > 0)
		CVLF.fitLine(lineFitPts, numPtsInSelection, aHand.slope, aHand.intercept, aHand.chiSqr);
	else {
		aHand.slope = 0;
		aHand.intercept = 0;
		aHand.chiSqr = 0;
	}
}

//------------------------------------------------------------
void handDetector::averageFinger (handBlob &aHand) {
	
	if (aHand.nFingers > 0) {
		
		float xSum = 0, ySum = 0;
		
		for (int i = 0; i < aHand.nFingers; i++) {
			xSum += aHand.fingerPos[i].x;
			ySum += aHand.fingerPos[i].y;
		}
		
		float averageX = xSum / aHand.nFingers;
		float averageY = ySum / aHand.nFingers;
		
		if (aHand.averagePt.x == -1 || aHand.averagePt.y == -1) {
			// Value needs to be reset, use average directly
			aHand.averagePt.x = averageX;
			aHand.averagePt.y = averageY;
		}
		else {
			if (fabs(averageX - aHand.averagePt.x) > AVERAGE_FINGER_JUMP_RADIUS
				|| fabs(averageY - aHand.averagePt.y) > AVERAGE_FINGER_JUMP_RADIUS) {
				// Jump so change value immediately
				aHand.averagePt.x = averageX;
				aHand.averagePt.y = averageY;
			}
			else {
				// Smooth with previous value
				aHand.averagePt.x += (averageX - aHand.averagePt.x) / AVERAGE_FINGER_DELAY;
				aHand.averagePt.y += (averageY - aHand.averagePt.y) / AVERAGE_FINGER_DELAY;
			}
		}
	}
}


//------------------------------------------------------------
void handDetector::finger(handBlob	&blob) {
	ofPoint point1, point2, point3;
	
	for (int i = 0; i < (blob.myBlob.nPts - 2); i++) {
		point1 = blob.myBlob.pts[i];
		point2 = blob.myBlob.pts[i+1];		
		point3 = blob.myBlob.pts[i+2];
		
		if(point2.y < point1.y && point2.y < point3.y) {
			int lineAC1 = pow(((pow((point1.y - point2.y),2)) + (pow((point2.x - point1.x),2))),(1/2));
			float angleA1 = acos(sin(lineAC1));
			int lineAC2 = pow(((pow((point3.y - point2.y),2))+(pow((point3.x - point2.x),2))),(1/2));
			float angleA2 = acos(sin(lineAC2));
			float angle = angleA1 + angleA2;
			//printf("Hoek %f \n", hoek);
			if(angle > 3){
				blob.nFingers++;
				//if(blob.fingerPos[i].x[(blob.nFingers[i]-1)] > 20 && (20+point2.x) == 0){
				if(blob.fingerPos[(blob.nFingers-1)].x > 20 && (20+point2.x) == 0){
					
				} else {
				//	blob.fingerPos[i].x[(blob.nFingers[i]-1)] = 20 + point2.x;
				//	blob.fingerPos[i].y[(blob.nFingers[i]-1)] = 20 + point2.y;
				//	blob.fingerPos[(blob.nFingers-1)].x = 20 + point2.x;
				//	blob.fingerPos[(blob.nFingers-1)].y = 20 + point2.y;
					blob.fingerPos[i].x = 20 + point2.x;
					blob.fingerPos[i].y = 20 + point2.y;
				
				}
				
			}
		}
	}
	printf("Array length %i \n",blob.nFingers);
}

//------------------------------------------------------------
void handDetector::fitLineThroughPts (handBlob	&blob, int pt, int fingerId){

	int count = 0;
	for (int j=0; j< NPTS_FOR_LINE_FIT; j++){
		int myLpt = ((pt - j) + blob.myBlob.nPts) % blob.myBlob.nPts;
		int myRpt = ((pt + j) + blob.myBlob.nPts) % blob.myBlob.nPts;
		lineFitPts[j].x = (blob.myBlob.pts[myLpt].x + blob.myBlob.pts[myRpt].x)/2;
		lineFitPts[j].y = (blob.myBlob.pts[myLpt].y + blob.myBlob.pts[myRpt].y)/2;
	}
	
	float slope, intercept, chisqr;
	CVLF.fitLine(lineFitPts, NPTS_FOR_LINE_FIT, slope, intercept, chisqr);
	
	//blob.s->slopeFinger[fingerId] = slope;
	//blob.s->interceptFinger[fingerId] = intercept;
	
	if (fabs(slope) < 0.000001) slope = 0.000001; // / by zero?
	
	ofVec3f pta;
	pta.x = 0;
	pta.y = intercept;

	ofVec3f ptb;
	ptb.y = 0;
	ptb.x = -intercept / slope;
	
	ofVec3f diff = pta - ptb;
	float angle = atan2(diff.y, diff.x);
	
	blob.fingerAngle[fingerId] = angle;
	//printf("%f %f %f \n", slope, intercept, chisqr);

}
		
