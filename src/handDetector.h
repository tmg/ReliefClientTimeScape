#include "ofMain.h"
#include "ofxOpenCV.h"
#include "cvLineFitter.h"
#include "ofVec3f.h"
#include "constants.h"

//  good finger tracking explanation here:
//  www.cs.toronto.edu/~smalik/downloads/2503_project_report.pdf 

typedef struct {
	
	ofxCvBlob 			myBlob;	
	float 				curvatureAtPt[MAX_CONTOUR_LENGTH];
	int 				nFingers;
	ofPoint 			fingerPos[MAX_NUM_FINGERS];
	float 				fingerAngle[MAX_NUM_FINGERS];
	float				slope;
	float				intercept;
	float				chiSqr;
	ofPoint				averagePt;
	bool				pinch;
	
} handBlob;

class handDetector {
	
	public:
		
		handDetector();
		
		void init					();
		void setROI					(int x, int y, int width, int height);
		void findFeatures			(ofxCvGrayscaleImage grayImage, unsigned char * pixels);
		void findFingers 			(handBlob &blob);
		void fitLineThroughBlob		(handBlob &aHand, int xSelection, int ySelection, int wSelection, int hSelection);
		void fitLineThroughPts 		(handBlob &blob, int pt, int fingerId);
		void finger					(handBlob &blob);
		void averageFinger			(handBlob &aHand);
		void findHandPoint			(handBlob &aHand);
	
		ofxCvContourFinder contourFinder;
	
		bool						* bFingerPointRuns;
		float						pctOfContourNumForAngleCheck;	// pt j, j-n, j+n, n = pct * contourPtNum
		float						angleThreshold;
		ofPoint						lineFitPts[NPTS_FOR_LINE_FIT];
		cvLineFitter				CVLF;
		
		handBlob					handBlobs[NUMBER_OF_HANDS];
		int							numHoles;
		int							numHands;
		float						averageHandHeight;
	
		ofRectangle					roiRect;
		int							cutoff;
};
