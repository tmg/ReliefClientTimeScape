#include "ofMain.h"
#include "gesturalReliefApp.h"
#include "ofAppGlutWindow.h"

#include "constants.h"

//========================================================================
int main( ){

    ofAppGlutWindow window;
#ifdef SPAN_SCREEN
	ofSetupOpenGL(&window, SCREEN_WIDTH, SCREEN_HEIGHT, OF_WINDOW);
#else
	ofSetupOpenGL(&window, SCREEN_WIDTH, SCREEN_HEIGHT, OF_FULLSCREEN);
#endif
	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp( new gesturalReliefApp());

}
