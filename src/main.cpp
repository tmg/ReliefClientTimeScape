#include "ofMain.h"
#include "gesturalReliefApp.h"
#include "ofAppGlutWindow.h"

#include "constants.h"

//========================================================================
int main( ){

    ofAppGlutWindow window;
	ofSetupOpenGL(&window, SCREEN_WIDTH, SCREEN_HEIGHT, OF_WINDOW);
	//ofSetupOpenGL(&window, SCREEN_WIDTH, SCREEN_HEIGHT, OF_FULLSCREEN);
    
	ofRunApp( new gesturalReliefApp());

}
