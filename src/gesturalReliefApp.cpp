#include "gesturalReliefApp.h"


//--------------------------------------------------------------
void gesturalReliefApp::setup(){
	
	/*****************
	 * General setup *
	 *****************/
	
	ofNoFill();
	glLineWidth(2);
	glPointSize(1);
	
	//OpenGL Setup	
	GLfloat red[] = {1.0, 0.0, 0.0}; //set the material to red
	GLfloat white[] = {1.0, 1.0, 1.0}; //set the material to white
	GLfloat green[] = {0.0, 1.0, 0.0}; //set the material to green
	GLfloat black[] = {0.0, 0.0, 0.0}; //set the light ambient to black
	GLfloat grey[] = {0.1, 0.1, 0.1}; //set the light ambient to black
	GLfloat mShininess[] = {2}; //set the shininess of the material
	//GLfloat lightPosition[] = {0, 0,0, 1};
	
	glLightfv(GL_LIGHT0, GL_SPECULAR, black);
	glLightfv(GL_LIGHT0, GL_AMBIENT, black);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
	//glLightfv (GL_LIGHT0, GL_POSITION, lightPosition);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);
	
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mShininess);
	
	GLfloat light_position[] = { 0,-100,10,0 };
	glLightfv(GL_LIGHT0,GL_POSITION,light_position);
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	
	
	roiRect.x = ROI_RECT_X;
	roiRect.y = ROI_RECT_Y;
	roiRect.width = ROI_RECT_W;
	roiRect.height = ROI_RECT_H;
	
	ofBackground(0, 0, 0);
	ofSetFrameRate(60);
	ofSetWindowTitle("Gestural Relief");
	
	previousSelectionColor = 255;
	
	projectionRect.x = PROJECTION_RECT_X;
	projectionRect.y = PROJECTION_RECT_Y;
	projectionRect.width = PROJECTION_RECT_WIDTH;
	projectionRect.height = PROJECTION_RECT_HEIGHT;
	
	parallaxRect.x = PARALLAX_ORIGIN_X;
	parallaxRect.y = PARALLAX_ORIGIN_Y;
	parallaxRect.width = PARALLAX_RATIO_X;
	parallaxRect.height = PARALLAX_RATIO_Y;
	
	transformImage.allocate(RELIEF_SIZE_X, RELIEF_SIZE_Y);
	
	// Set projection pixel buffer to basic values
	resetProjectionPixels();
	
	/****************
	 * Kinect setup *
	 ****************/
	
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
	
	calibratedColorImage.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
	grayImage.allocate(kinect.width, kinect.height);
	
	// Mem allocation for tracking
	grayThresh.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	// Init values for tracking
	bThreshWithOpenCV = KINECT_THRESHOLD_ENABLED;
	nearThreshold = KINECT_NEAR_THRESHOLD;
	farThreshold  = KINECT_FAR_THRESHOLD;
	
	// Init hand detector
	myHandDetector.init();
	myHandDetector.setROI(roiRect.x - ROI_BORDER_W, roiRect.y - ROI_BORDER_H, roiRect.width + ROI_BORDER_W * 2, roiRect.height + ROI_BORDER_H * 2);
	
	/****************
	 * Relief setup *
	 ****************/
	
	// Relief mask
	unsigned char mPinMaskDummy[RELIEF_SIZE_X][RELIEF_SIZE_Y] = {
		{0,0,0,1,1,1,1,1,1,0,0,0},
		{0,0,1,1,1,1,1,1,1,1,0,0},
		{0,1,1,1,1,1,1,1,1,1,1,0},
		{1,1,1,1,1,1,1,1,1,1,1,1},
		{1,1,1,1,1,1,1,1,1,1,1,1},
		{1,1,1,1,1,1,1,1,1,1,1,1},
		{1,1,1,1,1,1,1,1,1,1,1,1},
		{1,1,1,1,1,1,1,1,1,1,1,1},
		{1,1,1,1,1,1,1,1,1,1,1,1},
		{0,1,1,1,1,1,1,1,1,1,1,0},
		{0,0,1,1,1,1,1,1,1,1,0,0},
		{0,0,0,1,1,1,1,1,1,0,0,0}
	};
	memcpy(mPinMask, mPinMaskDummy, RELIEF_SIZE_X * RELIEF_SIZE_Y);
	generateMeshMask();
	// Initialize communication with Relief table
	//if(RELIEF_CONNECTED)
		//mIOManager = new ReliefIOManager();
	
	// Reset height pin arrays
	updateFromReliefHeight();
	
	//State machine init
	state = 0;
	
	current_frame = 0;
	editing = 1;
	recording = 0;
	current_instance = 0;
	visualizationOffset = 0;
	inverted_clip_height = 40;
	for (int x = 0; x < RELIEF_SIZE_X; x++) {
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			mPinHeightToRelief[x][y] = 100;
			mPinHeightToRelief[x][y] = 50; //use this starting value for a safe reset			
		}
	}
	pushInstance();
	buildshape();
	startLoading();
	visualizationOffset = SCREEN_HEIGHT*2;
	ofHideCursor();
	
	//temp camera values
	instanceCamera.y = 480;
	instanceCamera.z = -400;
	instanceCamera.angle = -15;
	
	frameCamera.y = 403;
	frameCamera.z = -148;
	frameCamera.angle = -11;
	
	font.loadFont("HelveticaLight.ttf", 14);

	std::vector<GLubyte> image;
	unsigned int width, height;
	unsigned error = lodepng::decode(image, width, height, "../../../data/mesh-texture.png");
	
	//printf("%d %d %d\n", width, height,image.size());
	
	// If there's an error, display it.
	if(error != 0) {
		std::cout << "error " << error << ": " << lodepng_error_text(error) << std::endl;
	}	
	
    // allocate a texture name
    glGenTextures( 1, &texture );
	
    // select our current texture
    glBindTexture( GL_TEXTURE_2D, texture );
	
	// select modulate to mix texture with color for shading
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
	
    // when texture area is small, bilinear filter the closest mipmap
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST );
    // when texture area is large, bilinear filter the first mipmap
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	
    // if wrap is true, the texture wraps over at the edges (repeat)
    //       ... false, the texture ends at the edges (clamp)
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,GL_CLAMP );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,GL_CLAMP );
	
    // build our texture mipmaps
    gluBuild2DMipmaps( GL_TEXTURE_2D, 4, width, height, GL_RGBA, GL_UNSIGNED_BYTE, &image[0] );
    
    //Network
    connected = false;
    sender.setup(HOST, CONNECTION_PORT);
    receiver.setup(LISTEN_PORT);
    lastPing = ofGetElapsedTimef();
	
}

//--------------------------------------------------------------
void gesturalReliefApp::update(){

	/*****************
	 * Kinect update *
	 *****************/
	
	kinect.update();
	
	unsigned char * pixels = kinect.getDepthPixels();
	calibratedColorImage.setFromPixels(kinect.getCalibratedRGBPixels(), kinect.width, kinect.height, OF_IMAGE_COLOR);
	int sum = 0;
	
	//invert heights
	/*for(int y = 0; y < kinect.height; y++) {
		for (int x = 0; x < kinect.width; x++) {
			pixels[(y*kinect.width)+x]= 255-pixels[(y*kinect.width)+x];
		}
	}*/
	
	//flip horizontally
	/*for(int y = 0; y < kinect.height; y++) {
		for (int x = 0; x < kinect.width/2; x++) {
			unsigned char temp = pixels[(y*kinect.width)+x];
			pixels[(y*kinect.width)+x] = pixels[(y*kinect.width)+(kinect.width-1-x)];
			pixels[(y*kinect.width)+(kinect.width-1-x)] = temp;
		}
	}*/
	
	//printf("%d\n",sum/(kinect.height*kinect.width));
	
	grayImage.setFromPixels(pixels, kinect.width, kinect.height);
	
	//pixels = grayImage.getPixels();
	// We do two thresholds - one for the far plane and one for the near plane
	// We then do a cvAnd to get the pixels which are a union of the two thresholds.	
	if ( bThreshWithOpenCV ){
		grayThreshFar = grayImage;
		grayThresh = grayImage;
		grayThreshFar.threshold(farThreshold);
		grayThresh.threshold(nearThreshold,true);
		cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		//grayImage = grayThresh;
		// These use OpenCV to blur the image
		//grayImage = grayThresh;
		grayImage.erode_3x3();
		grayImage.dilate_3x3();
	}
	
	// Update the cv image
	grayImage.flagImageChanged();
	
	// Find features using the hand detector
	myHandDetector.findFeatures(grayImage, pixels);
	
	/*****************
	 * Update cursor *
	 *****************/
	
	if (myHandDetector.numHands == 2) {
		// Update cursor
		float newSize = ofDist(myHandDetector.handBlobs[0].averagePt.x,myHandDetector.handBlobs[0].averagePt.y,myHandDetector.handBlobs[1].averagePt.x,myHandDetector.handBlobs[1].averagePt.y) + CURSOR_OFFSET_SIZE;
		float newX = (myHandDetector.handBlobs[0].averagePt.x + myHandDetector.handBlobs[1].averagePt.x) / 2 - newSize / 2 + CURSOR_OFFSET_X;
		float newY = (myHandDetector.handBlobs[0].averagePt.y + myHandDetector.handBlobs[1].averagePt.y) / 2 - newSize / 2 + CURSOR_OFFSET_Y;
		float newAngle = atan2(myHandDetector.handBlobs[1].averagePt.y - myHandDetector.handBlobs[0].averagePt.y, myHandDetector.handBlobs[1].averagePt.x - myHandDetector.handBlobs[0].averagePt.x);
		
		// Smooth
		cursorRect.width += (newSize - cursorRect.width) / CURSOR_DELAY;
		cursorRect.height += (newSize - cursorRect.height) / CURSOR_DELAY;
		cursorRect.x += (newX - cursorRect.x) / CURSOR_DELAY;
		cursorRect.y += (newY - cursorRect.y) / CURSOR_DELAY;
		cursorAngle += (newAngle - cursorAngle) / CURSOR_DELAY;
		
		mapToRelief(cursorRect, &reliefCursorRect);
		mapAngleToRelief(cursorAngle, &reliefCursorAngle);
	} else {
		cursorRect.width = 0;
		cursorRect.height = 0;
	}
	
	/*****************
	 * State machine *
	 *****************/
	updateFromReliefHeight();	
	if(state == 0){ //entering frame, waiting for two tracked hands
		if(myHandDetector.numHands == 2) state = 1;
		selectionOn = false;
		manipulationOn = false;
		
	}
	else if(state == 1){ //found two hands, woo!
		if(manipulationOn)
			frames_idle = 0; //no longer idle
		if(myHandDetector.numHands == 2){ //if we still got them
			if(myHandDetector.handBlobs[0].pinch && myHandDetector.handBlobs[1].pinch){ // if pinch is true, we control the height
				if (!manipulationOn) {
					// Store current cursorRect
					for (int h = 0; h < myHandDetector.numHands; h++) {
						lockedSelectionRect = reliefCursorRect;
						lockedSelectionAngle = reliefCursorAngle;
						lockedSelectionHeight = myHandDetector.averageHandHeight;
					}
					
					// Store current pin heights
					for (int x = 0; x < RELIEF_SIZE_X; x++) {
						for (int y = 0; y < RELIEF_SIZE_Y; y++) {
							lockedPinHeight[x][y] = mPinHeightToRelief[x][RELIEF_SIZE_Y - 1 - y];
						}
					}
				}
				
				selectionOn = false;
				manipulationOn = true;
			}
			else {
				//updateFromReliefHeight();
				selectionOn = true;
				manipulationOn = false;
			}
		}
		else{
			state = 0; // lost them, let's g back to state #1
			selectionOn = false;
			manipulationOn = false;
			resetProjectionPixels();
		}
	}
	
	/**********************
	 * Output Calculation *
	 **********************/
	
	resetProjectionPixels();
	for (int x = 0; x < RELIEF_SIZE_X; x++) {
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			projectionPixelMatrix[x][y].set(191 - mPinHeightToRelief[x][RELIEF_SIZE_Y - 1 - y]);
		}
	}
	
	if(manipulationOn) {
		transform();
	}
	
	
	/*****************
	 * Relief update *
	 *****************/
	if(recording == 1) { // if recording, record the frame
		pushFrame(current_instance);
	}	
	if(animating != 0) {
		frames_idle = 0;
		if (current_frame == instances[current_instance].frames.size()-1) { //animating finished
			animating = 0;
			startLoading();
			current_frame = 0;
		}
		changeFrame(animating);
		visualizationOffset = 0;
	}
	if(loading) {
		processLoading();
	} else if(editing == 1 && current_frame == 0 && frames_idle < IDLE_THRESHOLD) {
		//updateCurrentMesh();		
		instances[current_instance].frames[current_frame] = reliefatov(mPinHeightToRelief);
		updateMesh(current_instance);
	}
	
	//Handling Visualization Motion
	if(recording == 0) { //only if not recording
		if(visualizationMode == 0) {
			int threshold = 0.1*SCREEN_HEIGHT/(min(MAX_VISIBLE_INSTANCES,(int)instances.size()));
			if(transitioning != 0) {
				if(abs(visualizationOffset) < threshold){
					transitioning = 0;
					annotationFrame = 1;
				}
				visualizationOffset *= 0.93;
			} else {
				if(visualizationOffset > threshold){
					changeInstance(1);
				} else if (visualizationOffset < -threshold) {
					changeInstance(-1);
				}
				visualizationOffset *= 0.93;
			}			
		} else if(visualizationMode == 1){
			int dist = (int)visualizationOffset/OFFSET_PER_FRAME;
			int last_frame = (int)instances[current_instance].frames.size()-1;
			dist = min(dist, last_frame-current_frame);
			dist = max(dist, 0-current_frame);
			if(dist != 0)
				changeFrame(dist);
			if((current_frame == 0 && visualizationOffset < 0)||(current_frame == last_frame && visualizationOffset > 0)) {
				visualizationOffset *= 0.7;
			}
		}
	}
	
	//Idle behavior
	if(visualizationMode == 0)
		frames_idle++;
	if(frames_idle >= IDLE_THRESHOLD && (frames_idle-IDLE_THRESHOLD) % IDLE_ROTATION_TIME == 0) {
		changeInstance(1);
		frames_idle = IDLE_THRESHOLD;
	}
	
	float sensativity = 0.15;
	int frame_buffer = 15;
	float average = 0;
	int num = 0;
	for(int x = 0; x < RELIEF_SIZE_X; x++) {
		for(int y = 0; y < RELIEF_SIZE_Y; y++) {
			if (mPinMask[x][y]) {
				num++;
				average+=abs(mPinHeightFromRelief[x][y]-previousHeightFromRelief[x][y]);
				previousHeightFromRelief[x][y] = mPinHeightFromRelief[x][y];
			}
		}
	}	
	average /= num;
	if(average < sensativity) {
		if(stable_frames > frame_buffer) {
			recording = 0;//David's automatic record stop
		}
		stable_frames++;
	} else {		
		if(stable_frames > frame_buffer) {
			if(visualizationMode == 1) {
				//recording = 1;//David's automatic record start
				
			}
			
		}
		if(!loading) {
			frames_idle = 0;
			if(visualizationMode == 1) {
				recording = 1;
			}
		}
		stable_frames = 0;		
	}
	
	if(changingFrame > 0) {
		vector<vector<float> > &currentv = current_mesh.vertices;
		vector<vector<ofVec3f> > &currentn = current_mesh.normals;
		vector<vector<float> > &oldv = meshes[current_instance].vertices;
		vector<vector<ofVec3f> > &oldn = meshes[current_instance].normals;
		for(int x = 0; x < currentv.size(); x++) {
			for(int y = 0; y < currentv.size(); y++) {
				float diff = currentv[x][y] - oldv[x][y];
				oldv[x][y] += diff*0.2;
			}
		}
		
		
		if(changingFrame == 1) {
			instances[current_instance].frames[current_frame] = reliefatov(mPinHeightToRelief);
		} else if (changingFrame >= 15) {
			changingFrame = -1; //because the increment below pushes it to 0
		}
		changingFrame++;
	}
	if(visualizationMode == 1) {
		updateCurrentMesh();
	} else {
		updateAnnotations();
	}

	processMessages();
	if(RELIEF_CONNECTED){
		//mIOManager->sendPinHeightToRelief(mPinHeightToRelief);
	}
}

//--------------------------------------------------------------
void gesturalReliefApp::draw(){
//#ifdef SPAN_SCREEN
	/*// Kinect depth maps and contours
	grayImage.draw(0, 0, kinect.width, kinect.height);
	calibratedColorImage.draw(kinect.width, 0, kinect.width, kinect.height);
	
	// ROI rectangle
	ofRect(roiRect.x, roiRect.y - 1, roiRect.width + 1, roiRect.height + 1);
	ofRect(roiRect.x + kinect.width, roiRect.y - 1, roiRect.width + 1, roiRect.height + 1);
	
	// ROI rectangle border
	ofRect(roiRect.x - ROI_BORDER_W, roiRect.y - 1 - ROI_BORDER_H, roiRect.width + ROI_BORDER_W * 2, roiRect.height + ROI_BORDER_H * 2);
	
	// Need to translate because finger co-ordinates are offset by the ROI rect
	ofPushMatrix();
	ofTranslate(roiRect.x - ROI_BORDER_W, roiRect.y - ROI_BORDER_H);
	
	// Draw hand info	
	for (int h = 0; h < myHandDetector.numHands; h++) {
		
		// Draw fingers
		for (int i = 0; i < myHandDetector.handBlobs[h].nFingers; i++) {
			// Draw red circle on each finger
			ofEnableAlphaBlending();
			ofSetColor(0, 154, 233, 127);
			ofFill();
			ofCircle(myHandDetector.handBlobs[h].fingerPos[i].x, myHandDetector.handBlobs[h].fingerPos[i].y, 10); 
			
			ofDisableAlphaBlending();
		}
				
		// Draw average circle
		if (myHandDetector.handBlobs[h].pinch)
			ofSetColor(9, 231, 73);
		else
			ofSetColor(231, 9, 73);

		ofFill();
		ofCircle(myHandDetector.handBlobs[h].averagePt.x, myHandDetector.handBlobs[h].averagePt.y, 10);
		ofNoFill();
		ofSetColor(255, 255, 255);
		
	}
	
	ofPopMatrix();
	// Kinect settings
	ofSetColor(255, 255, 255);
	ofDrawBitmapString("accel is: " + ofToString(kinect.getMksAccel().x, 2) + " / " 
					   + ofToString(kinect.getMksAccel().y, 2) + " / "
					   + ofToString(kinect.getMksAccel().z, 2), 20, kinect.height + 20 );
	
	char reportStr[1024];
	sprintf(reportStr, "using opencv threshold = %i (press spacebar)\nset near threshold %i (press: l k)\nset far threshold %i (press: < >) num blobs found %i, fps: %f",bThreshWithOpenCV, nearThreshold, farThreshold, myHandDetector.contourFinder.nBlobs, ofGetFrameRate());
	ofDrawBitmapString(reportStr, 20, kinect.height + 40);
	//*/
    visualizeOnRelief();
    if (selectionOn || manipulationOn || (animating != 0))// && instances[current_instance].cursorRect[current_frame].width != -1))
		visualizeSelectionFeedback();
	

	char reportStr[1024];
	int text_position = 0;
	text_position += 20;
	sprintf(reportStr,"FPS: %.2f",ofGetFrameRate());
	ofDrawBitmapString(reportStr, 20,text_position);
	if(recording == 1) {
		text_position += 20;
		sprintf(reportStr,"Recording animation...");
		ofDrawBitmapString(reportStr, 20, text_position);
	} else if (animating != 0) {
		text_position += 20;
		sprintf(reportStr,"Playing animation...");
		ofDrawBitmapString(reportStr, 20, text_position);
	} 
	if (animating == 0 && loading) {
		text_position += 20;
		sprintf(reportStr,"Loading...");
		ofDrawBitmapString(reportStr, 20, text_position);
	}
	if (frames_idle > IDLE_THRESHOLD) {
		text_position += 20;
		sprintf(reportStr,"Idle...");
		ofDrawBitmapString(reportStr, 20, text_position);
	}	
	int frame_buffer = 15;
	if(visualizationMode == 1 && !loading && stable_frames > frame_buffer) {
		text_position += 20;
		sprintf(reportStr,"Recording Ready...");
		ofDrawBitmapString(reportStr, 20, text_position);
	}
    
	if(visualizationMode == 0) {
		
        drawInstances();
	} else if(visualizationMode == 1) {
		drawFrames();
    }
	
	// Draw selection feedback
	//#endif

	
}

//--------------------------------------------------------------
void gesturalReliefApp::visualizeSelectionFeedback()
{	
	ofPushMatrix();
	ofTranslate(projectionRect.x, projectionRect.y, 0);
	ofEnableSmoothing();
	//If playing back an animation
	if (animating != 0) {
		cursorRect = instances[current_instance].cursorRect[current_frame];
		manipulationOn = instances[current_instance].manipulationOn[current_frame];
	}
	ofRectangle projectionCursorRect;
	mapToProjection(cursorRect, &projectionCursorRect);
	
	if (manipulationOn) {
		previousSelectionColor += (0 - previousSelectionColor) / COLOR_CHANGE_DELAY_ON;
	}
	else {
		previousSelectionColor += (255 - previousSelectionColor) / COLOR_CHANGE_DELAY_OFF;
	}
	
	ofSetColor(previousSelectionColor, 255, previousSelectionColor);
	//printf("%.2f %.2f\n\n",projectionCursorRect.x,projectionCursorRect.width);
	int val = SCREEN_WIDTH - (projectionCursorRect.x + projectionRect.x);
	if(val > 0) {
		projectionCursorRect.width -= val;
		projectionCursorRect.x += val;
	}	
	ofFill();
	ofRect(projectionCursorRect.x, projectionCursorRect.y, projectionCursorRect.width, projectionCursorRect.height);
	ofNoFill();
	
	ofSetColor(255, 255, 255);
			 
	ofDisableSmoothing();
	ofPopMatrix();
}

//--------------------------------------------------------------
void gesturalReliefApp::visualizeOnRelief()
{	
	float projectionPixelSize = (float) projectionRect.width / RELIEF_SIZE_X;
	
	ofPushMatrix();
	ofTranslate(projectionRect.x, projectionRect.y, 0);
	
	for(int x = 0; x < RELIEF_SIZE_X; x++) {
		for(int y = 0; y < RELIEF_SIZE_Y; y++) {
			if (mPinMask[x][y]) {
				
				ofSetColor(projectionPixelMatrix[x][y].r,
						   projectionPixelMatrix[x][y].g,
						   projectionPixelMatrix[x][y].b,
						   projectionPixelMatrix[x][y].a);
				ofFill();
				
				float xChange = (parallaxRect.x - x) * (127 - mPinHeightToRelief[x][RELIEF_SIZE_Y - 1 - y]) * parallaxRect.width;
				float yChange = (parallaxRect.y - y) * (127 - mPinHeightToRelief[x][RELIEF_SIZE_Y - 1 - y]) * parallaxRect.height;
				
				ofRect(x * projectionPixelSize + PROJECTION_PIXEL_BORDER/2 - xChange,
					   y * projectionPixelSize + PROJECTION_PIXEL_BORDER/2 - yChange,
					   projectionPixelSize - PROJECTION_PIXEL_BORDER*2 + xChange * 2,
					   projectionPixelSize - PROJECTION_PIXEL_BORDER*2 + yChange * 2);
				
				ofSetColor(255, 255, 255);
				ofNoFill();
			
			}
		}
	}
	
	ofPopMatrix();

}

//--------------------------------------------------------------
void gesturalReliefApp::transform() {
	transformImage.set(0);
	unsigned char * transformBuffer = transformImage.getPixels();
	
	float translateX = reliefCursorRect.x - lockedSelectionRect.x;
	float translateY = reliefCursorRect.y - lockedSelectionRect.y;
	float translateZ = -(myHandDetector.averageHandHeight - lockedSelectionHeight); //this is flipped because lower values for the relief correspond to higher pins
	
	int newHeight;
	for (int x = 0; x < RELIEF_SIZE_X; x++) {
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			if (lockedSelectionRect.inside(x, y)) {								
				// Calculate new height
				newHeight = (int)(lockedPinHeight[x][y] + translateZ / RELIEF_HEIGHT_RATIO);
				// Bound to between 0 and 127
				if (newHeight > 127)
					newHeight = 127;
				else if (newHeight < 1)
					newHeight = 1;
				// Save transformed height to transform buffer
				transformBuffer[x + y * RELIEF_SIZE_Y] = newHeight;
			}
		}
	}
	transformImage.setFromPixels(transformBuffer, RELIEF_SIZE_X, RELIEF_SIZE_Y);
	//transformImage.rotate(reliefCursorAngle, lockedSelectionRect.getCenter().x, lockedSelectionRect.getCenter().y);
	transformImage.translate(-lockedSelectionRect.x, -lockedSelectionRect.y);
	transformImage.scale(reliefCursorRect.width / lockedSelectionRect.width, reliefCursorRect.width / lockedSelectionRect.width);
	transformImage.translate(lockedSelectionRect.x, lockedSelectionRect.y);
						 
	transformImage.translate(translateX, translateY);
	
	transformBuffer = transformImage.getPixels();
	
	for (int x = 0; x < RELIEF_SIZE_X; x++) {
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			if (lockedSelectionRect.inside(x, y) && transformBuffer[x + y * RELIEF_SIZE_Y] <= 0)
				newHeight = 127;
			else if (transformBuffer[x + y * RELIEF_SIZE_Y] > 0)
				newHeight = transformBuffer[x + y * RELIEF_SIZE_Y];
			else
				newHeight = lockedPinHeight[x][y];
			
			mPinHeightToRelief[x][RELIEF_SIZE_Y - 1 - y] += (newHeight - mPinHeightToRelief[x][RELIEF_SIZE_Y - 1 - y]) / HEIGHT_DELAY;
		}
	}
}

//--------------------------------------------------------------
void gesturalReliefApp::roundedRect(float x, float y, float w, float h, float r) {
    ofBeginShape();
	ofVertex(x+r, y);
	ofVertex(x+w-r, y);
	quadraticBezierVertex(x+w, y, x+w, y+r, x+w-r, y);
	ofVertex(x+w, y+h-r);
	quadraticBezierVertex(x+w, y+h, x+w-r, y+h, x+w, y+h-r);
	ofVertex(x+r, y+h);
	quadraticBezierVertex(x, y+h, x, y+h-r, x+r, y+h);
	ofVertex(x, y+r);
	quadraticBezierVertex(x, y, x+r, y, x, y+r);
    ofEndShape();
}

//--------------------------------------------------------------
void gesturalReliefApp::quadraticBezierVertex(float cpx, float cpy, float x, float y, float prevX, float prevY) {
	float cp1x = prevX + 2.0/3.0*(cpx - prevX);
	float cp1y = prevY + 2.0/3.0*(cpy - prevY);
	float cp2x = cp1x + (x - prevX)/3.0;
	float cp2y = cp1y + (y - prevY)/3.0;
	
	// finally call cubic Bezier curve function
	ofBezierVertex(cp1x, cp1y, cp2x, cp2y, x, y);
};

//--------------------------------------------------------------
void gesturalReliefApp::updateFromReliefHeight() {
	//mIOManager->getPinHeightFromRelief(mPinHeightFromRelief);    
	if(!loading || adjust_frame != 0){ //allow manipulation if not loading or in adjust phase
		for (int x = 0; x < RELIEF_SIZE_X; x++) {
			for (int y = 0; y < RELIEF_SIZE_Y; y++) {
				mPinHeightToRelief[x][y] += (mPinHeightFromRelief[x][y] - mPinHeightToRelief[x][y]) / DIRECT_MANIPULATION_DELAY;
			}
		}
	}
}

void gesturalReliefApp::resetProjectionPixels() {
	for (int x = 0; x < RELIEF_SIZE_X; x++) {
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			projectionPixelMatrix[x][y].set(63);
		}
	}
}

//--------------------------------------------------------------
void gesturalReliefApp::mapToProjection(ofPoint realPt, ofPoint * projectionPt) {
	(*projectionPt).x = (int)floor(ofMap(realPt.x, ROI_BORDER_W, roiRect.width + ROI_BORDER_W, projectionRect.width,0));
	(*projectionPt).y = (int)floor(ofMap(realPt.y, ROI_BORDER_H, roiRect.height + ROI_BORDER_H, 0, projectionRect.height));
}

//--------------------------------------------------------------
void gesturalReliefApp::mapToProjection(ofRectangle realRect, ofRectangle * _projectionRect) {	
	(*_projectionRect).x = (int)floor(ofMap(realRect.x, ROI_BORDER_W, roiRect.width + ROI_BORDER_W, projectionRect.width,0));
	(*_projectionRect).y = (int)floor(ofMap(realRect.y, ROI_BORDER_H, roiRect.height + ROI_BORDER_H, 0, projectionRect.height));
	(*_projectionRect).width = (int)floor(realRect.width / roiRect.width * projectionRect.width);
	(*_projectionRect).height = (int)floor(realRect.height / roiRect.height * projectionRect.height);
	(*_projectionRect).x -= (*_projectionRect).width; //We need to adjust the x because we are mirroring horizontally 
}

//--------------------------------------------------------------
void gesturalReliefApp::mapToRelief(ofPoint realPt, ofPoint * reliefPt) {
	(*reliefPt).x = (int)floor(ofMap(realPt.x, ROI_BORDER_W, ROI_BORDER_W + roiRect.width, RELIEF_SIZE_X,0));
	(*reliefPt).y = (int)floor(ofMap(realPt.y, ROI_BORDER_H, ROI_BORDER_H + roiRect.height, 0, RELIEF_SIZE_Y));
}
	
//--------------------------------------------------------------
void gesturalReliefApp::mapToRelief(ofRectangle realRect, ofRectangle * reliefRect) {	
	(*reliefRect).x = (int)floor(ofMap(realRect.x, ROI_BORDER_W, ROI_BORDER_W + roiRect.width, RELIEF_SIZE_X,0));
	(*reliefRect).y = (int)floor(ofMap(realRect.y, ROI_BORDER_H, ROI_BORDER_H + roiRect.height, 0, RELIEF_SIZE_Y));
	(*reliefRect).width = (int)floor(realRect.width / roiRect.width * RELIEF_SIZE_X);
	(*reliefRect).height = (int)floor(realRect.height / roiRect.height * RELIEF_SIZE_Y);
	(*reliefRect).x -= (*reliefRect).width; //We need to adjust the x because we are mirroring horizontally 
}

//--------------------------------------------------------------
void gesturalReliefApp::mapAngleToRelief(float realAngle, float * reliefAngle) {
	if (realAngle >= -PI/3 && realAngle < 0)
		*reliefAngle = 0;
	else if (realAngle >= -3*PI/4 && realAngle < -PI/3) {
		if (myHandDetector.handBlobs[1].averagePt.x > myHandDetector.handBlobs[0].averagePt.x){
			if (myHandDetector.handBlobs[1].averagePt.y < myHandDetector.handBlobs[0].averagePt.y)
			*reliefAngle = PI/2;
		}
		else
			*reliefAngle = -PI/2;
	}
}

ofVec3f gesturalReliefApp::mapHeightToColor(float height) {
	ofVec3f white(1,1,1);
	//ofVec3f lightb(0.24,0.69,1);
	ofVec3f lightb(0.2,0.3,0.7);	
	white *= ofMap(height, 0, 120, 0, 1, 1);
	lightb *= ofMap(height, 0, 120, 1, 0, 1);
	return white+lightb;
	
}

ofVec3f gesturalReliefApp::mapReliefTo3d(ofVec3f point) {
	return (* (new ofVec3f(-(FRAME_WIDTH/2) + point.x*GRID_WIDTH,120-point.y,(FRAME_WIDTH/2)-point.y*GRID_WIDTH)));

}

ofVec3f gesturalReliefApp::map3dToRelief(ofVec3f point) {
	
}

//--------------------------------------------------------------
void gesturalReliefApp::keyPressed(int key){
	frames_idle = 0;
	/*// Settings for kinect
	switch (key)
	{
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
		case '<':		
		case ',':		
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case 'l':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
		case 'k':		
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
		break;
	}
	//*/
	/*switch	(key){
	 case 'w':
	 instanceCamera.y++;
	 break;
	 case 's':
	 instanceCamera.y--;
	 break;
	 case 'a':
	 instanceCamera.z--;
	 break;
	 case 'd':
	 instanceCamera.z++;
	 break;
	 case 'r':
			instanceCamera.angle++;
		break;
	case 'f':
			instanceCamera.angle--;
		break;
		
	}*/
	
	//Settings for moving projected image on XY
	/*switch	(key){
		case 'w':
			projectionRect.y++;
			break;
		case 's':
			projectionRect.y--;
			break;
		case 'a':
			projectionRect.x--;
			break;
		case 'd':
			projectionRect.x++;
			break;
		case 'z':
			projectionRect.width++;
			projectionRect.height++;
			break;
		case 'x':
			projectionRect.width--;
			projectionRect.height--;
			break;
	}*/
	
	// Settings for selection box
	/*switch (key) {
		case 356:
			// left
			if (roiRect.x > 0)
				roiRect.x--;
			break;
		case 357:
			// up
			if (roiRect.y > 0)
				roiRect.y--;
			break;
		case 358:
			// right
			if (roiRect.x + roiRect.width < kinect.width)
				roiRect.x++;
			break;
		case 359:
			// down
			if (roiRect.y + roiRect.height < kinect.height)
				roiRect.y++;
			break;
		case '=':
			if (roiRect.x + roiRect.width < kinect.width && roiRect.y + roiRect.height < kinect.height) {
				roiRect.width++;
				roiRect.height++;
			}
			break;
		case '-':
			if (roiRect.width > 0 && roiRect.height > 0) {
				roiRect.width--;
				roiRect.height--;
			}
			break;
		
		default:
			break;
	}*/
	myHandDetector.setROI(roiRect.x - ROI_BORDER_W, roiRect.y - ROI_BORDER_H, roiRect.width + ROI_BORDER_W * 2, roiRect.height + ROI_BORDER_H * 2);
	
	// Settings for parallax of projected image
	/*switch	(key){
		case 't':
			parallaxRect.y += 1;
			break;
		case 'g':
			parallaxRect.y -= 1;
			break;
		case 'f':
			parallaxRect.x -= 1;
			break;
		case 'h':
			parallaxRect.x += 1;
			break;
		case 'c':
			parallaxRect.width += 0.001;
			break;
		case 'v':
			parallaxRect.width -= 0.001;
			break;
		case 'b':
			parallaxRect.height += 0.001;
			break;
		case 'n':
			parallaxRect.height -= 0.001;
			break;
	}*/
	
	// Saving and loading test
	switch (key) {
		case '=':
			loading = 0;
			break;
		case 357: //up arrow
			pushInstance();
			break;
		case 356: //left arrow
			changeInstance(-1);
			break;
		case 359: //down arrow
			//popInstance();
			break;
		case 358: //right arrow
			changeInstance(1);
			break;
		case '0':
			//interpolateInstances(20);
			break;
		case '9':
			animate(1);
			break;
		case '8':
			animate(-1);
			break;
		case '7':
			buildshape();
			break;
		case '2':
			visualizationOffset = 0;
			if (visualizationMode == 0) {
				visualizationMode = 1;
			} else if (visualizationMode == 1) {
				visualizationMode = 0;
				resetAnimation();
			}
			break;
		case '+':
			if(editing){
				editing = 0;
			} else {
				editing = 1;
			}
			break;
		case '1':
			/*if(recording != 1) {
				resetInstance(current_instance);
				lineHeights.clear();
				recording = 1;
			} else {
				recording = 0;
				resetAnimation();
			}*/
			break;
		case '3':
			resetAnimation();
			break;
		case 'c':
			//resetInstances();
			//break;
		case 'i':
			//changeInstance(1);
			frames_idle = IDLE_THRESHOLD;
			break;
		case 'p':
			annotationFrame = 1;
			break;
		case ' ':
			//pushChanges();
			visualizationOffset = 0;
			if (visualizationMode == 0) {
				visualizationMode = 1;
				generateLineHeights();
			} else if (visualizationMode == 1) {
				visualizationMode = 0;
				resetAnimation();
			}
			break;
		default:
			break;
	}
	/*cout << "H: " << roiRect.height << " W: " << roiRect.width << " X: "<< roiRect.x << " Y: " << roiRect.y << endl;
	cout << "projectionXPos: " << projectionRect.x << " projectionYPos: " << projectionRect.y << " projectionWidth:" << projectionRect.width << endl;
	cout << "parallaxRectX: " << parallaxRect.x << " parallaxRectY: " << parallaxRect.y << " parallaxRectW:" << parallaxRect.width << " parallaxRectH:" << parallaxRect.height << endl;*/
	printf("instanceCamera.y: %.2f instanceCamera.z: %.2f instanceCamera.angle: %.2f\n",instanceCamera.y,instanceCamera.z,instanceCamera.angle);
}

void gesturalReliefApp::buildshape() {
	//sphere radius 100
	float radius = 100;
	unsigned char relief[RELIEF_SIZE_X][RELIEF_SIZE_Y];
	for (int x = 0; x < RELIEF_SIZE_X; x++) { 
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			if(mPinMask[x][y] == 1) { 
				float xpos = ofMap(x,0,RELIEF_SIZE_X-1,-radius,radius);
				float ypos = ofMap(y,0,RELIEF_SIZE_X-1,-radius,radius);
				double zpos = sqrt(pow(radius,2) - pow(ypos,2) - pow(xpos, 2));
				if(zpos > 0) {
					relief[x][y] = (unsigned char)(103-zpos);
				} else {
					relief[x][y] = 100;
				}
			}
		}
	}
	//cout << "here" << endl;
	pushInstance();
	changeInstance(1);
	instances[current_instance].frames[0] = reliefatov(relief);
	updateMesh(current_instance);
	
	//square
	for (int x = 0; x < RELIEF_SIZE_X; x++) { 
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			if(mPinMask[x][y] == 1) {
				if (x>2 && x < 9 && y >2 && y < 9) {
					relief[x][y] = 3;
				} else {
					relief[x][y] = 100;
				}
			}
		}
	}
	pushInstance();	
	changeInstance(1);
	instances[current_instance].frames[0] = reliefatov(relief);
	updateMesh(current_instance);
	
	//cone center 6,6

	for (int x = 0; x < RELIEF_SIZE_X; x++) { 
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			if(mPinMask[x][y] == 1) {
				float xpos = ofMap(x,0,RELIEF_SIZE_X-1,-radius,radius);
				float ypos = ofMap(y,0,RELIEF_SIZE_X-1,-radius,radius);
				double zpos = 1.5*sqrt(pow(ypos,2) + pow(xpos, 2));
				if(zpos > 100){
					zpos = 100;
				}
				relief[x][y] = zpos;
			}
		}
	}
	pushInstance();	
	changeInstance(1);
	instances[current_instance].frames[0] = reliefatov(relief);
	updateMesh(current_instance);
	
	for (int x = 0; x < RELIEF_SIZE_X; x++) { 
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			if(mPinMask[x][y] == 1) {
				if (x == 6) {
					relief[x][y] = 10*(abs(y-7))+3;
				} else {
					relief[x][y] = 100;
				}

				
			}
		}
	}
	pushInstance();	
	changeInstance(1);
	instances[current_instance].frames[0] = reliefatov(relief);
	updateMesh(current_instance);
	
	changeInstance(1);
	startLoading();
	//visualizationOffset = 0;
}

void gesturalReliefApp::animate(int dir){
	animating = dir;
}

void gesturalReliefApp::pushFrame(int inst){
	frame relief = reliefatov(mPinHeightToRelief);
	for (int x = 0; x < RELIEF_SIZE_X; x++) { 
		for (int y = 0; y < RELIEF_SIZE_Y; y++) {
			if(relief[x][y] > 120) { //Set the floor for the motor
				relief[x][y] = 120;
			}
		}
	}
	if(current_frame >= (int)(instances[inst].frames.size())-1) {
		instances[inst].frames.push_back(relief);
		current_frame = instances[inst].frames.size()-1;
		instances[inst].manipulationOn.push_back(manipulationOn);
		instances[inst].cursorRect.push_back(cursorRect);
	} else {
		current_frame++;
		instances[inst].frames[current_frame] = relief;
		instances[inst].manipulationOn[current_frame] = manipulationOn;
		instances[inst].cursorRect[current_frame] = cursorRect;
	}
	
}

void gesturalReliefApp::pushInstance() {
	instance n;
	instances.push_back(n);
	pushFrame(instances.size()-1);
	pushMesh();
}

void gesturalReliefApp::pushChanges() {
	changingFrame = 1;
	/*instances[current_instance].frames[current_frame] = reliefatov(mPinHeightToRelief);
	updateMesh(current_instance);*/
}

void gesturalReliefApp::pushMesh() {
	meshes.push_back(*(new mesh));
	meshList.push_back(glGenLists(1));
	updateMesh(meshList.size()-1);
}

void gesturalReliefApp::changeFrame(int dist){
	startLoading();
	current_frame += dist;
	current_frame = (current_frame+instances[current_instance].frames.size())%instances[current_instance].frames.size();
	visualizationOffset += -OFFSET_PER_FRAME*dist; // handle visualization offset
}

void gesturalReliefApp::changeInstance(int dist){
	startLoading();
	int prev = current_instance;
	current_instance += dist;
	current_instance = (current_instance+instances.size())%instances.size();
	transitioning = dist;
	current_frame=0;
	float height = SCREEN_HEIGHT*0.95;
	visualizationOffset += -height/min(MAX_VISIBLE_INSTANCES,(int)instances.size()-1)*dist; // handle visualization offset (this needs to be changed to be correct, but it's close
	updateMesh(prev);
	annotationFrame = 0;
	changingFrame = 0;
}

void gesturalReliefApp::resetInstances() {
	instances.clear();
	pushInstance();
}

void gesturalReliefApp::resetInstance(int inst) {
	instances[inst].frames.clear();
	instances[inst].cursorRect.clear();
	instances[inst].manipulationOn.clear();
	current_frame = -1;
	pushFrame(inst);
	startLoading();
}

void gesturalReliefApp::reliefvtoa(vector<vector<unsigned char> > vec, unsigned char arr[RELIEF_SIZE_X][RELIEF_SIZE_Y]){
	for (int x = 0; x < RELIEF_SIZE_X; x++) {
		for(int y = 0;y < RELIEF_SIZE_Y; y++) {
			arr[x][y] = vec[x][y];
		}
	}
}

vector<vector<unsigned char> > gesturalReliefApp::reliefatov(unsigned char arr[RELIEF_SIZE_X][RELIEF_SIZE_Y]){
	vector<vector<unsigned char> > relief;	
	for (int x = 0; x < RELIEF_SIZE_X; x++) {
		vector<unsigned char> row;
		for(int y = 0;y < RELIEF_SIZE_Y; y++) {
			row.push_back(arr[x][y]);
		}
		relief.push_back(row);
	}
	return relief;
}

//Daniel Lines version
void gesturalReliefApp::drawFrames() {
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	int shift = 1000;
	float pixels_per_frame = 20;
	int drawDistance = 200*pixels_per_frame;
	glFrustum(-SCREEN_WIDTH/2, SCREEN_WIDTH/2, -SCREEN_HEIGHT/2, SCREEN_HEIGHT/2,shift, shift+SCREEN_HEIGHT*5);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity();
	glTranslatef(0,0, -shift);
	double angle = frameCamera.angle*PI/180;
	gluLookAt(0.0, frameCamera.y, frameCamera.z, 0, frameCamera.y+sin(angle), frameCamera.z+cos(angle), 0, 1, 0);
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	
	double alpha = 1.0;
	int num_frames = instances[current_instance].frames.size();	
	float frame_resolution = 10;
	int duration_length = SCREEN_HEIGHT*0.6;
	float partial_frame = current_frame+ofMap(visualizationOffset, 0, OFFSET_PER_FRAME, 0, 1);
	if(partial_frame < 0) {
		partial_frame = 0;
	}
	vector<vector<float> > &vertices = current_mesh.vertices;
	vector<vector<ofVec3f> > &normals = current_mesh.normals;
	for(int frame_pos = 0; frame_pos <= num_frames -1; frame_pos += frame_resolution){
		int offset = -(frame_pos-current_frame)*pixels_per_frame;
		alpha = ofMap((float)abs(frame_pos-current_frame),0,frame_resolution*3,1,0.15);
		if(alpha < 0.15)
			alpha = 0.15;
		alpha *= ofMap(offset,0,drawDistance,1,0);
		if(alpha < 0.01) {
			alpha = 0;
			
		}
		
		int line_heights_position = frame_pos/frame_resolution;
		float meshwidth = (float)FRAME_WIDTH/vertices.size();
		int center_index = vertices.size()/2;
		vector<float> line;
		for(int x = 0; x < vertices.size();x++) {
			line.push_back(vertices[x][center_index]);
		}
		if(frame_pos == current_frame){
			if(line_heights_position >= lineHeights.size()) {
				lineHeights.push_back(line);
			} else {
				lineHeights[line_heights_position] = line;
			}
		}
		line = lineHeights[line_heights_position];
		int i = center_index;
		if(alpha > 0){
			glPushMatrix();
			glTranslatef(0, 0, offset);
			glColor3f(alpha, alpha, alpha);
			glDepthMask(GL_FALSE);
			glBegin(GL_LINES);		
			for(int j = 0;j < vertices[i].size()-1; j++) {
				float xoffset = (FRAME_WIDTH/2) - j*meshwidth;
				float zoffset = -((FRAME_WIDTH/2)-i*meshwidth);			
				glVertex3f(-xoffset,line[j],-zoffset);			
				glVertex3f(-xoffset+meshwidth,line[j+1],-zoffset);
			}
			glEnd();
			glBegin(GL_POINTS);
			for(int j = 0;j < vertices[i].size()-1; j++) {
				float xoffset = (FRAME_WIDTH/2) - j*meshwidth;
				float zoffset = -((FRAME_WIDTH/2)-i*meshwidth);			
				glVertex3f(-xoffset,line[j],-zoffset);
			}
			glEnd();
			glDepthMask(GL_TRUE);
			glPopMatrix();
		}
	}
	
	//draw current relief
	
	glPushMatrix();
		drawMeshWire(current_mesh, 1, 1); 
	glPopMatrix();
	glColor3f(1, 1, 1);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glDisable(GL_COLOR_MATERIAL);
	
	//draw time stamp
	float time = (float)partial_frame/FPS;
	char reportStr[1024];
	sprintf(reportStr,"%.2fs",time);
	ofDrawBitmapString(reportStr, FRAME_WIDTH/2+75,15);//95 to compensate for the relief heights of the lines
	
	glPopMatrix();
}


void gesturalReliefApp::drawInstances(){
	
	int visible_instances = min(MAX_VISIBLE_INSTANCES,(int)instances.size());	
	int	target_instance = current_instance;	
	int display_size = SCREEN_HEIGHT*0.95;
	int offset_per_frame = (float)display_size/(visible_instances-1);

	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	int shift = 5000;
	glFrustum(-SCREEN_WIDTH/2, SCREEN_WIDTH/2, -SCREEN_HEIGHT/2, SCREEN_HEIGHT/2+200,shift, shift+SCREEN_HEIGHT);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT+200); //the 200s are added because the vertical screen was clipping the top of the visualization, this surely can be solved in a nicer way.
	
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity();
	glPushMatrix();
	glTranslatef(0,0, -shift);
	double angle = instanceCamera.angle*PI/180;
	gluLookAt(0.0, instanceCamera.y, instanceCamera.z, 0, instanceCamera.y+sin(angle), instanceCamera.z+cos(angle), 0, 1, 0);
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	
	
	double alpha = 0.1;
	for(int inst_pos = target_instance-1; inst_pos < target_instance+visible_instances; inst_pos++){ //the bounds are set so we will wrap around the screen
		frame relief;
		int draw_pos = (inst_pos+instances.size()) % instances.size();
		if (draw_pos < 0) {
			draw_pos += instances.size();
		}
		int xpos =0;
		int ypos = offset_per_frame*(inst_pos-target_instance) - visualizationOffset;
		int zpos = 0;
		float scale = 0.7;
		/*if(inst_pos == current_instance)
			alpha = ofMap(abs(visualizationOffset), 0, offset_per_frame, 1.0, 0.2,1);
		else if(inst_pos == current_instance - transitioning)
			alpha = ofMap(abs(visualizationOffset), 0, offset_per_frame, 0.2, 1,1);
		else
			alpha = 0.2;*/
		if(ypos > 0) {
			
			alpha = ofMap(abs(ypos), 0, offset_per_frame, 1, 0.2, 1);
		} else {
			alpha = ofMap(abs(ypos), offset_per_frame*0.1, offset_per_frame*0.9, 1, 0, 1);
		}
		//int offset = inst_pos*offset_per_frame;
		//ypos += offset;
		glPushMatrix();
		glTranslatef(xpos, ypos, zpos);
		//glColor3f(alpha,alpha,alpha);
		//cout << alpha << endl;
		//glScaled(scale, scale, scale);
		glRotated(-5, 0, 1, 0);
		//glLineWidth(1);
		drawMeshWire(meshes[draw_pos], alpha,inst_pos == current_instance);
		//drawMeshInverted(meshes[draw_pos], alpha,inst_pos == current_instance);
		//drawMeshTextured(meshes[draw_pos], alpha,inst_pos == current_instance);
		//drawMeshContour(meshes[inst_pos], alpha,inst_pos == current_instance);
		
		//glCallList(meshList[inst_pos]);
		glPopMatrix();
	}
	glColor3f(1, 1, 1);
	
	//glutSolidCube(100);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glDisable(GL_COLOR_MATERIAL);
	
	drawAnnotations();
	
	
	glPopMatrix();
}

void gesturalReliefApp::drawAnnotations() {
    ofVec3f diff = highestPoint - annotationPointH;
	diff *= 0.3;
	annotationPointH += diff;
	ofVec3f drawPointH = annotationPointH;
	drawPointH .y -=visualizationOffset;
	//drawPointH.y = annotationPointH.y + current_instance*offset_per_frame;
	
	diff = lowestPoint - annotationPointL;
	diff *= 0.3;
	annotationPointL += diff;
	ofVec3f drawPointL = annotationPointL;
	drawPointL .y -=visualizationOffset;
	//printf("%.2f %.2f %.2f\n", drawPointH.x, drawPointH.y, drawPointH.z);
	int frame_duration = FPS*2;
	if(annotationFrame > 0){
		//glPushMatrix();
		glRotated(-5, 0, 1, 0);
		//glScaled(1.35, 1.35, 1.35);
		ofVec3f point2H = drawPointH;
		point2H.x -= 60;
		point2H.y += 135 - point2H.z*0.25 - (annotationPointH.y*.7);
		//printf("%.2f %.2f %.2f\n", point2H.x, point2H.y, point2H.z);
		drawAnnotation(annotationPointH.y, drawPointH,point2H, (float)annotationFrame/frame_duration);
		
		//lowest point
		ofVec3f point2L = drawPointL;
		point2L.x -= 60;
		point2L.y += 140 - point2L.z*0.25 - (annotationPointL.y*.7);
		if (point2L.x-100 <= point2H.x && point2L.x >= point2H.x) {
			point2L.x = point2H.x+100;
		}
		if (point2L.x <= point2H.x &&  point2L.x >= point2H.x-100){
			point2L.x = point2H.x-100;
		}
		drawAnnotation(annotationPointL.y, drawPointL,point2L, (float)annotationFrame/frame_duration);
		
		if(annotationFrame < frame_duration)
			annotationFrame++;
	}
}

void gesturalReliefApp::drawRelief(frame relief, int width, int spacing, bool highlight) {
	int separation = width+spacing;
	for (int x = 0; x < RELIEF_SIZE_X ; x++) {
		for(int y = 0;y < RELIEF_SIZE_Y; y++) {
			if(mPinMask[x][y] == 1) {								
				glPushMatrix();
				glTranslatef(FRAME_WIDTH - x*separation,relief[x][y],-(FRAME_WIDTH-y*separation));
				if(highlight){
					if(stillLoading[x][y] == 1){
						glColor3f(0.0, 0.7, 0.0);
					} else {
						glColor3f(0.7, 0.7, 0.7);
					}
				}

				glBegin(GL_QUADS);
				glNormal3f(0, -1, 0);
				glVertex3f(0,0,0);
				glVertex3f(width,0,0);
				glVertex3f(width,0,width);
				glVertex3f(0,0,width);
				bool xokay = (x+1 <RELIEF_SIZE_X && mPinMask[x+1][y] == 1);
				bool yokay = (y+1 <RELIEF_SIZE_Y && mPinMask[x][y+1] == 1);
				if(xokay) {
					ofVec3f one(0,0,0);
					ofVec3f two(0,0,width);
					ofVec3f three(-spacing, relief[x+1][y]-relief[x][y], width);
					ofVec3f norm = (one-two).getCrossed(one-three).getNormalized(); 
					glNormal3f(norm.x,norm.y,norm.z);
					
					glVertex3f(one.x,one.y, one.z);
					glVertex3f(0, 0, width);
					glVertex3f(-spacing, relief[x+1][y]-relief[x][y], width);
					glVertex3f(-spacing, relief[x+1][y]-relief[x][y], 0);
				}
				
				if(yokay) {
					ofVec3f one(0,0,width);
					ofVec3f two(width,0,width);
					ofVec3f three(width, relief[x][y+1]-relief[x][y], width+spacing);
					ofVec3f norm = (one-two).getCrossed(one-three).getNormalized(); 
					glNormal3f(norm.x,norm.y,norm.z);
					
					glVertex3f(0, 0, width);
					glVertex3f(width, 0, width);
					glVertex3f(width, relief[x][y+1]-relief[x][y], width+spacing);
					glVertex3f(0, relief[x][y+1]-relief[x][y], width+spacing);
				}
				
				glEnd();
				if (xokay && yokay) {
					glBegin(GL_TRIANGLE_STRIP);
					ofVec3f one(0,0,width);
					ofVec3f two(-spacing, relief[x+1][y]-relief[x][y], width);
					ofVec3f three(0, relief[x][y+1]-relief[x][y], width+spacing);
					ofVec3f norm = (two-one).getCrossed(three-one).getNormalized();
					glVertex3f(0, 0, width);
					glVertex3f(-spacing, relief[x+1][y]-relief[x][y], width);
					glVertex3f(0, relief[x][y+1]-relief[x][y], width+spacing);
					if (mPinMask[x+1][y+1] == 1) {
						glVertex3f(-spacing, relief[x+1][y+1]-relief[x][y], width+spacing);
					}								
					glEnd();
				}						
				glPopMatrix();
			}
		}
	}
}

void gesturalReliefApp::updateMeshListSolid(int listIndex, mesh &relief) {
	glDeleteLists(meshList[listIndex], 1);
	GLuint index = glGenLists(1);
	glNewList(index, GL_COMPILE);
	//drawMeshSolid(relief);
	glEndList();
}

void gesturalReliefApp::drawMeshWire(mesh &relief, float color_scale, bool current) {
	vector<vector<float> > &vertices = relief.vertices;
	vector<vector<ofVec3f> > &normals = relief.normals;
	float width = (float)FRAME_WIDTH/vertices.size();
	float depth = (float)FRAME_WIDTH/(vertices.size());
	glPushMatrix();
	glDepthMask(GL_FALSE);
	glBegin(GL_LINES);
	//for (int i = 0; i < vertices.size()-1; i++) {	
	for (int i = 2; i < vertices.size()-1; i+=3) {
		for(int j = 0;j < vertices[i].size()-1; j++) {
			//horizontal
			float xoffset = (FRAME_WIDTH/2) - j*width;
			float zoffset = -((FRAME_WIDTH/2)-i*depth);
			ofVec3f norm = normals[j][i];
			glNormal3f(norm.x, norm.y, norm.z);
			ofVec3f color = mapHeightToColor(vertices[j][i]) * color_scale * meshMask[j][i];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset, vertices[j][i],-zoffset);
			
			norm = normals[j+1][i];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[j+1][i]) * color_scale * meshMask[j][i];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset+width, vertices[j+1][i],-zoffset);
			
			//vertical			
			xoffset = FRAME_WIDTH/2 - i*width;
			zoffset = -(FRAME_WIDTH/2-j*depth);
			
			norm = normals[i][j];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[i][j]) * color_scale * meshMask[i][j];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset, vertices[i][j],-zoffset);
			
			norm = normals[i][j+1];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[i][j+1]) * color_scale * meshMask[i][j];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset, vertices[i][j+1],-zoffset-depth);			
		}
	}
	glEnd();
	glBegin(GL_POINTS);
	for (int i = 2; i < vertices.size()-1; i+=3) {
		for(int j = 0;j < vertices[i].size()-1; j++) {
			//horizontal
			float xoffset = (FRAME_WIDTH/2) - j*width;
			float zoffset = -((FRAME_WIDTH/2)-i*depth);
			ofVec3f norm = normals[j][i];
			glNormal3f(norm.x, norm.y, norm.z);
			ofVec3f color = mapHeightToColor(vertices[j][i]) * color_scale * meshMask[j][i];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset, vertices[j][i],-zoffset);
			
			xoffset = FRAME_WIDTH/2 - i*width;
			zoffset = -(FRAME_WIDTH/2-j*depth);			
			norm = normals[i][j];
			glNormal3f(norm.x, norm.y, norm.z);	
			color = mapHeightToColor(vertices[i][j]) * color_scale * meshMask[i][j];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset, vertices[i][j],-zoffset);
		}
	}
	glEnd();
	glDepthMask(GL_TRUE);
	glPopMatrix();
	
}

void gesturalReliefApp::drawMeshContour(mesh &relief, float color_scale, bool current) {
	vector<vector<float> > &vertices = relief.vertices;
	vector<vector<ofVec3f> > &normals = relief.normals;
	float width = (float)FRAME_WIDTH/vertices.size();
	float depth = (float)FRAME_WIDTH/(vertices.size());
	//glDepthMask(GL_FALSE);
	if(current) {
		highestPoint.y = -130;
		lowestPoint.y = 130;
	}
	/*glBegin(GL_LINES);
	for (int i = 2; i < vertices.size()-1; i+=3) {
		for(int j = 0;j < vertices[i].size()-1; j++) {
			//horizontal
			float xoffset = (FRAME_WIDTH/2) - j*width;
			float zoffset = -((FRAME_WIDTH/2)-i*depth);
			if(meshMask[j][i] > 0.9 && current && vertices[j][i] > highestPoint.y ) {
				highestPoint.y = vertices[j][i];
				highestPoint.x = -xoffset;
				highestPoint.z = -zoffset;
			}
			if (meshMask[j][i] > 0.9 && current && vertices[j][i] < lowestPoint.y) {
				lowestPoint.y = vertices[j][i];
				lowestPoint.x = -xoffset;
				lowestPoint.z = -zoffset;
			}
			ofVec3f norm = normals[j][i];
			glNormal3f(norm.x, norm.y, norm.z);
			ofVec3f color = mapHeightToColor(vertices[j][i]) * color_scale * meshMask[j][i];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset, vertices[j][i],-zoffset);
			
			norm = normals[j+1][i];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[j][i]) * color_scale * meshMask[j][i];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset+width, vertices[j+1][i],-zoffset);
			
			//vertical			
			xoffset = FRAME_WIDTH/2 - i*width;
			zoffset = -(FRAME_WIDTH/2-j*depth);
			
			norm = normals[i][j];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[i][j]) * color_scale * meshMask[i][j];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset, vertices[i][j],-zoffset);
			
			norm = normals[i][j+1];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[i][j]) * color_scale * meshMask[i][j];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(-xoffset, vertices[i][j+1],-zoffset-depth);			
		}
	}
	glEnd();*/
	glBegin(GL_POINTS);
	for(int height = 0; height < 120; height+=20) {
		//vector<ofVec3f> points;
		for (int x = 0; x < vertices.size()-1; x++) {
			for(int y = 0;y < vertices[x].size()-1; y++) {				
				bool push = 0;
				ofVec3f point1,point2,point3;
				if((vertices[x][y] >= height && vertices[x+1][y] < height) || 
				   (vertices[x][y] < height && vertices[x+1][y] >= height)) {
					point1.set(-(FRAME_WIDTH/2) + x*width,vertices[x][y],(FRAME_WIDTH/2)-y*depth);
					point2.set(point1.x+width,vertices[x+1][y],point1.z);
					push = 1;
				}
				if((vertices[x][y] >= height && vertices[x][y+1] < height) || (vertices[x][y] < height && vertices[x][y+1] >= height)){
					point1.set(-(FRAME_WIDTH/2) + x*width,vertices[x][y],(FRAME_WIDTH/2)-y*depth);
					point2.set(point1.x,vertices[x][y+1],point1.z-depth);					
					push = 1;
				}
				if(push) {
					ofVec3f point3 = point1 + (point2-point1)*ofMap(height, point1.y, point2.y,0, 1, 1);
					ofVec3f norm = normals[x][y];
					glNormal3f(norm.x, norm.y, norm.z);
					ofVec3f color = mapHeightToColor(point3.y) * color_scale * meshMask[x][y];
					glColor3f(color.x,color.y,color.z);
					glVertex3f(point3.x,point3.y,point3.z);
					/*float mindist = -1;
					int minpos = 0;
					for (int i = 0; i < points.size(); i++) {
						float dist = pow(points[i].x-point3.x,2)+pow(points[i].y-point3.y,2)+pow(points[i].z-point3.z,2);				
						if(dist < mindist || mindist == -1) {
							mindist = dist;
							minpos = i;
						}
					}
					if(minpos == points.size()) {
						points.push_back(point3);
					} else {
						points.insert(points.begin()+minpos,point3);
					}*/
					
				}
			}
		}
		/*for(int i = 0; i < points.size(); i++) {
			ofVec3f norm = normals[x][y];
			glNormal3f(norm.x, norm.y, norm.z);
			ofVec3f color = mapHeightToColor(points[i].y) * color_scale * meshMask[x][y];
			glColor3f(color.x,color.y,color.z);
			glVertex3f(points[i].x,points[i].y,points[i].z);
		}*/
	}
	glEnd();
	//glDepthMask(GL_TRUE);
	
}

void gesturalReliefApp::updateMeshListWire(int listIndex, mesh &relief, bool current) {
	glDeleteLists(meshList[listIndex], 1);
	GLuint index = glGenLists(1);	
	glNewList(index, GL_COMPILE);
	float color_scale = 0.2;
	if (current) {
		color_scale = 1;
	}
	drawMeshWire(relief, color_scale);
	glEndList();
}



void gesturalReliefApp::updateCurrentMesh() {	
	current_mesh = generateMesh(reliefatov(mPinHeightToRelief));
	/*current_mesh.normals.clear();
	current_mesh.vertices.clear();
	vector<vector<float> > cols;	
	frame relief = reliefatov(mPinHeightToRelief);	
	for (int y = -1; y < RELIEF_SIZE_X+1 ; y++) {		
		vector<float> row;
		for(int x = -1;x < RELIEF_SIZE_Y+1; x++) {
			if(x == -1 || y == -1 || x == RELIEF_SIZE_X || y == RELIEF_SIZE_Y) { //border
				row.push_back(0);
			} else if(mPinMask[x][y] == 1) {								
				row.push_back(120-relief[x][y]); //also flipped
			} else {
				row.push_back(0);
			}
		}
		row = splinedouble(row);
		for (int i = 0; i < row.size(); i++) {
			if(i >= cols.size()){
				cols.push_back(*(new vector<float>));
			}
			cols[i].push_back(row[i]);
		}
	}
	for (int i = 0; i < cols.size(); i++) {
		current_mesh.vertices.push_back(splinedouble(cols[i]));
	}
	int size = current_mesh.vertices.size(); //convenience variable
	if(size > 0){
		float width = (float)FRAME_WIDTH/(size);
		float depth = width;
		
		vector<vector<ofVec3f> > faceNormals;
		for(int x = 0; x < size-1;x++) {
			vector<ofVec3f> col;
			for(int y = 0; y < size-1;y++) {
				float h1 = current_mesh.vertices[x][y];
				float h2 = current_mesh.vertices[x+1][y];
				float h3 = current_mesh.vertices[x+1][y+1];
				ofVec3f norm(depth*(h1-h2),width*depth,width*(h2-h1)); //calculated the normal by hand and simplfied for speed
				norm.normalize();
				col.push_back(-norm); //flip normals
			}
			faceNormals.push_back(col);
		}
		current_mesh.normals.clear();
		for(int x = 0; x < size;x++) {
			vector<ofVec3f> col;
			for(int y = 0; y < size;y++) {
				ofVec3f norm;
				if (x > 0) {
					if(y > 0) {
						norm += faceNormals[x-1][y-1];
					}
					if (y <= faceNormals.size()-1) {
						norm += faceNormals[x-1][y];
					}
				}
				if(x < faceNormals.size()) {
					if (y > 0) {
						norm += faceNormals[x][y-1];
					}
					if (y < faceNormals.size()) {
						norm += faceNormals[x][y];
					}					
				}
				norm.normalize();
				col.push_back(norm);
			}
			current_mesh.normals.push_back(col);
		}
	}*/
}

void gesturalReliefApp::updateAnnotations(){
	highestPoint.y = -130; //reset the annotations
	lowestPoint.y = 130;
	vector<vector<float> > & vertices = meshes[current_instance].vertices;
	float width = (float)FRAME_WIDTH/vertices.size();
	for (int y = 2; y < vertices.size()-1; y+=3) {
		for(int x = 0;x < vertices.size()-1; x++) {
			float xoffset = (FRAME_WIDTH/2) - x*width;
			float zoffset = -((FRAME_WIDTH/2)-y*width);
			float height = vertices[x][y];
			if(meshMask[x][y] > 0.9 && height > highestPoint.y ) {
				highestPoint.y = height;
				highestPoint.x = -xoffset;
				highestPoint.z = -zoffset;
			}
			if (meshMask[x][y] > 0.9 && height < lowestPoint.y) {
				lowestPoint.y = height;
				lowestPoint.x = -xoffset;
				lowestPoint.z = -zoffset;
			}
		}
	}
}

void gesturalReliefApp::updateMesh(int index) {
	bool current = (index == current_instance);	
	frame relief;
	if (current) {
		relief = instances[index].frames[current_frame];
	} else {
		relief = instances[index].frames[0];
	}
	meshes[index] = generateMesh(relief);
	/*mesh &current_mesh = meshes[index];
	current_mesh.normals.clear();
	current_mesh.vertices.clear();
	vector<vector<float> > cols;
	bool current = (index == current_instance);	
	frame relief;
	if (current) {
		relief = instances[index].frames[current_frame];
	} else {
		relief = instances[index].frames[0];
	}
	//frame relief = instances[current_instance].frames[0];
	for (int y = -1; y < RELIEF_SIZE_X+1 ; y++) {		
		vector<float> row;
		for(int x = -1;x < RELIEF_SIZE_Y+1; x++) {
			if(x == -1 || y == -1 || x == RELIEF_SIZE_X || y == RELIEF_SIZE_Y) { //border
				row.push_back(0);
			} else if(mPinMask[x][y] == 1) {								
				row.push_back(120-relief[x][y]); //also flipped
			} else {
				row.push_back(0);
			}
		}
		row = splinedouble(row);
		for (int i = 0; i < row.size(); i++) {
			if(i >= cols.size()){
				cols.push_back(*(new vector<float>));
			}
			cols[i].push_back(row[i]);
		}
	}
	for (int i = 0; i < cols.size(); i++) {
		current_mesh.vertices.push_back(splinedouble(cols[i]));
	}
	int size = current_mesh.vertices.size(); //convenience variable
	if(size > 0){
		float width = (float)FRAME_WIDTH/(size);
		float depth = width;
		
		vector<vector<ofVec3f> > faceNormals;
		for(int x = 0; x < size-1;x++) {
			vector<ofVec3f> col;
			for(int y = 0; y < size-1;y++) {
				float h1 = current_mesh.vertices[x][y];
				float h2 = current_mesh.vertices[x+1][y];
				float h3 = current_mesh.vertices[x+1][y+1];
				ofVec3f norm(depth*(h1-h2),width*depth,width*(h2-h1)); //calculated the normal by hand and simplfied for speed
				norm.normalize();
				col.push_back(-norm); //flip normals
			}
			faceNormals.push_back(col);
		}
		current_mesh.normals.clear();
		for(int x = 0; x < size;x++) {
			vector<ofVec3f> col;
			for(int y = 0; y < size;y++) {
				ofVec3f norm;
				if (x > 0) {
					if(y > 0) {
						norm += faceNormals[x-1][y-1];
					}
					if (y <= faceNormals.size()-1) {
						norm += faceNormals[x-1][y];
					}
				}
				if(x < faceNormals.size()) {
					if (y > 0) {
						norm += faceNormals[x][y-1];
					}
					if (y < faceNormals.size()) {
						norm += faceNormals[x][y];
					}					
				}
				norm.normalize();
				col.push_back(norm);
			}
			current_mesh.normals.push_back(col);
		}
	}*/
	//updateMeshListWire(index,meshes[index],current);
}

gesturalReliefApp::mesh gesturalReliefApp::generateMesh(frame relief){
	mesh m;
	vector<vector<float> > cols;
	for (int y = -1; y < RELIEF_SIZE_X+1 ; y++) {		
		vector<float> row;
		for(int x = -1;x < RELIEF_SIZE_Y+1; x++) {
			if(x == -1 || y == -1 || x == RELIEF_SIZE_X || y == RELIEF_SIZE_Y) { //border
				row.push_back(0);
			} else if(mPinMask[x][y] == 1) {								
				row.push_back(120-relief[x][y]); //also flipped
			} else {
				row.push_back(0);
			}
		}
		row = splinedouble(row);
		for (int i = 0; i < row.size(); i++) {
			if(i >= cols.size()){
				cols.push_back(*(new vector<float>));
			}
			cols[i].push_back(row[i]);
		}
	}
	for (int i = 0; i < cols.size(); i++) {
		m.vertices.push_back(splinedouble(cols[i]));
	}
	int size = m.vertices.size(); //convenience variable
	if(size > 0){
		float width = (float)FRAME_WIDTH/(size);
		float depth = width;
		
		vector<vector<ofVec3f> > faceNormals;
		for(int x = 0; x < size-1;x++) {
			vector<ofVec3f> col;
			for(int y = 0; y < size-1;y++) {
				float h1 = m.vertices[x][y];
				float h2 = m.vertices[x+1][y];
				float h3 = m.vertices[x+1][y+1];
				ofVec3f norm(depth*(h1-h2),width*depth,width*(h2-h1)); //calculated the normal by hand and simplfied for speed
				norm.normalize();
				col.push_back(-norm); //flip normals
			}
			faceNormals.push_back(col);
		}
		m.normals.clear();
		for(int x = 0; x < size;x++) {
			vector<ofVec3f> col;
			for(int y = 0; y < size;y++) {
				ofVec3f norm;
				if (x > 0) {
					if(y > 0) {
						norm += faceNormals[x-1][y-1];
					}
					if (y <= faceNormals.size()-1) {
						norm += faceNormals[x-1][y];
					}
				}
				if(x < faceNormals.size()) {
					if (y > 0) {
						norm += faceNormals[x][y-1];
					}
					if (y < faceNormals.size()) {
						norm += faceNormals[x][y];
					}					
				}
				norm.normalize();
				col.push_back(norm);
			}
			m.normals.push_back(col);
		}
	}
	return m;
}

void gesturalReliefApp::drawMeshSolid(mesh &relief,float color_scale, bool current) {
	vector<vector<float> > &vertices = relief.vertices;
	vector<vector<ofVec3f> > &normals = relief.normals;
	float width = (float)FRAME_WIDTH/vertices.size();
	float depth = (float)FRAME_WIDTH/(vertices.size());
	ofVec3f norm, color;
	glPushMatrix();
	//glEnable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D, texture);
	glBegin(GL_QUADS);
	for (int x = 0; x < vertices.size()-1; x++) {
		for(int y = 0;y < vertices[x].size()-1; y++) {
			float xoffset = -(FRAME_WIDTH/2) + x*width;
			float zoffset = (FRAME_WIDTH/2)-y*depth;			
			
			norm = normals[x][y];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[x][y]) * color_scale * meshMask[x][y];
			glColor3f(color.x,color.y,color.z);
			//glTexCoord2d(0, 0);
			glVertex3f(xoffset, vertices[x][y],zoffset);			
			
			norm = normals[x+1][y];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[x+1][y]) * color_scale * meshMask[x+1][y];
			glColor3f(color.x,color.y,color.z);
			//glTexCoord2d(1, 0);
			glVertex3f(xoffset+width, vertices[x+1][y],zoffset);
			
			norm = normals[x+1][y+1];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[x+1][y+1]) * color_scale * meshMask[x+1][y+1];
			glColor3f(color.x,color.y,color.z);
			//glTexCoord2d(0, 1);
			glVertex3f(xoffset+width, vertices[x+1][y+1],zoffset-width);				
			
			norm = normals[x][y+1];
			glNormal3f(norm.x, norm.y, norm.z);
			color = mapHeightToColor(vertices[x][y+1]) * color_scale * meshMask[x][y+1];
			glColor3f(color.x,color.y,color.z);
			//glTexCoord2d(1, 1);
			glVertex3f(xoffset, vertices[x][y+1],zoffset-width);
		}
	}
	glColor3f(1, 1, 1);
	glEnd();
	glPopMatrix();
}

void gesturalReliefApp::drawMeshTextured(mesh &relief,float color_scale, bool current) {
	vector<vector<float> > &vertices = relief.vertices;
	vector<vector<ofVec3f> > &normals = relief.normals;
	float width = (float)FRAME_WIDTH/vertices.size();
	float depth = (float)FRAME_WIDTH/(vertices.size());
	ofVec3f norm, color;
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture);
	glBegin(GL_QUADS);
	float minx = -(FRAME_WIDTH/2);
	float minz = (FRAME_WIDTH/2);
	float maxx = (FRAME_WIDTH/2);
	float maxz = -(FRAME_WIDTH/2);
	ofVec3f white(1,1,1);
	for (int x = 0; x < vertices.size()-1; x++) {
		for(int y = 0;y < vertices[x].size()-1; y++) {
			float xoffset = -(FRAME_WIDTH/2) + x*width;
			float zoffset = (FRAME_WIDTH/2)-y*depth;			
			
			norm = normals[x][y];
			glNormal3f(norm.x, norm.y, norm.z);
			color = white * color_scale * meshMask[x][y];
			glColor3f(color.x, color.y, color.z);
			glTexCoord2d(ofMap(xoffset,minx,maxx,1,0), ofMap(zoffset,minz,maxz,0,1));
			glVertex3f(xoffset, vertices[x][y],zoffset);			
			
			norm = normals[x+1][y];
			glNormal3f(norm.x, norm.y, norm.z);
			color = white * color_scale * meshMask[x+1][y];
			glColor3f(color.x, color.y, color.z);
			glTexCoord2d(ofMap(xoffset+width,minx,maxx,1,0), ofMap(zoffset,minz,maxz,0,1));
			glVertex3f(xoffset+width, vertices[x+1][y],zoffset);
			
			norm = normals[x+1][y+1];
			glNormal3f(norm.x, norm.y, norm.z);
			color = white * color_scale * meshMask[x+1][y+1];
			glColor3f(color.x, color.y, color.z);
			glTexCoord2d(ofMap(xoffset+width,minx,maxx,1,0), ofMap(zoffset-width,minz,maxz,0,1));
			glVertex3f(xoffset+width, vertices[x+1][y+1],zoffset-width);				
			
			norm = normals[x][y+1];
			glNormal3f(norm.x, norm.y, norm.z);
			color = white * color_scale * meshMask[x][y+1];
			glColor3f(color.x, color.y, color.z);
			glTexCoord2d(ofMap(xoffset,minx,maxx,1,0), ofMap(zoffset-width,minz,maxz,0,1));
			glVertex3f(xoffset, vertices[x][y+1],zoffset-width);
		}
	}
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

/*void gesturalReliefApp::drawMeshInverted(mesh &relief,float color_scale, bool current) {
	vector<vector<float> > &heights = relief.vertices;
	vector<vector<ofVec3f> > &normals = relief.normals;
    vector<ofVec3f> vertices(heights.size());
	int vert_mask[heights.size()][heights.size()];
	bool checked_mask[heights.size()][heights.size()];
	memset(checked_mask, 0, heights.size()*heights.size());
	//vector<vector<ofVec3f> > vertices;
	//vector<vector<ofVec3f> > vert_mask;
	
	float width = (float)FRAME_WIDTH/heights.size();
	float depth = (float)FRAME_WIDTH/(heights.size());
	
	
	for (int x = 0; x < heights.size()-1; x++) {		
		for(int y = 0;y < heights[x].size()-1; y++) {
			float xoffset = -(FRAME_WIDTH/2) + x*width;
			float zoffset = (FRAME_WIDTH/2)-y*depth;
			float height = heights[x][y];
			ofVec3f newpoint;
			int num = 0;
			ofVec3f point1(xoffset,height,zoffset);
			ofVec3f point2(xoffset+width,heights[x+1][y],zoffset);
			if((point1.y >= inverted_clip_height && point2.y <= inverted_clip_height) ||
			   (point2.y >= inverted_clip_height && point1.y <= inverted_clip_height)) {
				newpoint = point1+(point2-point1)*ofMap(inverted_clip_height, point1.y, point2.y, 0, 1, 1);
				num++;
			}
			point2.set(xoffset,heights[x][y+1],zoffset-depth);
			if((point1.y >= inverted_clip_height && point2.y <= inverted_clip_height) ||
			   (point2.y >= inverted_clip_height && point1.y <= inverted_clip_height)) {
				newpoint = point1+(point2-point1)*ofMap(inverted_clip_height, point1.y, point2.y, 0, 1, 1);
				num++;
			}
			if(num == 0) {
				newpoint = point1;
				if (newpoint.y <= inverted_clip_height) {
					vert_mask[x][y] = 1;
				} else {
					vert_mask[x][y] = 0;
				}
			}		
			vertices[x][y] = newpoint;			
		}
	}
	
	/*for (int x = 0; x < heights.size()-1; x++) {		
		for(int y = 0;y < heights[x].size()-1; y++) {
			float xoffset = -(FRAME_WIDTH/2) + x*width;
			float zoffset = (FRAME_WIDTH/2)-y*depth;
			float height = heights[x][y];
			if(height > inverted_clip_height) {
				vert_mask[x][y] = 0;
			} else {
				vert_mask[x][y] = 1;
			}
			ofVec3f newpoint;
			int num = 0;
			ofVec3f point1(xoffset,height,zoffset);
			ofVec3f point2(xoffset+width,heights[x+1][y],zoffset);
			if((point1.y >= inverted_clip_height && point2.y <= inverted_clip_height) ||
			   (point2.y >= inverted_clip_height && point1.y <= inverted_clip_height)) {
				newpoint += point1+(point2-point1)*ofMap(inverted_clip_height, point1.y, point2.y, 0, 1, 1);
				num++;
			}
			point2.set(xoffset,heights[x][y+1],zoffset-depth);
			if((point1.y >= inverted_clip_height && point2.y <= inverted_clip_height) ||
			   (point2.y >= inverted_clip_height && point1.y <= inverted_clip_height)) {
				newpoint += point1+(point2-point1)*ofMap(inverted_clip_height, point1.y, point2.y, 0, 1, 1);
				num++;
			}
			if(num == 0)
				newpoint = point1;
				
			vertices[x][y] = newpoint/num;			

				
		}
	}
	
	
	ofVec3f norm, color;
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture);
	//clipped bottom
	glBegin(GL_QUADS);
	float minx = -(FRAME_WIDTH/2);
	float minz = (FRAME_WIDTH/2);
	float maxx = (FRAME_WIDTH/2);
	float maxz = -(FRAME_WIDTH/2);
	ofVec3f white(1,1,1);
	for (int x = 0; x < heights.size()-1; x++) {
		for(int y = 0;y < heights[x].size()-1; y++) {
			if(vert_mask[x][y] == 1 &&vert_mask[x+1][y] == 1 &&vert_mask[x][y+1] == 1 &&vert_mask[x+1][y+1] == 1){
			float xoffset = -(FRAME_WIDTH/2) + x*width;
			float zoffset = (FRAME_WIDTH/2)-y*depth;			
			
			norm = normals[x][y];
			glNormal3f(norm.x, norm.y, norm.z);
			color = white * color_scale * meshMask[x][y]*vert_mask[x][y];
			glColor3f(color.x, color.y, color.z);
			glTexCoord2d(ofMap(xoffset,minx,maxx,1,0), ofMap(zoffset,minz,maxz,0,1));
			glVertex3f(vertices[x][y].x, vertices[x][y].y,vertices[x][y].z);			
			
			norm = normals[x+1][y];
			glNormal3f(norm.x, norm.y, norm.z);
			color = white * color_scale * meshMask[x+1][y]*vert_mask[x+1][y];
			glColor3f(color.x, color.y, color.z);
			glTexCoord2d(ofMap(xoffset+width,minx,maxx,1,0), ofMap(zoffset,minz,maxz,0,1));
			glVertex3f(vertices[x+1][y].x, vertices[x+1][y].y,vertices[x+1][y].z);
			
			norm = normals[x+1][y+1];
			glNormal3f(norm.x, norm.y, norm.z);
			color = white * color_scale * meshMask[x+1][y+1]*vert_mask[x+1][y+1];
			glColor3f(color.x, color.y, color.z);
			glTexCoord2d(ofMap(xoffset+width,minx,maxx,1,0), ofMap(zoffset-width,minz,maxz,0,1));
			glVertex3f(vertices[x+1][y+1].x, vertices[x+1][y+1].y,vertices[x+1][y+1].z);				
			
			norm = normals[x][y+1];
			glNormal3f(norm.x, norm.y, norm.z);
			color = white * color_scale * meshMask[x][y+1]*vert_mask[x][y+1];
			glColor3f(color.x, color.y, color.z);
			glTexCoord2d(ofMap(xoffset,minx,maxx,1,0), ofMap(zoffset-width,minz,maxz,0,1));
			glVertex3f(vertices[x][y+1].x, vertices[x][y+1].y,vertices[x][y+1].z);
			}
			
		}
	}
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}*/

/*
void gesturalReliefApp::drawMeshSolid(mesh &relief,float color_scale, bool current) {
	
	vector<vector<float> > &vertices = relief.vertices;
	vector<vector<ofVec3f> > &normals = relief.normals;
	float width = (float)FRAME_WIDTH/vertices.size();
	float depth = (float)FRAME_WIDTH/(vertices.size());	
	glBegin(GL_QUADS);
	for (int x = 0; x < vertices.size()-1; x++) {
		for(int y = 0;y < vertices[x].size()-1; y++) {
			float xoffset = -(FRAME_WIDTH/2) + x*width;
			float zoffset = -((FRAME_WIDTH/2) + y*depth);
			
			ofVec3f norm = normals[x][y];
			glNormal3f(norm.x, norm.y, norm.z);				
			glVertex3f(xoffset, vertices[x][y],zoffset);
			
			norm = normals[x+1][y];
			glNormal3f(norm.x, norm.y, norm.z);
			glVertex3f(xoffset-width, vertices[x+1][y],zoffset);
			
			norm = normals[x+1][y+1];
			glNormal3f(norm.x, norm.y, norm.z);
			glVertex3f(xoffset-width, vertices[x+1][y+1],zoffset+depth);
			
			norm = normals[x][y+1];
			glNormal3f(norm.x, norm.y, norm.z);
			glVertex3f(xoffset, vertices[x][y+1],zoffset+depth);
			
		}
	}
	glEnd();
}
*/

void gesturalReliefApp::processLoading() {
	//loading constants	
	int max_speed = 4;
	//int min_speed = 4;
	int diff_threshold = 13;
	int max_frames = FPS*2; // maximum loading frames incase of a pin-misfire
	int adjust_duration = FPS*0.5;
	
	if(adjust_frame == 0) {
		if(animating != 0) {
			max_speed = 100;
		} else if (visualizationMode == 1) {
			max_speed = 7;
		}
		//mIOManager->getPinHeightFromRelief(mPinHeightFromRelief); //update the from pin heights
		frames_loading++;
		bool continue_loading_flag = 0; //flag to see if any pins are still loading
		frame saved = instances[current_instance].frames[current_frame]; //the goal state
		for (int x = 0; x < RELIEF_SIZE_X; x++) {
			for (int y = 0; y < RELIEF_SIZE_Y; y++) {
				if(mPinMask[x][y]){ //only consider pins that exist							
					stillLoading[x][y] = 0; //loading flag for individual pins
					//int fdiff = (int)saved[x][y] - (int)mPinHeightFromRelief[x][y];
					int tdiff = (int)saved[x][y] - (int)mPinHeightToRelief[x][y];
					if(abs(tdiff) >= diff_threshold){
                    //if(abs(fdiff) >= diff_threshold){                            
                        stillLoading[x][y] = 1;
						continue_loading_flag = 1;
					}
					if (tdiff > 0) {
						mPinHeightToRelief[x][y] += min(max_speed,tdiff);
					} else if(tdiff < 0){
						mPinHeightToRelief[x][y] += max(-max_speed,tdiff);
					}
					
				}
			}
		}
		
		
		if(!continue_loading_flag || frames_loading > max_frames) { //conditions for loading finished
			adjust_frame = 1;
			for (int x = 0; x < RELIEF_SIZE_X; x++) {
				for (int y = 0; y < RELIEF_SIZE_Y; y++) {
					stillLoading[x][y] = 0;
				}
			}
		}
	} else {
		adjust_frame++;
		if (adjust_frame >= adjust_duration || (visualizationMode == 0 && frames_idle < IDLE_THRESHOLD)) { //conditions to end adjustment
			adjust_frame = 0;
			loading = 0;
		}
	}
}

void gesturalReliefApp::drawAnnotation(float value,ofVec3f point,ofVec3f point2, float progress) {
	progress = pow(progress,2);
	float time1 = 0.2;
	float time2 = 0.3;
	ofVec3f point3(point2.x - 80, point2.y,point2.z);
	if(progress > 0) {
		ofVec3f dist = point2-point;
		dist *= ofMap(progress, 0, time1, 0, 1,true);
		ofVec3f endpoint = point+dist;
		glBegin(GL_LINES);
		glColor4f(0.3, 0.3, 0.3,0.5);
		glVertex3f(point.x, point.y, point.z);
		glColor3f(time1, time1, time1);
		glVertex3f(endpoint.x, endpoint.y, endpoint.z);
		
		glEnd();
	}
	if(progress > time1) {
		ofVec3f dist = point3-point2;
		dist *= ofMap(progress, time1, time2, 0, 1,true);
		ofVec3f endpoint = point2+dist;
		glBegin(GL_LINES);
		glColor3f(time1, time1, time1);
		glVertex3f(point2.x,point2.y,point2.z);
		glColor3f(time2, time2, time2);
		glVertex3f(endpoint.x, endpoint.y, endpoint.z);
		//printf("end %.2f %.2f %.2f\n", endpoint.x, endpoint.y, endpoint.z);
		glEnd();
	}
	if(progress > time2) {
		double alpha = ofMap(progress, time2, 1, 0, 1,true);
		char buffer[1024];
		sprintf(buffer,"%.2fm",alpha*value);
		int width = font.stringWidth(buffer);
		int height = font.stringHeight(buffer);
		ofPushMatrix();		
		glColor4f(1, 1, 1,alpha);
		ofTranslate(point3.x+width,point3.y+10,point3.z);
		ofRotate(180, 0, 0, 1);
		//ofDrawBitmapString(buffer, -200,-700);
        font.drawString(buffer,0,0);
		glColor3f(1, 1, 1);
		ofPopMatrix();
	}
	glColor3f(1, 1, 1);
}

void gesturalReliefApp::startLoading() {
	loading = 1;
	frames_loading = 0;
	stable_frames = 0;
	adjust_frame = 0;
}

void gesturalReliefApp::resetAnimation() {
	animating = 0;
	current_frame = 0;
	startLoading();
}

//ManyMouse

void gesturalReliefApp::mouseMoved(int device, int axis, int value) {
	value = -value; //Invert the motion because it feels more intuitive to me that way.
	
	if(abs(value) > 9) {
		//unidle
		frames_idle = 0;
		if(visualizationMode == 1) { //if the wheel is moved break into seeking mode
			animating = 0;
			recording = 0;
		}
	}
	
	float wheelgain = 0.25;
	if (visualizationMode == 0) {
		wheelgain = 0.04;
	}
	value *= wheelgain;
	
	if(animating == 0 && recording == 0) { //ignore wheel movement while animating or recording
		//if(device == wheeldevice && axis == 1) {			
			visualizationOffset += value;
		//}
	}
}


float gesturalReliefApp::spline(vector<float> x,vector<float>y,float desired){
	for(int pos = 1; pos < x.size()-2; pos++){
		//double desired = 0;
		if(desired >= x[pos] && desired <= x[pos+1]){
			float h00,h01,h11,h10,p0,m0,p1,m1;
			float tension = 0;
			float t = ofMap(desired, x[pos], x[pos+1], 0, 1);
			//float dist = y[pos+1] - y[pos];
			h00 = 2*pow(t,3) - 3*pow(t,2) + 1;
			h10 = pow(t,3) - 2*pow(t,2) + t;
			h01 = -2*pow(t,3) + 3*pow(t,2);
			h11 = pow(t, 3) - pow(t, 2);
			p0 = y[pos];
			p1 = y[pos+1];
			m0 = (1-tension)*(y[pos+1]-y[pos-1])/(x[pos+1]-x[pos-1]);
			m1 = (1-tension)*(y[pos+2]-y[pos])/(x[pos+2]-x[pos]);
			//printf("%f.2 %f.2 %f.2 %f.2\n",p0,m0,p1,m1);
			float val = h00*p0 + h10*m0 + h01*p1 + h11*m1;
			return val;
		}
	}
}

template <class T>
vector<T> gesturalReliefApp::splinedouble(vector<T> y){
	vector<T> output;
	y.insert(y.begin(),y[0]-y[1]);
	y.push_back(y[y.size()-1]-y[y.size()-2]);
	int steps = 3;
	for(int pos = 1; pos < y.size()-2; pos++){
		for(int i = 0; i < steps; i++) { 
			float h00,h01,h11,h10,p0,m0,p1,m1;
			float tension = 0.5;
			float t = (float)i/steps;
			//float t = ofMap(desired, x[pos], x[pos+1], 0, 1);
			//float dist = y[pos+1] - y[pos];
			h00 = 2*pow(t,3) - 3*pow(t,2) + 1;
			h10 = pow(t,3) - 2*pow(t,2) + t;
			h01 = -2*pow(t,3) + 3*pow(t,2);
			h11 = pow(t, 3) - pow(t, 2);
			p0 = y[pos];
			p1 = y[pos+1];
			//m0 = (1-tension)*(y[pos+1]-y[pos-1])/(x[pos+1]-x[pos-1]);
			//m1 = (1-tension)*(y[pos+2]-y[pos])/(x[pos+2]-x[pos]);
			m0 = (1-tension)*(y[pos+1]-y[pos-1])/(2);
			m1 = (1-tension)*(y[pos+2]-y[pos])/(2);
			//printf("%.2f %.2f %.2f %.2f\n",p0,m0,p1,m1);
			T val = h00*p0 + h10*m0 + h01*p1 + h11*m1;
			//output.push_back(p0);
			output.push_back(val);
		}
	}
	output.push_back(y[(int)y.size()-2]);
	return output;
}

void gesturalReliefApp::generateMeshMask() {	
	meshMask.clear();
	vector<vector<float> > cols;
	frame relief = reliefatov(mPinMask);
	//frame relief = instances[current_instance].frames[0];
	for (int y = -1; y < RELIEF_SIZE_X+1 ; y++) {		
		vector<float> row;
		for(int x = -1;x < RELIEF_SIZE_Y+1; x++) {
			if(x == -1 || y == -1 || x == RELIEF_SIZE_X || y == RELIEF_SIZE_Y) { //border
				row.push_back(0);
			} else {
				row.push_back(relief[x][y]);
			}
		}
		row = splinedouble(row);
		for (int i = 0; i < row.size(); i++) {
			if(i >= cols.size()){
				cols.push_back(*(new vector<float>));
			}
			cols[i].push_back(row[i]);
		}
	}
	for (int i = 0; i < cols.size(); i++) {
		meshMask.push_back(splinedouble(cols[i]));
	}
	for(int x = 0; x < meshMask.size(); x++) {
		for(int y = 0; y < meshMask[x].size(); y++) {
			meshMask[x][y] = (meshMask[x][y]-0.5)*2;
			if (meshMask[x][y] > 1) {
				meshMask[x][y] = 1;
			} else if(meshMask[x][y] < 0) {
				meshMask[x][y] = 0;
			}
		}
	}
}

void gesturalReliefApp::generateLineHeights() {
	int frame_resolution = 10;
	cout << lineHeights.size() << endl;
	lineHeights.clear();
	for(int frame_pos = 0; frame_pos <= instances[current_instance].frames.size()-1; frame_pos += frame_resolution){
		mesh current = generateMesh(instances[current_instance].frames[frame_pos]);		
		vector<vector<float> > &vertices = current.vertices;
		vector<vector<ofVec3f> > &normals = current.normals;		
		int line_heights_position = frame_pos/frame_resolution;
		int center_index = vertices.size()/2;
		vector<float> line;
		for(int x = 0; x < vertices.size();x++) {
			line.push_back(vertices[x][center_index]);
		}
		lineHeights.push_back(line);
	}
	cout << lineHeights.size() << endl;;
}

void gesturalReliefApp::processMessages() {
    if(!connected) {
        ofxOscMessage m;
        m.setAddress("/relief/connect");
        m.addIntArg(LISTEN_PORT);
		sender.sendMessage(m);
        lastPing = ofGetElapsedTimef();
    } else {
        //if (connected && ofGetElapsedTimef() > lastPing + 2.f) {
            ofxOscMessage m;
            m.setAddress("/relief/ping");
            sender.sendMessage(m);
        //}
        
        ofxOscMessage m1;
        m1.setAddress("/relief/set");
        //send the square
        for (int x = 0; x < RELIEF_SIZE_X; x++) { 
            for (int y = 0; y < RELIEF_SIZE_Y; y++) {
               m1.addIntArg(ofMap(mPinHeightToRelief[x][y],RELIEF_FLOOR,RELIEF_CEIL,0,100,1));
            }
        }
        sender.sendMessage(m1);
    }
    
    while (receiver.hasWaitingMessages()) {
        ofxOscMessage m;
        receiver.getNextMessage(&m);        
        // check for mouse moved message
        if(m.getAddress() == "/relief/connect/reply"){           
            connected = true;
            printf("CONNECTED\n");
        }
        if(m.getAddress() == "/relief/update") {
            //unsigned char relief[RELIEF_SIZE_X][RELIEF_SIZE_Y];
            for (int x = 0; x < RELIEF_SIZE_X; x++) { 
                for (int y = 0; y < RELIEF_SIZE_Y; y++) {
                    unsigned char val = ofMap(m.getArgAsInt32(y+x*RELIEF_SIZE_Y),0,100,RELIEF_FLOOR,RELIEF_CEIL,1);
                    mPinHeightFromRelief[x][y] = val;
                    //relief[x][y] = val;
                }
            }
            //instances[current_instance].frames[current_frame] = reliefatov(relief);
        }
    }
}

void gesturalReliefApp::mousePressed(int device, int button) { //set the wheel mouse to be the one that was not clicked
	wheeldevice = 1-device;
}

//--------------------------------------------------------------
void gesturalReliefApp::keyReleased(int key){
}	

//--------------------------------------------------------------
void gesturalReliefApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void gesturalReliefApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void gesturalReliefApp::mousePressed(int x, int y, int button){
	//printf("Mouse Pressed: %d,%d\n",x,y);
}

//--------------------------------------------------------------
void gesturalReliefApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void gesturalReliefApp::windowResized(int w, int h){

}

