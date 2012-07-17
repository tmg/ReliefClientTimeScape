#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxManyMouse.h"
#include "ofxOsc.h"
#include "ReliefIOManager.h"
#include "handDetector.h"
#include "constants.h"
#include "lodepng.h"

class gesturalReliefApp : public ofBaseApp , public ofxManyMouse{

	public:
	
		typedef vector<vector< unsigned char> > frame;
		struct instance {
			vector<ofRectangle> cursorRect;
			vector<bool> manipulationOn;
			vector<frame> frames;
		};		
		
		struct mesh{
			vector<vector<ofVec3f> > normals;
			vector<vector<float> > vertices;
		};
	
		struct camera{
			float x;
			float y;
			float z;
			float angle;
		};
	
		void setup();
		void update();
		void draw();
		
		void resetProjectionPixels();
		void mapToProjection(ofPoint realPt, ofPoint * projectionPt);
		void mapToProjection(ofRectangle realRect, ofRectangle * projectionRect);
		void mapToRelief(ofPoint realPt, ofPoint * reliefPt);
		void mapToRelief(ofRectangle realRect, ofRectangle * reliefRect);
		void mapAngleToRelief(float realAngle, float * reliefAngle);
		void visualizeOnRelief();
		void visualizeSelectionFeedback();
		
		void transform();
	
		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
	
		//ManyMouse	
		void mouseMoved(int device,int axis, int value);
		void mousePressed(int device, int button);
	
		//pretty stuff
		void roundedRect(float x, float y, float w, float h, float r);
		void quadraticBezierVertex(float cpx, float cpy, float x, float y, float prevX, float prevY);
		
		// Kinect
		ofxKinect kinect;
		
		ofxCvGrayscaleImage grayImage;
		ofImage calibratedColorImage;
		
		ofxCvGrayscaleImage grayThresh;
		ofxCvGrayscaleImage grayThreshFar;
		
		bool bThreshWithOpenCV;
		
		int nearThreshold;
		int	farThreshold;
	
		// state machine variables
		int state;
		bool selectionOn;
		bool manipulationOn;
	
		// Hand feature detection
		handDetector myHandDetector;
		
		// Cursor tracking
		ofRectangle cursorRect;
		float cursorAngle;
		ofRectangle reliefCursorRect;
		float reliefCursorAngle;
	
		// Height manipulation bits and pieces
		ofRectangle lockedSelectionRect;
		float lockedSelectionHeight;
		float lockedSelectionAngle;
		unsigned char lockedPinHeight[RELIEF_SIZE_X][RELIEF_SIZE_Y];
	
		// Transform things
		ofxCvGrayscaleImage transformImage;
	
		// Used for kinect target box (used to select kinect input to send to relief)
		ofRectangle roiRect;
	
		// Projection bits and pieces
		int previousSelectionColor;
		ofRectangle projectionRect;
		ofColor projectionPixelMatrix[RELIEF_SIZE_X][RELIEF_SIZE_Y];
		ofRectangle parallaxRect;
		
		// Relief
		ReliefIOManager * mIOManager;
		unsigned char mPinHeightFromRelief [RELIEF_SIZE_X][RELIEF_SIZE_Y];
		unsigned char mPinHeightToRelief [RELIEF_SIZE_X][RELIEF_SIZE_Y];
		unsigned char mPinMask[RELIEF_SIZE_X][RELIEF_SIZE_Y];
		unsigned char previousHeightToRelief [RELIEF_SIZE_X][RELIEF_SIZE_Y];
		unsigned char previousHeightFromRelief [RELIEF_SIZE_X][RELIEF_SIZE_Y];
		vector<vector<float> > meshMask;
		void generateMeshMask();
	
		vector<instance> instances;
		
		int current_frame;
		int current_instance;
		
		void updateFromReliefHeight();
		void loadRelief();
		void reliefvtoa(frame, unsigned char arr[RELIEF_SIZE_X][RELIEF_SIZE_Y]);
		frame reliefatov(unsigned char arr[RELIEF_SIZE_X][RELIEF_SIZE_Y]);
		
		void pushFrame(int inst);
		void pushInstance();
		void pushMesh();
		void changeFrame(int dist);
		void changeInstance(int dist);
		void animate(int dir);
		void resetInstance(int inst);
		void resetInstances();
		void startLoading();
		void processLoading();
		void resetAnimation();
		void pushChanges();
		
		void drawInstances();
		void drawFrames();
        void drawAnnotations();
		void drawRelief(frame relief,int width, int spacing, bool highlight = 0);
		void drawMeshSolid(mesh &relief,float color_scale, bool current = 0);
		void drawMeshTextured(mesh &relief,float color_scale, bool current = 0);
		void drawMeshWire(mesh &relief, float color_scale, bool current = 0);
		void drawMeshContour(mesh &relief, float color_scale, bool current = 0);
		//void drawMeshInverted(mesh &relief, float color_scale, bool current = 0);
		void updateMeshListSolid(int listInxex,mesh &relief);
		void updateMeshListWire(int listInxex,mesh &relief, bool current = 0);
		void updateMesh(int index);
		void updateCurrentMesh();
		mesh generateMesh(frame relief);
		float spline(vector<float> x, vector<float> y, float desired);
	
		ofVec3f mapHeightToColor(float height);
		ofVec3f mapReliefTo3d(ofVec3f point);
		ofVec3f map3dToRelief(ofVec3f point);
	
		template <class T>
		vector<T> splinedouble(vector<T> y);
		vector<mesh> meshes;
	
		bool loading;
		int adjust_frame;
		bool editing;
		int transitioning;
		int animating;
		int recording;
		int frames_loading;
	
		int wheelseek;
		int wheeldevice;
	
		float visualizationOffset;
		int frames_idle;
	
		camera instanceCamera;
		camera frameCamera;
	
		int visualizationMode; //0: Instances 1:Frames
	
		unsigned char stillLoading[RELIEF_SIZE_X][RELIEF_SIZE_Y];
	
		void buildshape();
	
		ofVec3f highestPoint;
		ofVec3f lowestPoint;
		ofVec3f annotationPointH;
		ofVec3f annotationPointL;
		void drawAnnotation(float height,ofVec3f point,ofVec3f point2, float progress);
		float annotationFrame;
	
		vector<GLuint> meshList;
		vector<vector<float> > lineHeights;
		void generateLineHeights();
		void updateAnnotations();
	
		ofTrueTypeFont font;
		int stable_frames;
		int changingFrame;
	
		mesh current_mesh;
		GLuint texture;
	
		float inverted_clip_height;
    
        //Network
        ofxOscSender sender;
        ofxOscReceiver receiver;
        bool connected;
        void processMessages();
        float lastPing;
};

