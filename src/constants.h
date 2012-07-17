#ifndef _CONSTANTS
#define _CONSTANTS	
//#define SPAN_SCREEN

// David settings
/*
#define SCREEN_WIDTH 2704
#define SCREEN_HEIGHT 768

#define PROJECTION_RECT_X 2029
#define PROJECTION_RECT_Y 223
#define PROJECTION_RECT_WIDTH 414
#define PROJECTION_RECT_HEIGHT 414
*/

// Matt settings
//#define SCREEN_WIDTH 2304
//#define SCREEN_HEIGHT 768

//#define PROJECTION_RECT_X 1533
//#define PROJECTION_RECT_Y 129
#define PROJECTION_RECT_WIDTH 430
#define PROJECTION_RECT_HEIGHT 430

#define SCREEN_WIDTH 3000
#define SCREEN_HEIGHT 1000

#define PROJECTION_RECT_X 2300
#define PROJECTION_RECT_Y 450

// Tony settings
/*
#define SCREEN_WIDTH 2464
#define SCREEN_HEIGHT 768

#define PROJECTION_RECT_X 1679
#define PROJECTION_RECT_Y 310
#define PROJECTION_RECT_WIDTH 420
#define PROJECTION_RECT_HEIGHT 420
*/

// Matthew Vertical Screen Settings
#define VISUALIZATION_X 50
#define VISUALIZATION_Y 800
#define VISUALIZATION_Z -30

#ifndef SPAN_SCREEN
#define PROJECTION_RECT_WIDTH 430
#define PROJECTION_RECT_HEIGHT 430

#define VISUALIZATION_X 125
#define VISUALIZATION_Y 1100
#define VISUALIZATION_Z -10

#define SCREEN_WIDTH  768
#define SCREEN_HEIGHT 1024

#define PROJECTION_RECT_X 1158
#define PROJECTION_RECT_Y 665
#endif

#define FPS 25
#define IDLE_THRESHOLD (FPS*5)
#define IDLE_ROTATION_TIME (FPS*5)
#define MAX_VISIBLE_INSTANCES 5
#define OFFSET_PER_FRAME 20



#define PROJECTION_PIXEL_BORDER 0

#define	RELIEF_SIZE_X 12 
#define	RELIEF_SIZE_Y 12

#define GRID_WIDTH 43
#define FRAME_WIDTH (RELIEF_SIZE_X * GRID_WIDTH)

#define ROI_RECT_X 225
#define ROI_RECT_Y 150
#define ROI_RECT_W 187
#define ROI_RECT_H 187
#define ROI_BORDER_W 150
#define ROI_BORDER_H 45

#define PARALLAX_ORIGIN_X 58
#define PARALLAX_ORIGIN_Y 99
#define PARALLAX_RATIO_X 0.0011
#define PARALLAX_RATIO_Y 0.003

#define CURSOR_OFFSET_SIZE -20
#define CURSOR_OFFSET_X 10
#define CURSOR_OFFSET_Y 10

#define KINECT_THRESHOLD_ENABLED 1;
//#define KINECT_NEAR_THRESHOLD 75
//#define KINECT_FAR_THRESHOLD 178
#define KINECT_NEAR_THRESHOLD 230
#define KINECT_FAR_THRESHOLD 179



#define NUMBER_OF_HANDS 2
#define NUMBER_OF_FINGERS (NUMBER_OF_HANDS * 5)

// Hand feature finding parameters
#define FINGER_Y_CUTOFF 10;
#define FINGER_ANGLE_THRESHOLD -1.03;
#define MAX_NUM_FINGERS	5
#define MAX_CONTOUR_LENGTH 10000
#define BLOB_SIZE 25
#define NPTS_FOR_LINE_FIT 40

// Smoothing parameters
#define AVERAGE_FINGER_DELAY 2
#define AVERAGE_FINGER_JUMP_RADIUS 50
#define HEIGHT_DELAY 5
#define DIRECT_MANIPULATION_DELAY 4
#define CURSOR_DELAY 2
#define COLOR_CHANGE_DELAY_ON 5;
#define COLOR_CHANGE_DELAY_OFF 2;

#define NUM_SERIAL_CONNECTIONS 3
#define MAX_NUM_ARDUINOS_PER_CONNECTION 12
#define NUM_ARDUINOS 32
#define NUM_PINS_ARDUINO 4

#define GESTURE_BUFFER 10

#define RELIEF_HEIGHT_RATIO (12.0f / 127)

#define SERIAL_PORT_2 "/dev/tty.usbserial-A900ceuT"
#define SERIAL_PORT_1 "/dev/tty.usbserial-A800etID"
#define SERIAL_PORT_0 "/dev/tty.usbserial-A900cedr"
#define SERIAL_PORT_0_FIRST_ID 0
#define SERIAL_PORT_1_FIRST_ID 10
#define SERIAL_PORT_2_FIRST_ID 22
#define SERIAL_PORT_0_NUMEROFARDUINOS 10
#define SERIAL_PORT_1_NUMEROFARDUINOS 12
#define SERIAL_PORT_2_NUMEROFARDUINOS 10
#define SERIAL_BAUD_RATE 115200

#define ARDUINO_GAIN_P 150
#define ARDUINO_GAIN_I 35
#define ARDUINO_MAX_I  60
#define ARDUINO_DEADZONE 0

#define RELIEF_CONNECTED 1

#define PI 3.14159265

#define RELIEFSETTINGS "reliefSettings.xml"

#define HOST "localhost"
#define CONNECTION_PORT 78746
#define LISTEN_PORT 89001

#define RELIEF_FLOOR 100
#define RELIEF_CEIL 0

#endif