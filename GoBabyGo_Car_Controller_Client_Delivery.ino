//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE: GoBabyGo Car Controller - Simple 4 Direction
// NAME:  David Long
//
// PURPOSE: This version of the car controller limits the vehicle to one action at a time, either forward, reverse, turning left, or turning right.
//          The vehicle must come to a complete stop before any change in direction is allowed. This creates a small input delay, but prevents motors
//          from getting out of sync. A future version of this program will allow more analog control to the vehicle. The code also allows interaction
//          with the user adjustable values through an LCD interface.
//
//          
#include <Servo.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Adafruit_GFX.h"
#include "Adafruit_TSC2007.h"
#include "Adafruit_HX8357.h"
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*****************************************************************************************************************************
// Only enable this value when data is desired to be written to the EEPROM. The EEPROM has a limited lifespan of 100,000 writes
// so this feature should be used sparingly!!!
const boolean ALLOW_EEPROM_WRITES = true;
//*****************************************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Only have one of the following flags set to true at any given time, or the arduino may not have enough memory						//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
																																		//
// Set this value true if the car will be used with the touchscreen LCD, or false if it will not be.									//
const boolean USE_LCD = true;   																										//
																																		//
// Set this value true when setting up new hardware to calibrate the min speed limit for the motors and joystick sensitivity thresholds	//
const boolean CALIBRATION_MODE_ENABLE = false;																							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Set this value true to enable debug Over Serial (Make sure battery is disconnected from Arduino)
const boolean DEBUG = false;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MOTION CHARACTERISTIC CONSTRAINTS: Control the minimum and maximum values each vehicle parameter can take 
// (Some values may need to be changed depending on your particular hardware as detailed in user documentation)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const uint16_t MIN_FORWARD_THRESH = 600;
const uint16_t MAX_FORWARD_THRESH = 1000;

const uint16_t MIN_REVERSE_THRESH = 0;
const uint16_t MAX_REVERSE_THRESH = 400;

const uint16_t MIN_LEFT_THRESH = 0;
const uint16_t MAX_LEFT_THRESH = 400;

const uint16_t MIN_RIGHT_THRESH = 600;
const uint16_t MAX_RIGHT_THRESH = 1000;

const uint16_t MIN_SPEED_LIMIT = 72;
const uint16_t MAX_SPEED_LIMIT = 512;   // Do not change this value

const uint16_t MIN_INCREASE_RAMPING = 1;
const uint16_t MAX_INCREASE_RAMPING = 30;

const uint16_t MIN_DECREASE_RAMPING = 1;
const uint16_t MAX_DECREASE_RAMPING = 30;

uint8_t FWD_LEFT_TRIM = 0;				// Speed offset for left motor forward, if vehicle drifts right while driving forward, increase this value
uint8_t REV_LEFT_TRIM = 0;				// Speed offset for left motor reverse, if vehicle drifts right while driving backward, increase this value
uint8_t FWD_RIGHT_TRIM = 4;				// Speed offset for right motor forward, if vehicle drifts left while driving forward, increase this value
uint8_t REV_RIGHT_TRIM = 0;				// Speed offset for right motor reverse, if vehicle drifts left while driving backwards, increase this value

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adjustable values: you can change these to fit the user's needs (Values denoted "default" only used if useLCD is set to false)   //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                                                                                    //
// Increase/decrease these values to change the default joystick deadzones                                                          //
// (An increase in the RIGHT or FORWARD values or a decrease in the LEFT or REVERSE values increases the corresponding deadzones)   //                                                                                                                                    //
const uint16_t DEFAULT_FORWARD_THRESH = 630;    // Analog reads above this value should be considered forward joystick inputs       //
const uint16_t DEFAULT_REVERSE_THRESH = 340;    // Analog reads below this value should be considered reverse joystick inputs       //
const uint16_t DEFAULT_LEFT_THRESH    = 400;    // Analog reads below this value should be considered left joystick inputs          //
const uint16_t DEFAULT_RIGHT_THRESH   = 600;    // Analog reads above this value should be considered right joystick inputs         //
                                                                                                                                    //
// Increase/decrease these values to modify the driving characteristics                                                             //
const uint16_t DEFAULT_SPEED_LIMIT = 256;       // Between 100 - 512: This scales down all values sent to the motors                //
                                                // Increase value to increase the speed of the car                                  //
                                                                                                                                    //
const uint8_t DEFAULT_INCREASE_RAMPING = 15;    // This prevents the car from jerking by limiting how fast its speed can increase   //
                                                // If the car is not responding/accelerating fast enough, increase this value       //
                                                                                                                                    //
const uint8_t DEFAULT_DECREASE_RAMPING = 20;    // This prevents the car from jerking by limiting how fast its speed can decrease   //
                                                // If the car is not slowing down fast enough, increase this value                  //
																																	//
const float TURN_SPEED_RATIO = 1.5;    			// How many times slower the motors should spin when turning compared to driving 	//
												// straight (higher value = lower turn speed)										//
																																	//
const float REVERSE_SPEED_RATIO = 1.5;			// How many times slower the motors should spin when going in reverse compared to 	//
												// going forward (higher value = lower reverse speed)								//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                             
// Constants for Rev Robotics Spark Motor Controller
// | Full Reverse | Proportional Reverse |      Neutral      | Proportional Forward | Full Forward |
// |  p <= 1000   |   1000 < p < 1460    | 1460 <= p <= 1540 |   1540 < p < 2000    |  2000 <= p   |
const uint16_t REVERSE_PULSE = 1000;   // Default pulse width for full reverse is 1000us
const uint16_t FORWARD_PULSE = 2000;   // Default pulse width for full forward is 2000us


// Pin Assignments
const int JOYSTICK_X 	  = A7; // Joystick X-input
const int JOYSTICK_Y 	  = A6; // Joystick Y-input
const int TFT_CS 		  = A3; // LCD chip select pin
const int TFT_DC 		  = A2;	// LCD data control pin
const int TFT_RST 		  = -1;	// Reset pins tied between LCD and Arduino
const int TOUCH_INT 	  = 2;	// Touch screen interrupt pin
const int LEFT_MOTOR_PIN  = 10; // digital pin 10
const int RIGHT_MOTOR_PIN = 9;  // digital pin 9


// LCD Colors
const uint16_t BLACK    = 0x0000;
const uint16_t WHITE    = 0xFFFF;
const uint16_t GREY     = 0xBDF7;
const uint16_t GREEN    = 0x07E0;
const uint16_t L_YELLOW = 0xEFB7;


// LCD Menu Elements (Left and Right menus)
const uint16_t MENU_HEADER_HEIGHT      = 51;   	// Height (in pixels) of the header boxes for both the input sensitivity and motion characteristics menus
const uint16_t MENU_HEADER_WIDTH       = 227;  	// Width (in pixels) of the header boxes for both the input sensitivity and motion characteristics menus

const uint16_t MENU_LINE_HEIGHT        = 31;   	// Height (in pixels) of each box in the menu lines of both the input sensitivity and motion characteristics menus

const uint16_t BUTTON_WIDTH            = 31;    // Width (in pixels) of the up and down buttons for both the input sensitivity and motion characteristics menus (1st and 3rd column)
const uint16_t UP_BUTTON_X_OFFSET      = 144;   // Pixel offset in x-direction from top left corner of the menu line to the top left corner of the up button (both menus)

const uint16_t READOUT_BOX_WIDTH       = 43;    // Width (in pixels) of the data readout textbox for both input sensitivity and motion characteristics menu lines (4th column)
const uint16_t READOUT_BOX_X_OFFSET    = 184;   // Pixel offset in x-direction from top left corner of menu line to the top left of the data readout box
const uint16_t READOUT_TEXT_X_OFFSET   = 189;   // Pixel offset in x-direction from top left corner of menu line to the bottom left of the data readout text
const uint16_t READOUT_TEXT_Y_OFFSET   = 21;    // Pixel offset in y-direction from top left corner of menu line to the bottom left of the data readout text


// LCD Menu Elements (Left menu)
const uint16_t L_MENU_LINES_X0         = 5;    	// X-coordinate of top left of all 4 input sensitivity menu lines
const uint16_t L_MENU_LINE1_Y0         = 65;   	// Y-coordinate of top left of input sensitivity menu line 1 (forward sensitivity)
const uint16_t L_MENU_LINE2_Y0         = 105;  	// Y-coordinate of top left of input sensitivity menu line 2 (reverse sensitivity)
const uint16_t L_MENU_LINE3_Y0         = 145;  	// Y-coordinate of top left of input sensitivity menu line 3 (left sensitivity)
const uint16_t L_MENU_LINE4_Y0         = 185;  	// Y-coordinate of top left of input sensitivity menu line 5 (right sensitivity)

const uint16_t SENS_TYPE_BOX_WIDTH     = 95;    // Width (in pixels) of the input sensitivity menu line sensitivity type text box (2nd column)
const uint16_t SENS_TYPE_BOX_X_OFFSET  = 40;    // Pixel offset in x-direction from top left corner of menu line to the top left of the sensitivity type textbox
const uint16_t SENS_TYPE_TEXT_X_OFFSET = 47;    // Pixel offset in x-direction from top left corner of menu line to the bottom left of the sensitivity type text
const uint16_t SENS_TYPE_TEXT_Y_OFFSET = 21;    // Pixel offset in y-direction from top left corner of menu line to the bottom left of the sensitivity type text

const uint16_t L_MENU_INC_BTNS_Y0	   = 224;  	// Y-coordinate of top left of increment selection buttons
const uint16_t INC_BUTTON_WIDTH		   = 43;    // Width (in pixels) of the increment selection buttons
const uint16_t x1_BUTTON_X_OFFSET	   = 40;	// Pixel offset in x-direction from top left corner of menu line x0 to the top left corner of the x1 increment button
const uint16_t x5_BUTTON_X_OFFSET	   = 92;    // Pixel offset in x-direction from top left corner of menu line x0 to the top left corner of the x5 increment button
const uint16_t x10_BUTTON_X_OFFSET	   = 144;   // Pixel offset in x-direction from top left corner of menu line x0 to the top left corner of the x10 increment button
const uint16_t x1_TEXT_X_OFFSET		   = 51;	// Pixel offset in x-direction from top left corner of menu line x0 to the bottom left of the x1 button text
const uint16_t x5_TEXT_X_OFFSET		   = 103;   // Pixel offset in x-direction from top left corner of menu line x0 to the bottom left of the x5 button text
const uint16_t x10_TEXT_X_OFFSET	   = 149;   // Pixel offset in x-direction from top left corner of menu line x0 to the bottom left of the x10 button text
const uint16_t INC_TEXT_Y_OFFSET       = 22;	// Pixel offset in y-direction from top left corner of increment buttons line to the bottom left of the increment button text

const uint16_t L_MENU_SAVE_BOX_Y0      = 266;  	// Y_coordinate of top left of save values button
const uint16_t SAVE_TEXT_Y_OFFSET	   = 23;   	// Pixel offset in y-direction from top left corner of save button box to bottom left of save text
const uint16_t SAVE_TEXT_X_OFFSET	   = 36;	// Pixel offset in x-direction from top left corner of save button box to bottom left of save text


// LCD Menu Elements (Right menu)
const uint16_t R_MENU_LINES_X0         = 249;   // X-coordinate of top left of all 3 motion characteristics menu lines
const uint16_t R_MENU_LINE1_Y0         = 65;    // Y-coordinate of top left of motion characteristics menu line 1 (maximum speed)
const uint16_t R_MENU_LINE2_Y0         = 173;   // Y-coordinate of top left of motion characteristics menu line 2 (acceleration rate)
const uint16_t R_MENU_LINE3_Y0         = 281;   // Y-coordinate of top left of motion characteristics menu line 3 (deceleration rate)

const uint16_t MAG_BAR_BOX_WIDTH       = 103;   // Width (in pixels) of the motion characteristics menu line magnitude bar bounding box (2nd Column)
const uint16_t MAG_BAR_BOX_X_OFFSET    = 36;    // Pixel offset in x-direction from top left corner of motion characteristics menu line to the magnitude bar bounding box


// Input sensitivity menu button touch screen extents (Touch screen coordinates are rotated 90 degrees clockwise from the LCD coordinates)
const uint16_t L_BUTTON_R1_TOUCH_X1 = 1000;  // X value of touchscreen corresponding to top left corner of buttons in row 1 of the left menu screen (buttons for forward sensitivity row)
const uint16_t L_BUTTON_R1_TOUCH_X2 = 1300;  // X value of touchscreen corresponding to bottom right corner of buttons in row 1 of the left menu screen (buttons for forward sensitivity row)

const uint16_t L_BUTTON_R2_TOUCH_X1 = 1450;	 // X value of touchscreen corresponding to top left corner of buttons in row 2 of the left menu screen (buttons for reverse sensitivity row)
const uint16_t L_BUTTON_R2_TOUCH_X2 = 1750;  // X value of touchscreen corresponding to bottom right corner of buttons in row 2 of the left menu screen (buttons for reverse sensitivity row)

const uint16_t L_BUTTON_R3_TOUCH_X1 = 1900;  // X value of touchscreen corresponding to top left corner of buttons in row 3 of the left menu screen (buttons for left sensitivity row)
const uint16_t L_BUTTON_R3_TOUCH_X2 = 2200;  // X value of touchscreen corresponding to bottom right corner of buttons in row 3 of the left menu screen (buttons for left sensitivity row)

const uint16_t L_BUTTON_R4_TOUCH_X1 = 2350;  // X value of touchscreen corresponding to top left corner of buttons in row 4 of the left menu screen (buttons for right sensitivity row)
const uint16_t L_BUTTON_R4_TOUCH_X2 = 2650;  // X value of touchscreen corresponding to bottom right corner of buttons in row 4 of the left menu screen (buttons for right sensitivity row)

const uint16_t L_BUTTON_C1_TOUCH_Y1 = 3870;  // Y value of touchscreen corresponding to top left corner of buttons in column 1 of the left menu screen (down buttons for input sensitivity menu)
const uint16_t L_BUTTON_C1_TOUCH_Y2 = 3650;  // Y value of touchscreen corresponding to bottom right corner of buttons in column 1 of the left menu screen (down buttons for input sensitivity menu)

const uint16_t L_BUTTON_C2_TOUCH_Y1 = 2800;  // Y value of touchscreen corresponding to top left corner of buttons in column 2 of the left menu screen (up buttons for input sensitivity menu)
const uint16_t L_BUTTON_C2_TOUCH_Y2 = 2550;  // Y value of touchscreen corresponding to bottom right corner of buttons in column 2 of the left menu screen (up buttons for input sensitivity menu)


// Increment selection buttons touch screen extents
const uint16_t INC_BTNS_TOUCH_X1 = 2800;	 // X value of touchscreen corresponding to top edge of increment buttons
const uint16_t INC_BTNS_TOUCH_X2 = 3100;	 // X value of touchscreen corresponding to bottom edge of increment buttons

const uint16_t x1_TOUCH_Y1  = 3550;			 // Y value of touchscreen corresponding to left edge of x1 increment button
const uint16_t x1_TOUCH_Y2  = 3200;			 // Y value of touchscreen corresponding to right edge of x1 increment button

const uint16_t x5_TOUCH_Y1  = 3150;			 // Y value of touchscreen corresponding to left edge of x5 increment button
const uint16_t x5_TOUCH_Y2  = 2800;			 // Y value of touchscreen corresponding to right edge of x5 increment button

const uint16_t x10_TOUCH_Y1 = 3750;			 // Y value of touchscreen corresponding to left edge of x10 increment button
const uint16_t x10_TOUCH_Y2 = 2400;			 // Y value of touchscreen corresponding to right edge of x10 increment button

uint8_t readoutIncrement = 1;	// Amount to increment or decrement readouts when updated via button press 


// Save values button touch screen extents
const uint16_t SAVE_VAL_TOUCH_X1 = 3250;	// X value of touchscreen cooresponding to top left corner of the "save values" button
const uint16_t SAVE_VAL_TOUCH_X2 = 3550;	// X value of touchscreen cooresponding to bottom right corner of the "save values" button
const uint16_t SAVE_VAL_TOUCH_Y1 = 3870;	// X value of touchscreen cooresponding to top left corner of the "save values" button
const uint16_t SAVE_VAL_TOUCH_Y2 = 2190;	// X value of touchscreen cooresponding to bottom right corner of the "save values" button


// Motion characteristics menu button touch screen extents
const uint16_t R_BUTTON_R1_TOUCH_X1 = 1000;  // X value of touchscreen corresponding to top left corner of buttons in row 1 of the right menu screen (buttons for maximum speed row)
const uint16_t R_BUTTON_R1_TOUCH_X2 = 1300;  // X value of touchscreen corresponding to bottom right corner of buttons in row 1 of the right menu screen (buttons for maximum speed row)

const uint16_t R_BUTTON_R2_TOUCH_X1 = 2200;  // X value of touchscreen corresponding to top left corner of buttons in row 2 of the right menu screen (buttons for acceleration rate row)
const uint16_t R_BUTTON_R2_TOUCH_X2 = 2550;  // X value of touchscreen corresponding to bottom right corner of buttons in row 2 of the right menu screen (buttons for acceleration rate row)

const uint16_t R_BUTTON_R3_TOUCH_X1 = 3400;  // X value of touchscreen corresponding to top left corner of buttons in row 1 of the right menu screen (buttons for deceleration rate row)
const uint16_t R_BUTTON_R3_TOUCH_X2 = 3700;  // X value of touchscreen corresponding to bottom right corner of buttons in row 1 of the right menu screen (buttons for deceleration rate row)

const uint16_t R_BUTTON_C1_TOUCH_Y1 = 2000;  // Y value of touchscreen corresponding to top left corner of buttons in column 1 of the right menu screen (down buttons for motion characteristics menu)
const uint16_t R_BUTTON_C1_TOUCH_Y2 = 1770;  // Y value of touchscreen corresponding to bottom right corner of buttons in column 1 of the right menu screen (down buttons for motion characteristics menu)

const uint16_t R_BUTTON_C2_TOUCH_Y1 = 860;   // Y value of touchscreen corresponding to top left corner of buttons in column 2 of the right menu screen (up buttons for motion characteristics menu)
const uint16_t R_BUTTON_C2_TOUCH_Y2 = 650;   // Y value of touchscreen corresponding to bottom right corner of buttons in column 2 of the right menu screen (up buttons for motion characteristics menu)


// Create LCD display object using hardware SPI
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);


// Touch screen variables
Adafruit_TSC2007 touch;
uint16_t touch_x  = 0;   // X coordinate of touch input
uint16_t touch_y  = 0;   // Y coordinate of touch input
uint16_t touch_z1 = 0;   // Pressure value 1 of touch input
uint16_t touch_z2 = 0;   // Pressure value 2 of touch input


// Time related variables
const uint16_t CONSECUTIVE_TOUCH_DELAY = 100;   // Number of milliseconds that must past before another touch input should be accepted
unsigned long timePressed;                      // The time the most recent touch input was detected
unsigned long prevTimePressed = millis();       // The time the previous touch input was detected


// Calibration mode variables
const uint16_t SPEED_CALIBRATION_TIME_INTERVAL = 2000;			// Number of milliseconds to wait between increasing speedLimit in min speed calibration mode
const uint16_t COURSE_SPEED_CALIBRATION_INC = 32;				// Amount to increment speedLimit by during course min speed calibration process
const uint16_t FINE_SPEED_CALIBRATION_INC = 2;					// Amount to increment speedLimit by during fine min speed callibration process
boolean calibrationStarted = false;								// Indicates if the calibration process has started (user has responded to all prompts)
String calibrationMode = "";									// Indicates Which calibration mode was selected (speed limit '1', or joystick sensitivity '2') 
typedef enum {COURSE, FINE} incType;							// Indicates if increment in calibration mode should be COURSE or FINE 
incType courseOrFine = COURSE;
				

// Vehicle motion and sensitivity characteristics
// These values will change depending on whether reading from EEPROM or using default values defined by constants in "Adjustable Values" block
// Either way they must stay within the min and max values defined by constants in the "Reasonable Vehicle Parameter Ranges" block
uint16_t forwardThresh;
uint16_t reverseThresh;
uint16_t leftThresh;
uint16_t rightThresh;
uint16_t speedLimit;
int16_t  accRate;
int16_t  decRate;


// Motion characteristic percentage values (these are displayed on the LCD)
uint16_t forwardSensPercent;
uint16_t reverseSensPercent;
uint16_t leftSensPercent;
uint16_t rightSensPercent;
uint16_t speedLimitPercent;
uint16_t accRatePercent;
uint16_t decRatePercent;


// EEPROM Memory Addresses
uint16_t FORWARD_SENS_ADDRESS = 1;  // Address forwardSensPercent will be read from if USE_LCD enabled
uint16_t REVERSE_SENS_ADDRESS = 2;  // Address reverseSensPercent will be read from if USE_LCD enabled
uint16_t LEFT_SENS_ADDRESS = 3;     // Address leftSensPercent will be read from if USE_LCD enabled
uint16_t RIGHT_SENS_ADDRESS = 4;    // Address rightSensPercent will be read from if USE_LCD enabled
uint16_t SPEED_LIMIT_ADDRESS = 5;   // Address speedLimitPercent will be read from if USE_LCD enabled
uint16_t ACC_RATE_ADDRESS = 6;      // Address accRatePercent will be read from if USE_LCD enabled
uint16_t DEC_RATE_ADDRESS = 7;      // Address decRatePercent will be read from if USE_LCD enabled


// Create Servo object for both motors
Servo leftMotor;
Servo rightMotor;


void setup() {    
    // Pin configurations
    pinMode(JOYSTICK_X, INPUT);
    pinMode(JOYSTICK_Y, INPUT);
    pinMode(LEFT_MOTOR_PIN,OUTPUT);
    pinMode(RIGHT_MOTOR_PIN,OUTPUT);
    pinMode(TOUCH_INT, INPUT_PULLUP);

    // Set up interrupt pin to detect touch screen presses
    attachInterrupt(digitalPinToInterrupt(TOUCH_INT), touchDetected, FALLING);

    leftMotor.attach(LEFT_MOTOR_PIN);
    rightMotor.attach(RIGHT_MOTOR_PIN);

    // Wait for Serial connection to be made
    Serial.begin(9600);
    while (!Serial) delay(10);
     
    
    // If the LCD should be used
    if (USE_LCD) {
        
		// Read in the sensitivity and motion characteristic values from the Arduino EEPROM        
		forwardSensPercent = EEPROM.read(FORWARD_SENS_ADDRESS);
		reverseSensPercent = EEPROM.read(REVERSE_SENS_ADDRESS);
		leftSensPercent    = EEPROM.read(LEFT_SENS_ADDRESS);
		rightSensPercent   = EEPROM.read(RIGHT_SENS_ADDRESS);
		speedLimitPercent  = EEPROM.read(SPEED_LIMIT_ADDRESS);
		accRatePercent     = EEPROM.read(ACC_RATE_ADDRESS);
		decRatePercent     = EEPROM.read(DEC_RATE_ADDRESS);

		// If any of the values read from the EEPROM were >100, set them to 100. This is useful for the first program run after installing a new Arduino
		// where the EEPROM may not have been written to before (Data defaults to 255 at every unwritten EEPROM address)
		if (forwardSensPercent > 100) forwardSensPercent = 100;
		if (reverseSensPercent > 100) reverseSensPercent = 100;
		if (leftSensPercent > 100) leftSensPercent = 100;
		if (rightSensPercent > 100) rightSensPercent = 100;
		if (speedLimitPercent > 100) speedLimitPercent = 100;
		if (accRatePercent > 100) accRatePercent = 100;
		if (decRatePercent > 100) decRatePercent = 100;

		// Convert EEPROM read values from percents to counts (like the constants at top of code)
		forwardThresh = map(forwardSensPercent, 0, 100, MAX_FORWARD_THRESH, MIN_FORWARD_THRESH);
		reverseThresh = map(reverseSensPercent, 0, 100, MIN_REVERSE_THRESH, MAX_REVERSE_THRESH);
		leftThresh	  = map(leftSensPercent, 0, 100, MIN_LEFT_THRESH, MAX_LEFT_THRESH);
		rightThresh	  = map(rightSensPercent, 0, 100, MAX_RIGHT_THRESH, MIN_RIGHT_THRESH);
		speedLimit	  = map(speedLimitPercent, 0, 100, MIN_SPEED_LIMIT, MAX_SPEED_LIMIT);
		accRate		  = map(accRatePercent, 0, 100, MIN_INCREASE_RAMPING, MAX_INCREASE_RAMPING);
		decRate		  = map(decRatePercent, 0, 100, MIN_DECREASE_RAMPING, MAX_DECREASE_RAMPING);       

        // Wait for touchscreen to connect
        if (!touch.begin()) {
            while (1) delay(10);
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Draw Initial Screen Layout
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        tft.begin();
        tft.setRotation(1);         // Rotate screen to landscape mode
        tft.fillScreen(WHITE);      // Set screen background to white
        tft.setTextColor(BLACK);    // Set default text color to black
    
        ///////////////////////////////////////
        // Draw Input Sensitivity (%) Header
        ///////////////////////////////////////
        drawMenuBox(5, 5, MENU_HEADER_WIDTH, MENU_HEADER_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(81, 23);
        tft.println("Input");
        tft.setCursor(23, 46);
        tft.println("Sensitivity(%)");
    
        ///////////////////////////////////////
        // Draw Forward Sensitivity Menu Line
        ///////////////////////////////////////
        drawLeftMenuLine(L_MENU_LINES_X0, L_MENU_LINE1_Y0, BLACK, GREY, "Forward", forwardSensPercent);
    
        ///////////////////////////////////////
        // Draw Reverse Sensitivity Menu Line
        ///////////////////////////////////////
        drawLeftMenuLine(L_MENU_LINES_X0, L_MENU_LINE2_Y0, BLACK, GREY, "Reverse", reverseSensPercent);
    
        ///////////////////////////////////////
        // Draw Left Sensitivity Menu Line
        ///////////////////////////////////////
        drawLeftMenuLine(L_MENU_LINES_X0, L_MENU_LINE3_Y0, BLACK, GREY, "Left", leftSensPercent);
    
        ///////////////////////////////////////
        // Draw Right Sensitivity Menu Line
        ///////////////////////////////////////
        drawLeftMenuLine(L_MENU_LINES_X0, L_MENU_LINE4_Y0, BLACK, GREY, "Right", rightSensPercent);
		
		///////////////////////////////////////
		// Draw Increment Selection Buttons
		///////////////////////////////////////
		drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 1, BLACK, L_YELLOW);
		drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 5, BLACK, GREY);
		drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 10, BLACK, GREY);		
    
        ///////////////////////////////////////
        // Draw Save Values Button
        ///////////////////////////////////////       
		drawSaveValuesButton(BLACK, GREY);
    
        ////////////////////////////////////////////////////////
        // Draw Maximum Speed (%) Header, Control, and Readout
        ////////////////////////////////////////////////////////
        drawMenuBox(249, 5, MENU_HEADER_WIDTH, MENU_HEADER_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(273, 23);
        tft.println("Maximum Speed");
        tft.setCursor(340, 46);
        tft.println("(%)");   
        drawRightMenuLine(R_MENU_LINES_X0, R_MENU_LINE1_Y0, BLACK, GREY, speedLimitPercent);
    
        ///////////////////////////////////////////////////////////
        // Draw Acceleration Rate (%) Header, Control, and Readout
        ///////////////////////////////////////////////////////////
        drawMenuBox(249, 113, MENU_HEADER_WIDTH, MENU_HEADER_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(281, 131);
        tft.println("Acceleration");
        tft.setCursor(322, 153);
        tft.println("Rate(%)");   
        drawRightMenuLine(R_MENU_LINES_X0, R_MENU_LINE2_Y0, BLACK, GREY, accRatePercent);
    
        ///////////////////////////////////////////////////////////
        // Draw Deceleration Rate (%) Header, Control, and Readout
        ///////////////////////////////////////////////////////////
        drawMenuBox(249, 221, MENU_HEADER_WIDTH, MENU_HEADER_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(281, 239);
        tft.println("Deceleration");
        tft.setCursor(322, 262);
        tft.println("Rate(%)");   
        drawRightMenuLine(R_MENU_LINES_X0, R_MENU_LINE3_Y0, BLACK, GREY, decRatePercent);
        delay(100);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
	// } else if (not CALIBRATE_SPEED_LIMIT and not CALIBRATE_DEADZONES) { 	// Use default values in "Adjustable Values" block as long as calibration mode not enabled
	} else {
        // Set motion characteristics to default values
		forwardThresh = DEFAULT_FORWARD_THRESH;
        reverseThresh = DEFAULT_REVERSE_THRESH;
        leftThresh    = DEFAULT_LEFT_THRESH;
        rightThresh   = DEFAULT_RIGHT_THRESH;
        speedLimit    = DEFAULT_SPEED_LIMIT;
        accRate       = DEFAULT_INCREASE_RAMPING;
        decRate       = DEFAULT_DECREASE_RAMPING;
    } // End if/else			
} // End setup()





void loop() {	
    if (USE_LCD) {      		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Touchscreen Interaction
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
        // If the touchDetected ISR was triggered and the appropriate ammount of time has passed since the last input, register a new touch input
        if (timePressed > prevTimePressed + CONSECUTIVE_TOUCH_DELAY) {	
            touch.read_touch(&touch_x, &touch_y, &touch_z1, &touch_z2);		// Read touchscreen
            prevTimePressed = timePressed;									// Store time touchscreen was pressed at
            
            // Determine which (if any) button was pressed
            if (touch_y < L_BUTTON_C1_TOUCH_Y1 && touch_y > L_BUTTON_C1_TOUCH_Y2) {             // A decrease button was pressed in the input sensitivity menu           
                if (touch_x > L_BUTTON_R1_TOUCH_X1 && touch_x < L_BUTTON_R1_TOUCH_X2) {             // Decrease forward sensitivity
                    updateSensitivityReadout(1, 'D', readoutIncrement);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE1_Y0, 'D', BLACK, L_YELLOW);             	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE1_Y0, 'D', BLACK, GREY);         
					
                } else if (touch_x > L_BUTTON_R2_TOUCH_X1 && touch_x < L_BUTTON_R2_TOUCH_X2) {      // Decrease reverse sensitivity
                    updateSensitivityReadout(2, 'D', readoutIncrement);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE2_Y0, 'D', BLACK, L_YELLOW);             	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE2_Y0, 'D', BLACK, GREY); 
					
                } else if (touch_x > L_BUTTON_R3_TOUCH_X1 && touch_x < L_BUTTON_R3_TOUCH_X2) {      // Decrease left sensitivity
                    updateSensitivityReadout(3, 'D', readoutIncrement);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE3_Y0, 'D', BLACK, L_YELLOW);             	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE3_Y0, 'D', BLACK, GREY); 
					
                } else if (touch_x > L_BUTTON_R4_TOUCH_X1 && touch_x < L_BUTTON_R4_TOUCH_X2) {      // Decrease right sensitivity
                    updateSensitivityReadout(4, 'D', readoutIncrement);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE4_Y0, 'D', BLACK, L_YELLOW);             	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE4_Y0, 'D', BLACK, GREY);				
                } 
                
                
            } else if (touch_y < L_BUTTON_C2_TOUCH_Y1 && touch_y > L_BUTTON_C2_TOUCH_Y2) {                  // An increase button was pressed in the input sensitivity menu
                if (touch_x > L_BUTTON_R1_TOUCH_X1 && touch_x < L_BUTTON_R1_TOUCH_X2) {                         // Increase forward sensitivity
                    updateSensitivityReadout(1, 'U', readoutIncrement);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE1_Y0, 'U', BLACK, L_YELLOW);    	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE1_Y0, 'U', BLACK, GREY); 
					
                } else if (touch_x > L_BUTTON_R2_TOUCH_X1 && touch_x < L_BUTTON_R2_TOUCH_X2) {                  // Increase reverse sensitivity
                    updateSensitivityReadout(2, 'U', readoutIncrement);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE2_Y0, 'U', BLACK, L_YELLOW);    	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE2_Y0, 'U', BLACK, GREY);
					
                } else if (touch_x > L_BUTTON_R3_TOUCH_X1 && touch_x < L_BUTTON_R3_TOUCH_X2) {                  // Increase left sensitivity
                    updateSensitivityReadout(3, 'U', readoutIncrement);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE3_Y0, 'U', BLACK, L_YELLOW);    	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE3_Y0, 'U', BLACK, GREY);
					
                } else if (touch_x > L_BUTTON_R4_TOUCH_X1 && touch_x < L_BUTTON_R4_TOUCH_X2) {                  // Increase right sensitivity
                    updateSensitivityReadout(4, 'U', readoutIncrement);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE4_Y0, 'U', BLACK, L_YELLOW);    	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE4_Y0, 'U', BLACK, GREY);    
                }
                
            } else if (touch_y < R_BUTTON_C1_TOUCH_Y1 && touch_y > R_BUTTON_C1_TOUCH_Y2) {      // A decrease button was pressed in the motion characteristics menu           
                if (touch_x > R_BUTTON_R1_TOUCH_X1 && touch_x < R_BUTTON_R1_TOUCH_X2) {             // Decrease maximum speed
                    updateMotionCharReadout(1, 'D', readoutIncrement);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE1_Y0, 'D', BLACK, L_YELLOW);             	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE1_Y0, 'D', BLACK, GREY);
                    
                } else if (touch_x > R_BUTTON_R2_TOUCH_X1 && touch_x < R_BUTTON_R2_TOUCH_X2) {      // Decrease acceleration rate
                    updateMotionCharReadout(2, 'D', readoutIncrement);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE2_Y0, 'D', BLACK, L_YELLOW);             	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE2_Y0, 'D', BLACK, GREY);
                    
                } else if (touch_x > R_BUTTON_R3_TOUCH_X1 && touch_x < R_BUTTON_R3_TOUCH_X2) {      // Decrease deceleration rate
                    updateMotionCharReadout(3, 'D', readoutIncrement);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE3_Y0, 'D', BLACK, L_YELLOW);             	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE3_Y0, 'D', BLACK, GREY);
                } 
                
            } else if (touch_y < R_BUTTON_C2_TOUCH_Y1 && touch_y > R_BUTTON_C2_TOUCH_Y2) {                      // An increase button was pressed in the motion characteristics menu
                if (touch_x > R_BUTTON_R1_TOUCH_X1 && touch_x < R_BUTTON_R1_TOUCH_X2) {                             // Increase maximum speed
                    updateMotionCharReadout(1, 'U', readoutIncrement);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE1_Y0, 'U', BLACK, L_YELLOW);        	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE1_Y0, 'U', BLACK, GREY);
                    
                } else if (touch_x > R_BUTTON_R2_TOUCH_X1 && touch_x < R_BUTTON_R2_TOUCH_X2) {                  	// Increase acceleration rate
                    updateMotionCharReadout(2, 'U', readoutIncrement);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE2_Y0, 'U', BLACK, L_YELLOW);        	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE2_Y0, 'U', BLACK, GREY);
                    
                } else if (touch_x > R_BUTTON_R3_TOUCH_X1 && touch_x < R_BUTTON_R3_TOUCH_X2) {                  	// Increase deceleration rate
                    updateMotionCharReadout(3, 'U', readoutIncrement);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE3_Y0, 'U', BLACK, L_YELLOW);        	// Flash button (Draw with yellow background, wait, then redraw with grey background)
                    delay(100);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE3_Y0, 'U', BLACK, GREY);
                } 
            }
			
            if (touch_x > INC_BTNS_TOUCH_X1 && touch_x < INC_BTNS_TOUCH_X2) {			// One of the increment selection buttons was pressed
				if (touch_y < x1_TOUCH_Y1 && touch_y > x1_TOUCH_Y2) {							// x1 Increment selected
					readoutIncrement = 1;
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 1, BLACK, L_YELLOW);			// Redraw x1 button with yellow background to denote as the active increment
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 5, BLACK, GREY);				// Redraw x5 button with grey background (denotes inactive)
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 10, BLACK, GREY);			// Redraw x10 button with grey background (denotes inactive)
				
				} else if (touch_y < x5_TOUCH_Y1 && touch_y > x5_TOUCH_Y2) {					// x5 Increment selected
					readoutIncrement = 5;
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 5, BLACK, L_YELLOW);			// Redraw x5 button with yellow background to denote as the active increment
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 1, BLACK, GREY);				// Redraw x1 button with grey background (denotes inactive)
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 10, BLACK, GREY);			// Redraw x10 button with grey background (denotes inactive)
				
				} else if (touch_y < x10_TOUCH_Y1 && touch_y > x10_TOUCH_Y2) {					// x10 Increment selected
					readoutIncrement = 10;
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 10, BLACK, L_YELLOW);		// Redraw x10 button with yellow background to denote as the active increment
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 1, BLACK, GREY);				// Redraw x1 button with grey background (denotes inactive)
					drawIncButton(L_MENU_LINES_X0, L_MENU_INC_BTNS_Y0, 5, BLACK, GREY);				// Redraw x5 button with grey background (denotes inactive)
				}			
			} 
			
			if (touch_x > SAVE_VAL_TOUCH_X1 && touch_x < SAVE_VAL_TOUCH_X2 && touch_y < SAVE_VAL_TOUCH_Y1 && touch_y > SAVE_VAL_TOUCH_Y2) {     // The save values button was pressed           
                // Flash the touch box momentarily
				drawSaveValuesButton(BLACK, L_YELLOW);               
                delay(100);
				drawSaveValuesButton(BLACK, GREY);                
    
                // Save values to EEPROM
                // *CAREFUL* The EEPROM has a limited lifespan of 100,000 writes. Do not overuse this function! 
                if (ALLOW_EEPROM_WRITES) {
                    tft.setFont(&FreeMonoBold9pt7b);
                    tft.setCursor(78, 313);
                    tft.println("*SAVED*");
					
                    EEPROM.update(FORWARD_SENS_ADDRESS, forwardSensPercent);
                    EEPROM.update(REVERSE_SENS_ADDRESS, reverseSensPercent);
                    EEPROM.update(LEFT_SENS_ADDRESS, leftSensPercent);
                    EEPROM.update(RIGHT_SENS_ADDRESS, rightSensPercent);
                    EEPROM.update(SPEED_LIMIT_ADDRESS, speedLimitPercent);
                    EEPROM.update(ACC_RATE_ADDRESS, accRatePercent);
                    EEPROM.update(DEC_RATE_ADDRESS, decRatePercent);
					
					// Updates motion charateristics from EEPROM values
					forwardThresh = map(forwardSensPercent, 0, 100, MIN_FORWARD_THRESH, MAX_FORWARD_THRESH);
					reverseThresh = map(reverseSensPercent, 0, 100, MIN_REVERSE_THRESH, MAX_REVERSE_THRESH);
					leftThresh	  = map(leftSensPercent, 0, 100, MIN_LEFT_THRESH, MAX_LEFT_THRESH);
					rightThresh	  = map(rightSensPercent, 0, 100, MIN_RIGHT_THRESH, MAX_RIGHT_THRESH);
					speedLimit	  = map(speedLimitPercent, 0, 100, MIN_SPEED_LIMIT, MAX_SPEED_LIMIT);
					accRate		  = map(accRatePercent, 0, 100, MIN_INCREASE_RAMPING, MAX_INCREASE_RAMPING);
					decRate		  = map(decRatePercent, 0, 100, MIN_DECREASE_RAMPING, MAX_DECREASE_RAMPING);
					
                } else {
                    tft.setFont(&FreeMonoBold9pt7b);
                    tft.setCursor(25, 313);
                    tft.println("*SAVING DISABLED*");
                }
            }
        }
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    } // End if(USE_LCD)

	
		
    if (CALIBRATION_MODE_ENABLE and not calibrationStarted) {	 
		Serial.println("Calibration Mode: Press either '1' or '2' followed by 'enter' to begin the cooresponding calibration mode");
		Serial.println("1: Minimum Motor Speed Calibration");
		Serial.println("2: Joystick sensitivity Calibration\n");
		Serial.println("NOTE: The wheels will spin during this process, make sure the vehicle is");
		Serial.println("      elevated off the ground and the wheel can spin freely\n");
		
		// Clear any garbage stuck in the serial buffer
		serialDump();		
		
		// Wait for user input
		while (Serial.available() == 0);
		
		// Read in user input
		calibrationMode = Serial.readStringUntil('\n');
		serialDump();		
		
		// If user chose speed limit calibration mode
		if (calibrationMode == "1") {
			speedLimit = 32;
			accRate = 50;
			decRate = 50;
			forwardThresh = 900;
			reverseThresh = 100;
			leftThresh = 100;
			rightThresh = 900;
			FWD_LEFT_TRIM = 0;				
			REV_LEFT_TRIM = 0;	
			FWD_RIGHT_TRIM = 0;			
			REV_RIGHT_TRIM = 0;	
			
			Serial.println("MINIMUM MOTOR SPEED CALIBRATION SELECTED");
			Serial.println("Push and hold joystick into the forward track, then");
			Serial.println("Press 'Enter' to begin the calibration process.\n");
			
			// Wait for user to press enter
			while (Serial.available() == 0);
			serialDump();
			
			Serial.println(F("BEGINNING CALIBRATION"));
			Serial.println(F("Push the joystick into the forward track and check if the wheels spin.\n"));
			Serial.println(F("List of Controls:"));
			Serial.print  (F("C/c -> Select minSpeed COURSE increment : "));
			Serial.println(COURSE_SPEED_CALIBRATION_INC);
			Serial.print  (F("F/f -> Select minSpeed FINE increment : "));
			Serial.println(FINE_SPEED_CALIBRATION_INC);
			Serial.println(F("\nU/u -> increase minimum speedLimit by selected increment"));
			Serial.println(F("D/d -> decrease minimum speedLimit by selected increment\n"));
			Serial.println(F("1 -> Increase Left Trim's FORWARD Value"));
			Serial.println(F("2 -> Decrease Left Trim's FORWARD Value\n"));
			Serial.println(F("3 -> Increase Left Trim's REVERSE Value"));
			Serial.println(F("4 -> Decrease Left Trim's REVERSE Value\n"));
			Serial.println(F("5 -> Increase Right Trim's FORWARD Value"));
			Serial.println(F("6 -> Decrease Right Trim's FORWARD Value\n"));
			Serial.println(F("7 -> Increase Right Trim's REVERSE Value"));
			Serial.println(F("8 -> Decrease Right Trim's REVERSE Value\n"));			
			Serial.println(F("E/e -> Exit minimum speedLimit calibration menu\n"));
			Serial.println(F("Course Increment Default"));
			Serial.print  (F("speedLimit: "));			
			Serial.println(speedLimit);
			Serial.print  (F("\nLEFT Forward Trim: "));
			Serial.println(FWD_LEFT_TRIM);
			Serial.print  (F("LEFT Reverse Trim: "));
			Serial.println(REV_LEFT_TRIM);
			Serial.print  (F("RIGHT Forward Trim: "));
			Serial.println(FWD_RIGHT_TRIM);
			Serial.print  (F("RIGHT Reverse Trim: "));
			Serial.println(REV_RIGHT_TRIM);
			Serial.print("\n");
			calibrationStarted = true;			
			serialDump();
			
		} else if (calibrationMode == "2") {
			Serial.println("JOYSTICK SENSITIVY CALIBRATION SELECTED");
			Serial.println("This feature has not been implemented yet. Please see user documentation for procedure");
		} else {
			Serial.println("INVALID SELECTION\n");
		}		
		
	} else {	
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Vehicle operation
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		getJoystickInputs();
	}
} // End loop()




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   Draws a box with a 2 pixel wide border and solid fill color
 *@param:   x0 -> Top left corner x coordinate
 *@param:   y0 -> Top left corner y coordinate
 *@param    w  -> Width in pixels
 *@param    h  -> Height in pixels
 *@param    border_color -> 16-bit 5-6-5 Color to draw border with
 *@param    fill_color -> 16-bit 5-6-5 Color to fill with
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawMenuBox(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t border_color, uint16_t fill_color) {
    // Draw 2-pixel thick border rectangle
    tft.drawRect(x0, y0, w, h, border_color);
    tft.drawRect(x0+1, y0+1, w-2, h-2, border_color); 

    // Fill in border with color
    tft.fillRect(x0+2, y0+2, w-4, h-4, fill_color);

} // End drawMenuBox()




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   Draws initial menu items for the left side of the screen
 *@param:   x0 -> Top left corner x coordinate
 *@param:   y0 -> Top left corner y coordinate
 *@param    border_color  -> the color the box borders should be drawn with
 *@param    fill_color  -> the color the boxes should be filled with
 *@param    menuDataType -> The text that should be printed to show what value the menu item line is for
 *@param    menuDataVal -> The initial value that should be displayed in the data readout textbox
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawLeftMenuLine(uint16_t x0, uint16_t y0, uint16_t border_color, uint16_t fill_color, String menuDataType, uint8_t menuDataVal) {
    tft.setTextColor(BLACK);
    tft.setFont(&FreeMonoBold9pt7b);
                                 
    drawButton(x0, y0, 'D', border_color, fill_color);                                                              // Draw decrease button
    
    drawMenuBox(x0 + SENS_TYPE_BOX_X_OFFSET, y0, SENS_TYPE_BOX_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);  // Draw data type textBox
    tft.setCursor(x0 + SENS_TYPE_TEXT_X_OFFSET, y0 + SENS_TYPE_TEXT_Y_OFFSET);                                      // Set cursor to proper position in data type textBox
    tft.println(menuDataType);                                                                                      // Print menuDataType string to screen
            
    drawButton(x0 + UP_BUTTON_X_OFFSET, y0, 'U', border_color, fill_color);                                         // Draw increase button 
    
    drawMenuBox(x0 + READOUT_BOX_X_OFFSET, y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);      // Draw value readout textBox
    tft.setCursor(x0 + READOUT_TEXT_X_OFFSET, y0 + READOUT_TEXT_Y_OFFSET);                                          // Set cursor to proper position in data readout textBox
    tft.println(String(menuDataVal));                                                                               // Convert menuDataVal to string and print to screen 
    
} // End drawLeftMenuLine()




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   Draws initial menu items for the right side of the screen
 *@param:   x0 -> Top left corner x coordinate
 *@param:   y0 -> Top left corner y coordinate
 *@param    border_color  -> the color the box borders should be drawn with
 *@param    fill_color  -> the color the boxes be filled with
 *@param    menuDataVal -> The initial value that should be displayed in the data readout textbox
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawRightMenuLine(uint16_t x0, uint16_t y0, uint16_t border_color, uint16_t fill_color, uint8_t menuDataVal) {
    tft.setTextColor(BLACK);
    tft.setFont(&FreeMonoBold9pt7b);

    drawButton(x0, y0, 'D', border_color, fill_color);                                                          // Draw decrease button
    
    drawMenuBox(x0 + MAG_BAR_BOX_X_OFFSET, y0, MAG_BAR_BOX_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);  // Draw data type textBox
    drawMagnitudeBar(x0 + MAG_BAR_BOX_X_OFFSET, y0, menuDataVal);

    drawButton(x0 + UP_BUTTON_X_OFFSET, y0, 'U', border_color, fill_color);                                     // Draw increase button 
    
    drawMenuBox(x0 + READOUT_BOX_X_OFFSET, y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);  // Draw value readout textBox
    tft.setCursor(x0 + READOUT_TEXT_X_OFFSET, y0 + READOUT_TEXT_Y_OFFSET);                                      // Set cursor to proper position in data readout textBox
    tft.println(String(menuDataVal));                                                                           // Print menuDataVal string to screen 
} // End drawRightMenuLine()




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   Draws an arrow button at given coordinates
 *@param:   x0 -> Top left corner x coordinate
 *@param:   y0 -> Top left corner y coordinate
 *@param    buttonType -> What button to draw ('U' = Up arrow, 'D' = down arrow)
 *@param    border_color  -> the color the bounding box borders and button arrow should be drawn with
 *@param    fill_color  -> the color the bounding box should be filled with
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawButton(uint16_t x0, uint16_t y0, char buttonType, uint16_t border_color, uint16_t fill_color) {
    drawMenuBox(x0, y0, BUTTON_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);                                  
    if (buttonType == 'D') {                                                                 // Draw decrease button
        tft.fillTriangle(x0 + 6, y0 + 17, x0 + 25, y0 + 17, x0 + 15, y0 + 27, BLACK);           // Draw down arrow triangle
        tft.fillRect(x0 + 12, y0 + 5, 8, 13, BLACK);                                            // Draw down arrow rectangle
    } else if (buttonType == 'U') {                                                          // Draw increase button
        tft.fillTriangle(x0 + 6, y0 + 14, x0 + 25, y0 + 14, x0 + 15, y0 + 4, BLACK);         // Draw up arrow triangle
        tft.fillRect(x0 + 12, y0 + 14, 8, 13, BLACK);                                           // Draw up arrow rectangle
    }  
} // End drawButton()




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:	Draws an increment selection button at given coordinates
 *@param:	x0 -> Top left corner x coordinate
 *@param:   y0 -> Top left corner y coordinate
 *@param:	incType -> Which increment button to draw (x1, x5, or x10)
 *@param:	border_color -> the color the bounding box borders should be drawn with
 *@param:   fill_color -> the color the bounding box should be filled with
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawIncButton(uint16_t x0, uint16_t y0, uint8_t incType, uint16_t border_color, uint16_t fill_color) {
	tft.setFont(&FreeMonoBold9pt7b);
	
	if (incType == 1) {
		drawMenuBox(x0 + x1_BUTTON_X_OFFSET, y0, INC_BUTTON_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);	 // Draw the x1 increment button
		tft.setCursor(x0 + x1_TEXT_X_OFFSET, y0 + INC_TEXT_Y_OFFSET);
		tft.println("x1");
	} else if (incType == 5) {
		drawMenuBox(x0 + x5_BUTTON_X_OFFSET, y0, INC_BUTTON_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);	 // Draw the x1 increment button
		tft.setCursor(x0 + x5_TEXT_X_OFFSET, y0 + INC_TEXT_Y_OFFSET);
		tft.println("x5");
	} else if (incType == 10) {
		drawMenuBox(x0 + x10_BUTTON_X_OFFSET, y0, INC_BUTTON_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);	 // Draw the x1 increment button
		tft.setCursor(x0 + x10_TEXT_X_OFFSET, y0 + INC_TEXT_Y_OFFSET);
		tft.println("x10");
	}
} // End drawIncButton()




////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:	Draws the save value button
 *@param:	border_color -> the color the bounding box borders should be drawn with
 *@param:   fill_color -> the color the bounding box should be filled with
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawSaveValuesButton(uint16_t border_color, uint16_t fill_color) {
	drawMenuBox(L_MENU_LINES_X0, L_MENU_SAVE_BOX_Y0, MENU_HEADER_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);
    tft.setFont(&FreeMonoBold12pt7b);
	tft.setCursor(L_MENU_LINES_X0 + SAVE_TEXT_X_OFFSET, L_MENU_SAVE_BOX_Y0 + SAVE_TEXT_Y_OFFSET);
    tft.println("Save Values"); 
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   Draws the magnitude bar made up of 25 segments where the number of segments filled with a color gradient represents the %magnitude
 *@param:   x0 -> Top left corner x coordinate
 *@param:   y0 -> Top left corner y coordinate
 *@param    percentMagnitude -> The percentage magnitude the bar should represent (how many segments should be filled with a gradient)
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawMagnitudeBar(uint16_t x0, uint16_t y0, uint8_t percentMagnitude) {

    // Convert percentage into number of bars where each bar = 4% (cast to float to get better rounding then cast back to int)
    uint8_t numSegmentsToFill = int(round(float(percentMagnitude) / 4.0));   

    // If percentMagnitude is not zero, at least one bar should be filled
    if (percentMagnitude > 0 && numSegmentsToFill == 0) {   
        numSegmentsToFill = 1;
    }
    
    uint8_t currBar = 1;
    uint16_t currBarX0 = x0 + 2;
    uint16_t currBarY0 = y0 + 27;
    uint8_t currBarHeight = 2;
    uint16_t currBarColor = GREEN;

    // If there are bars to fill, the initial color is green, otherwise the initial color is white.
    if (numSegmentsToFill == 0) {
        currBarColor = WHITE;
    } else {
        currBarColor = GREEN;
    } 
	
    // Logic to determine what color each bar should be filled with (B denotes a binary, D a decimal, and 0x a hexadecimal value)
    //
    // Colors on the LCD are packed into a single 16-bit number where the the 5 most significant bits are for red, the middle 6 bits are for green, and the 5 least significant bits are for blue. 
    // The magnitude bar is filled with a gradient from green to red where the first color is (Rd=B00000 Gr=B111111 Bl=B00000 -> 0x07E0) the color at the middle of the gradient is (Rd=B11111 Gn=B111111 Bl=B00000) 
    // and the last color is (Rd=B11111 Gn=B000000 Bl=B00000). Since there are 25 segments, the transition from start to center of the gradient should take 12 segments and the transition from center to end 
    // of the gradient should take the remaining 13 segments. 
    //
    // A transition from Rd=B00000=D0 to Rd=B11111=D31 over 12 segments requires a change of 2.58 per segment. The value can only be increased by integers, however, so the red values of the first 2 segments 
    // of the transition will be increased by 2 and the last 10 will be increased by 3. Since the red portion of the packed 16-bit RGB color is represented by the 5 most significant bits, an increase of 2 in 
    // the red value is equivalent to an increase of (B0001000000000000=0x1000) for the packed color value and an increase of 3 in the red value is equivalent to an increase of (B0001100000000000=0x1800) to 
    // the packed color value. 
    //
    // The transition from the center to the end of the gradient represents a change from Bl=B111111=D63 to Bl=000000=D0 over 13 segments requires a change of 4.85 per segment. This will be split up such that
    // the blue values of the first 11 segments will be decreased by 5 and the last 2 will be decreased by 4. A decrease of 5 in the blue value is equivalent to a decrease of (B0000000010100000=0x00A0) for 
    // the packed color value and a decrease of 4 in the blue value is equivalent to a decrease of (B0000000010000000=0x0080) for the the packed color value.
    //
    // The gradient stops and the remaining segments are filled with white once the desired number of segments are filled
    
    for (currBar = 1; currBar <= 25; currBar++) {      
        // Draw the segment
        tft.fillRect(currBarX0, currBarY0, 3, currBarHeight, currBarColor);
        
        // Increase the dimensions of the next segment to be drawn
        currBarX0 += 4;
        currBarY0 -= 1;
        currBarHeight += 1;

        // Find what color to make the next segment
        if (currBar < numSegmentsToFill) {  // If there are still more bars to fill with the gradient
            if (currBar <= 2) {                 // Increase the red value of segments [1-2] by D2
                currBarColor += 0x1000;
            } else if (currBar < 12) {          // Increase the red value of segments [3-12] by D3
                currBarColor += 0x1800;               
            } else if (currBar < 23) {          // Decrease the blue value of segments [13-23] by D5
                currBarColor -= 0x00A0;
            } else {                            // Decrease the blue value of segments [24-25] by D4
                currBarColor -= 0x0080;
            }
        } else {                            // Else if there are no more bars to fill with the gradient
            currBarColor = WHITE;               // Fill remaining bars with white
        }     
    }
} // End drawMagnitudeBar()




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   Increases or decreases sensitivity value and updates it cooresponding readout on the screen
 *@param:   row -> Which row of the input sensitivity menu to update (1=forward, 2=reverse, 3=left, 4=right)
 *@param:   upOrDown -> Whether to increase or decrease the data at row ('U' = increase, 'D' = decrease)
 *@param:	incAmount -> Amount to increment or decrement by
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateSensitivityReadout(uint8_t row, char upOrDown, uint8_t incAmount) {
    tft.setTextColor(BLACK);
    tft.setFont(&FreeMonoBold9pt7b);
    
	switch (row) {        
		// Update forward sensitivity readout
        case 1:   
            if (upOrDown == 'U' && forwardSensPercent != 100) {    	// If the data should be increased
                if (forwardSensPercent + incAmount > 100) {				// If increasing by incAmount will lead to value greater than 100%
					forwardSensPercent = 100;								// Set to 100%
				} else {												// Otherwise
					forwardSensPercent += incAmount;                        // Increase by incAmount%
				}
            } else if (upOrDown == 'D' && forwardSensPercent != 0) { // If the data should be decreased
                if (forwardSensPercent - incAmount > 100) {				// If decreasing by incAmount will lead to value less than 0% (exhibited by overflow to 65k)
					forwardSensPercent = 0;									// Set to 0%
				} else {												// Otherwise
					forwardSensPercent -= incAmount;                   		// Decrease by incAmount% 
				}
            }            
            drawMenuBox(L_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, L_MENU_LINE1_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(L_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, L_MENU_LINE1_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(forwardSensPercent);  // Print new data to the LCD        
            break;
            
        // Update reverse sensitivity readout
        case 2:         
            if (upOrDown == 'U' && reverseSensPercent != 100) {    	// If the data should be increased
                if (reverseSensPercent + incAmount > 100) {				// If increasing by incAmount will lead to value greater than 100%
					reverseSensPercent = 100;								// Set to 100%
				} else {												// Otherwise
					reverseSensPercent += incAmount;						// Increase by incAmount%
				}
            } else if (upOrDown == 'D' && reverseSensPercent != 0) { // If the data should be decreased
                if (reverseSensPercent - incAmount > 100) {				// If decreasing by incAmount will lead to value less than 0% (exhibited by overflow to 65k)
					reverseSensPercent = 0;									// Set to 0%
				} else {												// Otherwise
					reverseSensPercent -= incAmount;                   		// Decrease by incAmount% 
				}
            }            
            drawMenuBox(L_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, L_MENU_LINE2_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(L_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, L_MENU_LINE2_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(reverseSensPercent);  // Print new data to the LCD        
            break; 

        // Update left sensitivity readout
        case 3:      
            if (upOrDown == 'U' && leftSensPercent != 100) {       	// If the data should be increased
                if (leftSensPercent + incAmount > 100) {				// If increasing by incAmount will lead to value greater than 100%
					leftSensPercent = 100;									// Set to 100%
				} else {												// Otherwise
					leftSensPercent += incAmount;                        	// Increase by incAmount%
				}
            } else if (upOrDown == 'D' && leftSensPercent != 0) {    // If the data should be decreased
                if (leftSensPercent - incAmount > 100) {				// If decreasing by incAmount will lead to value less than 0% (exhibited by overflow to 65k)
					leftSensPercent = 0;									// Set to 0%
				} else {												// Otherwise
					leftSensPercent -= incAmount;                   		// Decrease by incAmount% 
				}
            }            
            drawMenuBox(L_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, L_MENU_LINE3_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(L_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, L_MENU_LINE3_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(leftSensPercent);  // Print new data to the LCD        
            break;

        // Update right sensitivity readout
        case 4:       
            if (upOrDown == 'U' && rightSensPercent != 100) {      	// If the data should be increased
                if (rightSensPercent + incAmount > 100) {				// If increasing by incAmount will lead to value greater than 100%
					rightSensPercent = 100;									// Set to 100%
				} else {												// Otherwise
					rightSensPercent += incAmount;							// Increase by incAmount%
				}
            } else if (upOrDown == 'D' && rightSensPercent != 0) {   // If the data should be decreased
                if (rightSensPercent - incAmount > 100) {				// If decreasing by incAmount will lead to value less than 0% (exhibited by overflow to 65k)
					rightSensPercent = 0;									// Set to 0%
				} else {												// Otherwise
					rightSensPercent -= incAmount;                   		// Decrease by incAmount% 
				}
            }            
            drawMenuBox(L_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, L_MENU_LINE4_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(L_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, L_MENU_LINE4_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(rightSensPercent);  // Print new data to the LCD        
            break;
    } // End Switch
	
	tft.fillRect(5, 298, 228, 27, WHITE);	// Clear the *SAVED* or *SAVING DISABLED* message area by filling with white rectangle
	
} // End updateSensitivityReadout()




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   Increases or decreases motion characteristic value and updates it cooresponding readout on the screen. The Magnitude bar of the chosen row is also 
 *          updated to reflect the new value.
 *@param:   row -> Which row of the motion characteristics menu to update (1=max speed, 2=acceleration rate, 3=deceleration rate)
 *@param:   upOrDown -> Whether to increase or decrease the data at row ('U' = increase, 'D' = decrease)
 *@param:	incAmount -> Amount to increment or decrement by
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateMotionCharReadout(uint8_t row, char upOrDown, uint8_t incAmount) {
    tft.setTextColor(BLACK);
    tft.setFont(&FreeMonoBold9pt7b);

    switch (row) {
        // Update maximum speed readout
        case 1:
            if (upOrDown == 'U' && speedLimitPercent != 100) {    	// If the data should be increased
                if (speedLimitPercent + incAmount > 100) {				// If increasing by incAmount will lead to value greater than 100%
					speedLimitPercent = 100;								// Set to 100%
				} else {												// Otherwise
					speedLimitPercent += incAmount;							// Increase by incAmount%
				}
            
			} else if (upOrDown == 'D' && speedLimitPercent != 0) { 	// If the data should be decreased
                if (speedLimitPercent - incAmount > 100) {				// If decreasing by incAmount will lead to value less than 0% (exhibited by overflow to 65k)
					speedLimitPercent = 0;									// Set to 0%
				} else {												// Otherwise
					speedLimitPercent -= incAmount;                   		// Decrease by incAmount% 
				}
            }            
            drawMenuBox(R_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, R_MENU_LINE1_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text                
            tft.setCursor(R_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, R_MENU_LINE1_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(speedLimitPercent);  // Print new data to the LCD    
            drawMagnitudeBar(R_MENU_LINES_X0 + MAG_BAR_BOX_X_OFFSET, R_MENU_LINE1_Y0, speedLimitPercent);    // Update the speedLimit magnitude bar    
            break;
            
        // Update acceleration rate readout
        case 2:       
            if (upOrDown == 'U' && accRatePercent != 100) {    	// If the data should be increased
                if (accRatePercent + incAmount > 100) {				// If increasing by incAmount will lead to value greater than 100%
					accRatePercent = 100;								// Set to 100%
				} else {											// Otherwise
					accRatePercent += incAmount;						// Increase by incAmount%
				}
            } else if (upOrDown == 'D' && accRatePercent != 0) { // If the data should be decreased
                if (accRatePercent - incAmount > 100) {				// If decreasing by incAmount will lead to value less than 0% (exhibited by overflow to 65k)
					accRatePercent = 0;									// Set to 0%
				} else {											// Otherwise
					accRatePercent -= incAmount;                   		// Decrease by incAmount% 
				}
            }            
            drawMenuBox(R_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, R_MENU_LINE2_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(R_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, R_MENU_LINE2_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(accRatePercent);  // Print new data to the LCD
            drawMagnitudeBar(R_MENU_LINES_X0 + MAG_BAR_BOX_X_OFFSET, R_MENU_LINE2_Y0, accRatePercent);    // Update the accRate magnitude bar        
            break; 

        // Update deceleration rate readout
        case 3:        
            if (upOrDown == 'U' && decRatePercent != 100) {			// If the data should be increased
                if (decRatePercent + incAmount > 100) {					// If increasing by incAmount will lead to value greater than 100%
					decRatePercent = 100;									// Set to 100%
				} else {												// Otherwise
					decRatePercent += incAmount;							// Increase by incAmount%
				}
            } else if (upOrDown == 'D' && decRatePercent != 0) {	// If the data should be decreased
                if (decRatePercent - incAmount > 100) {					// If decreasing by incAmount will lead to value less than 0% (exhibited by overflow to 65k)
					decRatePercent = 0;										// Set to 0%
				} else {												// Otherwise
					decRatePercent -= incAmount;                   			// Decrease by incAmount% 
				}
            }            
            drawMenuBox(R_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, R_MENU_LINE3_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(R_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, R_MENU_LINE3_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(decRatePercent);  // Print new data to the LCD 
            drawMagnitudeBar(R_MENU_LINES_X0 + MAG_BAR_BOX_X_OFFSET, R_MENU_LINE3_Y0, decRatePercent);    // Update the decRate magnitude bar       
            break;  
    } // End switch
	
	tft.fillRect(5, 298, 228, 27, WHITE);   // Clear the *SAVED* or *SAVING DISABLED* message area by filling with white rectangle
	
} // End updateMotionCharReadout()




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   ISR called when touch is detected. Finds the time at which the touch occured
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void touchDetected() {  
    timePressed = millis();   
} // End touchDetected()




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief:	Reads joystick inputs to determine if the car should rotate or drive forward
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getJoystickInputs() {
	// Convert the joystick axis voltages to directions
	uint16_t x_p = analogRead(JOYSTICK_X) > rightThresh ? 1 : 0;
	uint16_t x_n = analogRead(JOYSTICK_X) < leftThresh ? 1 : 0;
	uint16_t y_p = analogRead(JOYSTICK_Y) > forwardThresh ? 1 : 0;
	uint16_t y_n = analogRead(JOYSTICK_Y) < reverseThresh ? 1 : 0;
	
	////////////////////////////////////////////////
	// JOYSTICK INPUT DIAGRAM                     //
	//                                            //
	//                 + Y Input                  //
	//                   ____                     //
	//                  |    |                    //
	//                  |    |                    //
	//            ______|    |______              //
	//  -X Input |       /``\       |  + X Input  //
	//           |______ \__/ ______|             //
	//                  |    |                    //
	//                  |    |                    //
	//                  |____|                    //
	//                                            //
	//                 -Y Input                   //
	//                                            //
	////////////////////////////////////////////////
	
	// set x and y to either 0, 512, or 1023 based on the joystick values where 0 is negative input, 512 is neutral (no input), and 1023 is positive input
	// Truth Table for X, Y output (D/C = Don't Care, N/P = Not Possible)
	//  ____________________________________
	// | x_p | x_n | y_p | y_n ||  x  |  y  |
	// |  0  |  0  | D/C | D/C || 512 | D/C |
	// |  0  |  1  | D/C | D/C ||  0  | D/C |
	// |  1  |  0  | D/C | D/C || 1023| D/C |
	// |  1  |  1  | D/C | D/C || N/P | D/C |
	// | D/C | D/C |  0  |  0  || D/C | 512 |
	// | D/C | D/C |  0  |  1  || D/C |  0  |
	// | D/C | D/C |  1  |  0  || D/C | 1023|
	// | D/C | D/C |  1  |  1  || D/C | N/P |
	// ``````````````````````````````````````
	
	// Set x and y based on truth table above
	uint16_t x = x_p ? 1023 : (x_n ? 0 : 512);
	uint16_t y = y_p ? 1023 : (y_n ? 0 : 512);

	// Map speeds to within speed limit (y speeds i.e. rotation speed is mapped between a smaller boundry to reduce turn speed compared to forward/backward speed)
	x = map(x, 0, 1023, 512 - (speedLimit / TURN_SPEED_RATIO), 512 + (speedLimit / TURN_SPEED_RATIO));
	y = map(y, 0, 1023, 512 - speedLimit, 512 + speedLimit);
  
	int16_t moveValue = 0;        
	if (y > 512) {
		moveValue = y - 512;
	} else {
		moveValue = -(512 - y);
	}

	int16_t rotateValue = 0;
	if (x > 512) {
		rotateValue = x - 512;
	} else {
		rotateValue = -(512 - x);
	}
	
	// Set motor speeds to MIN_SPEED_LIMIT if target speed is below minimum
	if (rotateValue < -3 and rotateValue > -MIN_SPEED_LIMIT) {
		rotateValue = -MIN_SPEED_LIMIT;
	} else if (rotateValue > 3 and rotateValue < MIN_SPEED_LIMIT) {
		rotateValue = MIN_SPEED_LIMIT;
	}

	setLeftRightMotorSpeeds(moveValue, rotateValue);    
	delay(30); // Make loop run approximately 50hz  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief:  Get left and right motor speeds
 * @desc :  Convert moveValue and rotateValue to left and right motor speeds to send to the drive function
 * @param:  moveValue   -> Value indicating speed and direction of movement, + value indicates moving forward, - value indicates moving backwards
 *                         ranges from [512 - speedLimit, 512 + speedLimit]
 * @param:  rotateValue -> Value indicating speed and direction of rotation, + value indicates CW rotation, - value indicates CCW rotation
 *                         ranges from [512 - speedLimit, 512 + speedLimit]
 * @return: None
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// These variables are used to keep track of changes in direction in order to reset the speeds sent to the motor before allowing additional input
bool directionChanged = false;
bool leftResetComplete = true;
bool rightResetComplete = true;

void setLeftRightMotorSpeeds(int16_t moveValue, int16_t rotateValue) {
    static uint16_t prevLeftMotorSpeed = 0;      
    static uint16_t prevRightMotorSpeed = 0;
    
    uint16_t leftMotorSpeed = 0;
    uint16_t rightMotorSpeed = 0;
    
    // Driving forward
    if (moveValue > 0.0) {
        leftMotorSpeed = moveValue - FWD_LEFT_TRIM;
        rightMotorSpeed = moveValue - FWD_RIGHT_TRIM;
		
    
	// Driving backwards
	} else if (moveValue < 0.0) {
        leftMotorSpeed = moveValue + REV_LEFT_TRIM;
        rightMotorSpeed = moveValue + REV_RIGHT_TRIM;
    
	// Turning left
	} else if (rotateValue < 0.0) {
        leftMotorSpeed = rotateValue + REV_LEFT_TRIM;
        rightMotorSpeed = -rotateValue - FWD_RIGHT_TRIM; 	
    
	// Turning right
	} else if (rotateValue > 0.0) {
        leftMotorSpeed = rotateValue - FWD_LEFT_TRIM;
        rightMotorSpeed = -rotateValue + REV_RIGHT_TRIM;
    }  
	
    // Find out if a direction change occured, then store the previous speed values
	if (leftMotorSpeed != prevLeftMotorSpeed || rightMotorSpeed != prevRightMotorSpeed) {
		directionChanged = true;
		leftResetComplete = false;
		rightResetComplete = false;
	}
	
    prevLeftMotorSpeed = leftMotorSpeed;
    prevRightMotorSpeed = rightMotorSpeed; 
    
    //Drive the motors at the given speeds
    drive(leftMotorSpeed, rightMotorSpeed);

} // End setLeftRightMotorSpeeds




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief: Send info to motors to drive the vehicle
 * @desc:  Add or subtract nextLeftSpeed and nextRightSpeed pulse width offsets from default pulse widths to control speed and direction of each 
 *         motor. See table above for coorelation between pulse width and motor direction/speed. Acceleration is limited by a ramping constant.
 * @param: nextLeftSpeed  -> The pulse width offset representing the next speed the left motor should drive at
 * @param: nextRightSpeed -> The pulse width offset representing the next speed the right motor should drive at
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// These two values are used by the drive function to track the previous cycle speeds in order to ramp the speed up or down
int16_t prevLeft = 500;
int16_t prevRight = 500;
const uint16_t NEUTRAL_SPEED = 500;    // Speed value that represents motor neutral (no motion)

void drive(int16_t nextLeftSpeed, int16_t nextRightSpeed) {
    uint16_t leftMotorPulse = 0;       // Pulsewidth (in us) to send to left motor controller
    uint16_t rightMotorPulse = 0;      // Pulsewidth (in us) to send to right motor controller
	uint16_t leftDriveSpeed;
	uint16_t rightDriveSpeed;
    // If a direction change has occured, reset motor speeds to neutral before accepting new input from the joystick
    if (directionChanged) {	
        resetMotorSpeeds();
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LEFT MOTOR CONTROL
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // variable controls the next speed the left motor should drive at 
	
	leftDriveSpeed = map(nextLeftSpeed, -512, 512, 0, FORWARD_PULSE - REVERSE_PULSE);
	

    // If left motor is signaled to increase from its previous speed by at least RAMPING constant, increase its speed by RAMPING constant
    if (leftDriveSpeed > prevLeft + accRate) {
        if (prevLeft + accRate < FORWARD_PULSE - REVERSE_PULSE) {
            leftDriveSpeed = prevLeft + accRate;
        } else {
            leftDriveSpeed = FORWARD_PULSE - REVERSE_PULSE;
        }
    }
    
    // If left motor is signaled to decrease from its previous speed by at least RAMPING constant, decrease its speed by RAMPING constant 
    else if(leftDriveSpeed < prevLeft - accRate) {
        if (prevLeft - accRate > 0) {
            leftDriveSpeed = prevLeft - accRate;
        } else {
            leftDriveSpeed = 0;
        }
    }
	
    // Calculate pulse width to send to left motor controller       
    leftMotorPulse = REVERSE_PULSE + leftDriveSpeed;
    
    // Send PWM signal to the left motor (As long as DEBUG mode is not enabled)
    if(!DEBUG) {		
		leftMotor.writeMicroseconds(leftMotorPulse);
    }
    
    // Store previous speed value of left motor
    prevLeft = leftDriveSpeed;    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // RIGHT MOTOR CONTROL
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // variable controls the next speed the right motor should drive at
	rightDriveSpeed = map(nextRightSpeed, -512, 512, 0, FORWARD_PULSE - REVERSE_PULSE);
	

    // If right motor is signaled to increase from previous speed by at least RAMPING constant, increase speed by RAMPING constant  
    if(rightDriveSpeed > prevRight + accRate) {        
        if (rightDriveSpeed < FORWARD_PULSE - REVERSE_PULSE) {
            rightDriveSpeed = prevRight + accRate;
        } else {
            rightDriveSpeed = FORWARD_PULSE - REVERSE_PULSE;
        }        
    
	// If right motor is signaled to decrease from previous speed by at least RAMPING constant, decrease speed by RAMPING constant 
	} else if(rightDriveSpeed < prevRight - accRate) {        
        if (prevRight - accRate > 0) {
            rightDriveSpeed = prevRight - accRate;
        } else {
            rightDriveSpeed = 0;
        }
    }   

    // Calculate pulse width to send to right motor controller
    rightMotorPulse = REVERSE_PULSE + rightDriveSpeed;     

    // Send PWM signal to the right motor (As long as DEBUG mode is not enabled)
    if (!DEBUG) {    
		rightMotor.writeMicroseconds(rightMotorPulse);
    }
    // Store previous speed value of right motor
    prevRight = rightDriveSpeed;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
} // End drive()




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief: Bring left and right motors to a stop
 * @desc:  Similar to the drive function, but contains its own loop to allow the speed of both left and right motors to be completely reset to 
 *         neutral before accepting further inputs. This function is called when a direction change is signaled by the joystick inputs. This keeps 
 *         both motors in sync with eachother to avoid drifting of the vehicle
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void resetMotorSpeeds() {
    uint16_t leftResetSpeed = prevLeft;      // The current speed of the left motor while it is in its reset cycle
    uint16_t rightResetSpeed = prevRight;    // The current speed of the right motor while it is in its reset cycle
    
    // While direction changed flag is true, reset left and right motors to zero speed before allowing more speed input
    while (directionChanged) {     

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // RESET LEFT MOTOR
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
        if (leftResetSpeed > NEUTRAL_SPEED + decRate) {
            leftResetSpeed = prevLeft - decRate;        
		} else if (leftResetSpeed < NEUTRAL_SPEED - decRate) {
            leftResetSpeed = prevLeft + decRate;       
	    } else {
            leftResetComplete = true;   // Signal that the left motor has come to a stop
        }
		
        // Calculate pulse width to send to right motor controller
        uint16_t leftMotorPulse = 0;    
        leftMotorPulse = REVERSE_PULSE + leftResetSpeed;
		
        // Send PWM signal to the left motor (As long as DEBUG mode is not enabled)
        if(!DEBUG) {
            leftMotor.writeMicroseconds(leftMotorPulse);
        }
        // Store previous speed value of left motor
        prevLeft = leftResetSpeed;
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // RESET RIGHT MOTOR
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (rightResetSpeed > NEUTRAL_SPEED + decRate) {
            rightResetSpeed = prevRight - decRate;
        } else if (rightResetSpeed < NEUTRAL_SPEED - decRate) {
            rightResetSpeed = prevRight + decRate;
        } else {
            rightResetComplete = true;  // Signal that the right motor has come to a stop
        }
        // Calculate pulse width to send to right motor controller
        uint16_t rightMotorPulse = 0;    
        rightMotorPulse = REVERSE_PULSE + rightResetSpeed;
		
        // Send PWM signal to the right motor (As long as DEBUG mode is not enabled)
        if(!DEBUG) {
            rightMotor.writeMicroseconds(rightMotorPulse);
        }
        // Store previous speed value of left motor
        prevRight = rightResetSpeed;        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // If both left and right motors have come to a stop, signal that they are ready to receive new inputs
        if (leftResetComplete && rightResetComplete) {
            directionChanged = false;
        }
        delay(30);  // Make loop run at approximately 50Hz
    }
} // End resetMotorSpeeds()




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief Clears the serial buffer of garbage data
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void serialDump() {
	int dumpster;
	while (Serial.available()) {			
		dumpster = Serial.read();
	}
} // End serialDump()




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief: Print debug information to the serial monitor
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void debug(String s, int value){
  if(DEBUG){
    Serial.print(s);
    Serial.print(": ");
    Serial.println(value);
  }
} // End debug()


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief:	Activated when a serial input is detected. Used to facilitate navigation through the calibration modes
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void serialEvent() {
	if (CALIBRATION_MODE_ENABLE && calibrationStarted) {
		
		// Minimum speed limit calibration selected
		if (calibrationMode = "1") {
			uint8_t menuControl = Serial.read();
			serialDump();
			
			switch(menuControl) {	
					
				// Select COURSE increment type
				case 'c':
				case 'C':
					courseOrFine = COURSE;
					Serial.println("\n*COURSE Inc Selected*\n");
					break;
				
				// Select FINE increment type
				case 'f':
				case 'F':
					courseOrFine = FINE;
					Serial.println("\n*Fine Inc Selected*\n");
					break;
				
				// Increase min speedLimit
				case 'u':
				case 'U':
					if (courseOrFine == COURSE) {
						if (speedLimit + COURSE_SPEED_CALIBRATION_INC <= 512) {
							speedLimit += COURSE_SPEED_CALIBRATION_INC;
						}
					} else if (courseOrFine == FINE) {
						if (speedLimit + FINE_SPEED_CALIBRATION_INC <= 512) {
							speedLimit += FINE_SPEED_CALIBRATION_INC;
						}
					}	
					Serial.print(F("speedLimit: "));
					Serial.println(speedLimit);
					break;
				
				// Decrease min speedLimit
				case 'D':
				case 'd':
					if (courseOrFine == COURSE) {
						if (speedLimit - COURSE_SPEED_CALIBRATION_INC <= 512) {
							speedLimit -= COURSE_SPEED_CALIBRATION_INC;
						}
					} else if (courseOrFine == FINE) {
						if (speedLimit - FINE_SPEED_CALIBRATION_INC <= 512) {
							speedLimit -= FINE_SPEED_CALIBRATION_INC;
						}
					}								
					Serial.print(F("speedLimit: "));
					Serial.println(speedLimit);
					break;
					
				// Increase FWD_LEFT_TRIM
				case '1':
					FWD_LEFT_TRIM += 1;
					Serial.print(F("FWD_LEFT_TRIM: "));
					Serial.println(FWD_LEFT_TRIM);
					break;
				
				// Decrease FWD_LEFT_TRIM
				case '2':
					FWD_LEFT_TRIM -= 1;
					Serial.print(F("FWD_LEFT_TRIM: "));
					Serial.println(FWD_LEFT_TRIM);
					break;
					
				// Increase REV_LEFT_TRIM
				case '3':
					REV_LEFT_TRIM += 1;
					Serial.print(F("REV_LEFT_TRIM: "));
					Serial.println(REV_LEFT_TRIM);
					break;
				
				// Decrease REV_LEFT_TRIM
				case '4':
					REV_LEFT_TRIM -= 1;
					Serial.print(F("REV_LEFT_TRIM: "));
					Serial.println(REV_LEFT_TRIM);
					break;
					
				// Increase FWD_RIGHT_TRIM
				case '5':
					FWD_RIGHT_TRIM += 1;
					Serial.print(F("FWD_RIGHT_TRIM: "));
					Serial.println(FWD_RIGHT_TRIM);
					break;
				
				// Decrease FWD_RIGHT_TRIM
				case '6':
					FWD_RIGHT_TRIM -= 1;
					Serial.print(F("FWD_RIGHT_TRIM: "));
					Serial.println(FWD_RIGHT_TRIM);
					break;
					
				// Increase REV_RIGHT_TRIM
				case '7':
					REV_RIGHT_TRIM += 1;
					Serial.print(F("REV_RIGHT_TRIM: "));
					Serial.println(REV_RIGHT_TRIM);
					break;
				
				// Decrease REV_RIGHT_TRIM
				case '8':
					REV_RIGHT_TRIM -= 1;
					Serial.print(F("REV_RIGHT_TRIM: "));
					Serial.println(REV_RIGHT_TRIM);
					break;
				
				// Exit
				case 'e':
				case 'E':			
					Serial.print  ("\nReplace the value of 'MIN_SPEED_LIMIT' with ");
					Serial.println(speedLimit);
					Serial.print  ("Replace the value of FWD_LEFT_TRIM with ");
					Serial.println(FWD_LEFT_TRIM);
					Serial.print  ("Replace the value of REV_LEFT_TRIM with ");
					Serial.println(REV_LEFT_TRIM);
					Serial.print  ("Replace the value of FWD_RIGHT_TRIM with ");
					Serial.println(FWD_RIGHT_TRIM);
					Serial.print  ("Replace the value of REV_RIGHT_TRIM with ");
					Serial.println(REV_RIGHT_TRIM);
					Serial.println("");
					calibrationStarted = false;
					break;					
			}
			serialDump(); 
		}
	}
	
}
