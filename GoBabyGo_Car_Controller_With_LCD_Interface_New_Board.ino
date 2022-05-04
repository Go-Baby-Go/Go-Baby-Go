//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TITLE: GoBabyGo Car Controller - Simple 4 Direction
// Written by FIRST Team 1939 (The Kuhnigits)
// Modified by David Long
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
bool allow_EEPROM_writes = false;
//*****************************************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// TODO: Add function to choose between these values or the percents stored in EEPROM. If percents are desired, they must first be
//  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adjustable values: you can change these to fit the needs of each user                                                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                                                                                                    //
// Increase/decrease these values to change the joystick deadzones                                                                  //
// (An increase in the RIGHT or FORWARD values or a decrease in the LEFT or REVERSE values increases the corresponding deadzones)   //
const uint16_t RIGHT_THRESH = 600;      // Analog reads above this value should be considered right joystick inputs                 //
const uint16_t LEFT_THRESH = 400;       // Analog reads below this value should be considered left joystick inputs                  //
const uint16_t FORWARD_THRESH = 630;    // Analog reads above this value should be considered forward joystick inputs               //
const uint16_t REVERSE_THRESH = 340;    // Analog reads below this value should be considered reverse joystick inputs               //
                                                                                                                                    //
// Increase/decrease these values to modify the driving characteristics                                                             //
const uint16_t SPEED_LIMIT = 206;       // Between 100 - 512: This scales down all values sent to the motors                        //
                                        // Increase value to increase the speed of the car                                          //
                                                                                                                                    //
const uint8_t INCREASE_RAMPING = 5;     // This prevents the car from jerking by limiting how fast its speed can increase           //
                                        // If the car is not responding/accelerating fast enough, increase this value               //
                                                                                                                                    //
const uint8_t DECREASE_RAMPING = 15;    // This prevents the car from jerking by limiting how fast its speed can decrease           //
                                        // If the car is not slowing down fast enough, increase this value                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Set this value true to enable debug Over Serial (Make sure battery is disconnected from Arduino.
const boolean DEBUG = false;

// Set this value false if the car will not be used with the touchscreen LCD
const boolean USE_LCD = false;

                             
// Constants for Rev Robotics Spark Motor Controller
// | Full Reverse | Proportional Reverse |      Neutral      | Proportional Forward | Full Forward |
// |  p <= 1000   |   1000 < p < 1460    | 1460 <= p <= 1540 |   1540 < p < 2000    |  2000 <= p   |
const uint16_t REVERSE_PULSE = 1000;   // Default pulse width for full reverse is 1000us
const uint16_t FORWARD_PULSE = 2000;   // Default pulse width for full forward is 2000us


// Invert left or right motor                                                                                                       
// (If true, CW rotation as viewed from the front of the motor will be considered forward drive)
// (If false, CCW rotation as viewed from the front of the motor will be considered forward drive)
const boolean INVERT_LEFT_MOTOR = false;
const boolean INVERT_RIGHT_MOTOR = true;


// Pin Assignments
const int JOYSTICK_X = A7;       // Joystick X-input
const int JOYSTICK_Y = A6;       // Joystick Y-input
const int TFT_CS = A3;           // LCD chip select pin
const int TFT_DC = A2;           // LCD data control pin
const int TFT_RST = -1;          // Reset pins tied between LCD and Arduino
const int TOUCH_INT = 2;         // Touch screen interrupt pin
const int LEFT_MOTOR_PIN   = 9;  // digital pin 6 (PF4)
const int RIGHT_MOTOR_PIN  = 10; // digital pin 5 (PB2)


// LCD Colors
const uint16_t BLACK    = 0x0000;
const uint16_t WHITE    = 0xFFFF;
const uint16_t GREY     = 0xBDF7;
const uint16_t GREEN    = 0x07E0;
const uint16_t L_YELLOW = 0xEFB7;


// LCD Menu Elements
const uint8_t L_MENU_LINES_X0         = 5;    // X-coordinate of top left of all 4 input sensitivity menu lines
const uint8_t L_MENU_LINE1_Y0         = 65;   // Y-coordinate of top left of input sensitivity menu line 1 (forward sensitivity)
const uint8_t L_MENU_LINE2_Y0         = 113;  // Y-coordinate of top left of input sensitivity menu line 2 (reverse sensitivity)
const uint8_t L_MENU_LINE3_Y0         = 161;  // Y-coordinate of top left of input sensitivity menu line 3 (left sensitivity)
const uint8_t L_MENU_LINE4_Y0         = 209;  // Y-coordinate of top left of input sensitivity menu line 5 (right sensitivity)

const uint8_t R_MENU_LINES_X0         = 249;  // X-coordinate of top left of all 3 motion characteristics menu lines
const uint8_t R_MENU_LINE1_Y0         = 65;   // Y-coordinate of top left of motion characteristics menu line 1 (maximum speed)
const uint8_t R_MENU_LINE2_Y0         = 173;  // Y-coordinate of top left of motion characteristics menu line 2 (acceleration rate)
const uint16_t R_MENU_LINE3_Y0        = 281;  // Y-coordinate of top left of motion characteristics menu line 3 (deceleration rate)

const uint8_t MENU_HEADER_HEIGHT      = 51;   // Height (in pixels) of the header boxes for both the input sensitivity and motion characteristics menus
const uint8_t MENU_HEADER_WIDTH       = 227;  // Width (in pixels) of the header boxes for both the input sensitivity and motion characteristics menus

const uint8_t MENU_LINE_HEIGHT        = 31;   // Height (in pixels) of each box in the menu lines of both the input sensitivity and motion characteristics menus

const uint8_t SENS_TYPE_BOX_WIDTH     = 95;   // Width (in pixels) of the input sensitivity menu line sensitivity type text box (2nd column)
const uint8_t SENS_TYPE_BOX_X_OFFSET  = 40;   // Pixel offset in x-direction from top left corner of menu line to the top left of the sensitivity type textbox
const uint8_t SENS_TYPE_TEXT_X_OFFSET = 47;   // Pixel offset in x-direction from top left corner of menu line to the bottom left of the sensitivity type text
const uint8_t SENS_TYPE_TEXT_Y_OFFSET = 21;   // Pixel offset in y-direction from top left corner of menu line to the bottom left of the sensitivity type text

const uint8_t MAG_BAR_BOX_WIDTH       = 103;  // Width (in pixels) of the motion characteristics menu line magnitude bar bounding box (2nd Column)
const uint8_t MAG_BAR_BOX_X_OFFSET    = 36;   // Pixel offset in x-direction from top left corner of motion characteristics menu line to the magnitude bar bounding box

const uint8_t BUTTON_WIDTH            = 31;   // Width (in pixels) of the up and down buttons for both the input sensitivity and motion characteristics menus (1st and 3rd column)
const uint8_t UP_BUTTON_X_OFFSET      = 144;  // Pixel offset in x-direction from top left corner of the menu line to the top left corner of the up button (both menus)

const uint8_t READOUT_BOX_WIDTH       = 43;   // Width (in pixels) of the data readout textbox for both input sensitivity and motion characteristics menu lines (4th column)
const uint8_t READOUT_BOX_X_OFFSET    = 184;  // Pixel offset in x-direction from top left corner of menu line to the top left of the data readout box
const uint8_t READOUT_TEXT_X_OFFSET   = 189;  // Pixel offset in x-direction from top left corner of menu line to the bottom left of the data readout text
const uint8_t READOUT_TEXT_Y_OFFSET   = 21;   // Pixel offset in y-direction from top left corner of menu line to the bottom left of the data readout text

const uint16_t SAVE_BOX_Y0            = 261;  // Y_coordinate of top left of save values button


// Input sensitivity menu button touch screen extents
const uint16_t L_BUTTON_R1_TOUCH_X1 = 1000;  // X value of touchscreen corresponding to top left corner of buttons in row 1 of the left menu screen (buttons for forward sensitivity row)
const uint16_t L_BUTTON_R1_TOUCH_X2 = 1300;  // X value of touchscreen corresponding to bottom right corner of buttons in row 1 of the left menu screen (buttons for forward sensitivity row)

const uint16_t L_BUTTON_R2_TOUCH_X1 = 1550;  // X value of touchscreen corresponding to top left corner of buttons in row 2 of the left menu screen (buttons for reverse sensitivity row)
const uint16_t L_BUTTON_R2_TOUCH_X2 = 1850;  // X value of touchscreen corresponding to bottom right corner of buttons in row 2 of the left menu screen (buttons for reverse sensitivity row)

const uint16_t L_BUTTON_R3_TOUCH_X1 = 2100;  // X value of touchscreen corresponding to top left corner of buttons in row 3 of the left menu screen (buttons for left sensitivity row)
const uint16_t L_BUTTON_R3_TOUCH_X2 = 2400;  // X value of touchscreen corresponding to bottom right corner of buttons in row 3 of the left menu screen (buttons for left sensitivity row)

const uint16_t L_BUTTON_R4_TOUCH_X1 = 2650;  // X value of touchscreen corresponding to top left corner of buttons in row 4 of the left menu screen (buttons for right sensitivity row)
const uint16_t L_BUTTON_R4_TOUCH_X2 = 2950;  // X value of touchscreen corresponding to bottom right corner of buttons in row 4 of the left menu screen (buttons for right sensitivity row)

const uint16_t L_BUTTON_C1_TOUCH_Y1 = 3870;  // Y value of touchscreen corresponding to top left corner of buttons in column 1 of the left menu screen (up buttons for input sensitivity menu)
const uint16_t L_BUTTON_C1_TOUCH_Y2 = 3650;  // Y value of touchscreen corresponding to bottom right corner of buttons in column 1 of the left menu screen (up buttons for input sensitivity menu)

const uint16_t L_BUTTON_C2_TOUCH_Y1 = 2800;  // Y value of touchscreen corresponding to top left corner of buttons in column 2 of the left menu screen (down buttons for input sensitivity menu)
const uint16_t L_BUTTON_C2_TOUCH_Y2 = 2550;  // Y value of touchscreen corresponding to bottom right corner of buttons in column 2 of the left menu screen (down buttons for input sensitivity menu)


// Motion characteristics menu button touch screen extents
const uint16_t R_BUTTON_R1_TOUCH_X1 = 1000;  // X value of touchscreen corresponding to top left corner of buttons in row 1 of the right menu screen (buttons for maximum speed row)
const uint16_t R_BUTTON_R1_TOUCH_X2 = 1300;  // X value of touchscreen corresponding to bottom right corner of buttons in row 1 of the right menu screen (buttons for maximum speed row)

const uint16_t R_BUTTON_R2_TOUCH_X1 = 2200;  // X value of touchscreen corresponding to top left corner of buttons in row 2 of the right menu screen (buttons for acceleration rate row)
const uint16_t R_BUTTON_R2_TOUCH_X2 = 2550;  // X value of touchscreen corresponding to bottom right corner of buttons in row 2 of the right menu screen (buttons for acceleration rate row)

const uint16_t R_BUTTON_R3_TOUCH_X1 = 3400;  // X value of touchscreen corresponding to top left corner of buttons in row 1 of the right menu screen (buttons for deceleration rate row)
const uint16_t R_BUTTON_R3_TOUCH_X2 = 3700;  // X value of touchscreen corresponding to bottom right corner of buttons in row 1 of the right menu screen (buttons for deceleration rate row)

const uint16_t R_BUTTON_C1_TOUCH_Y1 = 2000;  // Y value of touchscreen corresponding to top left corner of buttons in column 1 of the right menu screen (down buttons for motion characteristics menu)
const uint16_t R_BUTTON_C1_TOUCH_Y2 = 1770;  // Y value of touchscreen corresponding to bottom right corner of buttons in column 1 of the right menu screen (down buttons for motion characteristics menu)

const uint16_t R_BUTTON_C2_TOUCH_Y1 = 860;  // Y value of touchscreen corresponding to top left corner of buttons in column 2 of the right menu screen (up buttons for motion characteristics menu)
const uint16_t R_BUTTON_C2_TOUCH_Y2 = 650;  // Y value of touchscreen corresponding to bottom right corner of buttons in column 2 of the right menu screen (up buttons for motion characteristics menu)


// Save values button touch screen extents
const uint16_t SAVE_VAL_TOUCH_X1 = 3200;
const uint16_t SAVE_VAL_TOUCH_X2 = 3500;
const uint16_t SAVE_VAL_TOUCH_Y1 = 3870;
const uint16_t SAVE_VAL_TOUCH_Y2 = 2190;


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

uint8_t forwardSens;
uint8_t reverseSens;
uint8_t leftSens;
uint8_t rightSens;
uint8_t maxSpeed;
uint8_t accRate;
uint8_t decRate;

// EEPROM Memory Addresses
uint16_t FORWARD_SENS_ADDRESS = 1;
uint16_t REVERSE_SENS_ADDRESS = 2;
uint16_t LEFT_SENS_ADDRESS = 3;
uint16_t RIGHT_SENS_ADDRESS = 4;
uint16_t MAX_SPEED_ADDRESS = 5;
uint16_t ACC_RATE_ADDRESS = 6;
uint16_t DEC_RATE_ADDRESS = 7;


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
    
    attachInterrupt(digitalPinToInterrupt(TOUCH_INT), touchDetected, FALLING);
    
    leftMotor.attach(LEFT_MOTOR_PIN);
    rightMotor.attach(RIGHT_MOTOR_PIN);

    
    // If the LCD should be used
    if (USE_LCD) {
        // Read in the sensitivity and motion characteristic values from the Arduino EEPROM        
        uint8_t forwardSens = EEPROM.read(FORWARD_SENS_ADDRESS);
        uint8_t reverseSens = EEPROM.read(REVERSE_SENS_ADDRESS);
        uint8_t leftSens    = EEPROM.read(LEFT_SENS_ADDRESS);
        uint8_t rightSens   = EEPROM.read(RIGHT_SENS_ADDRESS);
        uint8_t maxSpeed    = EEPROM.read(MAX_SPEED_ADDRESS);
        uint8_t accRate     = EEPROM.read(ACC_RATE_ADDRESS);
        uint8_t decRate     = EEPROM.read(DEC_RATE_ADDRESS);

        // If any of the values read from the EEPROM were >100, set them to 100. This is useful for the first program run after installing a new Arduino
        // where the EEPROM may not have been written to before (Data defaults to 255 at every unwritten EEPROM address)
        if (forwardSens > 100) forwardSens = 100;
        if (reverseSens > 100) reverseSens = 100;
        if (leftSens > 100) leftSens = 100;
        if (rightSens > 100) rightSens = 100;
        if (maxSpeed > 100) maxSpeed = 100;
        if (accRate > 100) accRate = 100;
        if (decRate > 100) decRate = 100;

        // Wait for Serial connection to be made
        Serial.begin(9600);
        while (!Serial) delay(10);

        // Wait for touchscreen to connect
        if (!touch.begin()) {
            while (1) delay(10);
        }

        ///////////////////////////////////////////////////////////////////////////////////////
        // Draw Initial Screen Layout
        ///////////////////////////////////////////////////////////////////////////////////////
        tft.begin();
        tft.setRotation(1);         // Rotate screen to landscape mode
        tft.fillScreen(WHITE);      // Set screen background to white
        tft.setTextColor(BLACK);    // Set default text color to black
    
        /////////////////////////////////
        // Input Sensitivity (%) Header
        /////////////////////////////////
        drawMenuBox(5, 5, MENU_HEADER_WIDTH, MENU_HEADER_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(81, 23);
        tft.println("Input");
        tft.setCursor(23, 46);
        tft.println("Sensitivity(%)");
    
        /////////////////////////////////
        // Forward Sensitivity Menu Line
        /////////////////////////////////
        drawLeftMenuLine(L_MENU_LINES_X0, L_MENU_LINE1_Y0, BLACK, GREY, "Forward", forwardSens);
    
        /////////////////////////////////
        // Reverse Sensitivity Menu Line
        /////////////////////////////////
        drawLeftMenuLine(L_MENU_LINES_X0, L_MENU_LINE2_Y0, BLACK, GREY, "Reverse", reverseSens);
    
        /////////////////////////////////
        // Left Sensitivity Menu Line
        /////////////////////////////////
        drawLeftMenuLine(L_MENU_LINES_X0, L_MENU_LINE3_Y0, BLACK, GREY, "Left", leftSens);
    
        /////////////////////////////////
        // Right Sensitivity Menu Line
        /////////////////////////////////
        drawLeftMenuLine(L_MENU_LINES_X0, L_MENU_LINE4_Y0, BLACK, GREY, "Right", rightSens);
    
        /////////////////////////////////
        // Save Values Button
        /////////////////////////////////
        drawMenuBox(L_MENU_LINES_X0, SAVE_BOX_Y0, MENU_HEADER_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(41, 284);
        tft.println("Save Values");   
    
        //////////////////////////////////////////////////
        // Maximum Speed (%) Header, Control, and Readout
        //////////////////////////////////////////////////
        drawMenuBox(249, 5, MENU_HEADER_WIDTH, MENU_HEADER_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(273, 23);
        tft.println("Maximum Speed");
        tft.setCursor(340, 46);
        tft.println("(%)");   
        drawRightMenuLine(R_MENU_LINES_X0, R_MENU_LINE1_Y0, BLACK, GREY, maxSpeed);
    
        /////////////////////////////////////////////////////
        // Acceleration Rate (%) Header, Control, and Readout
        /////////////////////////////////////////////////////
        drawMenuBox(249, 113, MENU_HEADER_WIDTH, MENU_HEADER_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(281, 131);
        tft.println("Acceleration");
        tft.setCursor(322, 153);
        tft.println("Rate(%)");   
        drawRightMenuLine(R_MENU_LINES_X0, R_MENU_LINE2_Y0, BLACK, GREY, accRate);
    
        /////////////////////////////////////////////////////
        // Deceleration Rate (%) Header, Control, and Readout
        /////////////////////////////////////////////////////
        drawMenuBox(249, 221, MENU_HEADER_WIDTH, MENU_HEADER_HEIGHT, BLACK, GREY);
        tft.setFont(&FreeMonoBold12pt7b);
        tft.setCursor(281, 239);
        tft.println("Deceleration");
        tft.setCursor(322, 262);
        tft.println("Rate(%)");   
        drawRightMenuLine(R_MENU_LINES_X0, R_MENU_LINE3_Y0, BLACK, GREY, decRate);
        delay(100);
        ///////////////////////////////////////////////////////////////////////////////////////
    } // End if
} // End setup()




void loop() {
    if (USE_LCD) {
        // TODO: May need to add flag that makes this section run only until user presses a button on the touch screen
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Touchscreen Interaction
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // If the touchDetected ISR was triggered and the appropriate ammount of time has passed since the last input, register a new touch input
        if (timePressed > prevTimePressed + CONSECUTIVE_TOUCH_DELAY) {
            touch.read_touch(&touch_x, &touch_y, &touch_z1, &touch_z2);
            prevTimePressed = timePressed;
            
            // Determine which (if any) button was pressed
            if (touch_y < L_BUTTON_C1_TOUCH_Y1 && touch_y > L_BUTTON_C1_TOUCH_Y2) {             // A decrease button was pressed in the input sensitivity menu           
                if (touch_x > L_BUTTON_R1_TOUCH_X1 && touch_x < L_BUTTON_R1_TOUCH_X2) {             // Decrease forward sensitivity
                    updateSensitivityReadout(1, 'D');
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE1_Y0, 'D', BLACK, L_YELLOW);             // Flash button
                    delay(100);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE1_Y0, 'D', BLACK, GREY);         
                           
                } else if (touch_x > L_BUTTON_R2_TOUCH_X1 && touch_x < L_BUTTON_R2_TOUCH_X2) {      // Decrease reverse sensitivity
                    updateSensitivityReadout(2, 'D');
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE2_Y0, 'D', BLACK, L_YELLOW);             // Flash button
                    delay(100);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE2_Y0, 'D', BLACK, GREY); 
                    
                } else if (touch_x > L_BUTTON_R3_TOUCH_X1 && touch_x < L_BUTTON_R3_TOUCH_X2) {      // Decrease left sensitivity
                    updateSensitivityReadout(3, 'D');
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE3_Y0, 'D', BLACK, L_YELLOW);             // Flash button
                    delay(100);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE3_Y0, 'D', BLACK, GREY); 
                    
                } else if (touch_x > L_BUTTON_R4_TOUCH_X1 && touch_x < L_BUTTON_R4_TOUCH_X2) {      // Decrease right sensitivity
                    updateSensitivityReadout(4, 'D');
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE4_Y0, 'D', BLACK, L_YELLOW);             // Flash button
                    delay(100);
                    drawButton(L_MENU_LINES_X0, L_MENU_LINE4_Y0, 'D', BLACK, GREY); 
                } 
                tft.fillRect(5, 293, 228, 27, WHITE);   // Clear the *SAVED* or *SAVING DISABLED* message area by filling with white rectangle
                
            } else if (touch_y < L_BUTTON_C2_TOUCH_Y1 && touch_y > L_BUTTON_C2_TOUCH_Y2) {                  // An increase button was pressed in the input sensitivity menu
                if (touch_x > L_BUTTON_R1_TOUCH_X1 && touch_x < L_BUTTON_R1_TOUCH_X2) {                         // Increase forward sensitivity
                    updateSensitivityReadout(1, 'U');
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE1_Y0, 'U', BLACK, L_YELLOW);    // Flash button
                    delay(100);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE1_Y0, 'U', BLACK, GREY); 
                    
                } else if (touch_x > L_BUTTON_R2_TOUCH_X1 && touch_x < L_BUTTON_R2_TOUCH_X2) {                  // Increase reverse sensitivity
                    updateSensitivityReadout(2, 'U');
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE2_Y0, 'U', BLACK, L_YELLOW);    // Flash button
                    delay(100);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE2_Y0, 'U', BLACK, GREY);
                    
                } else if (touch_x > L_BUTTON_R3_TOUCH_X1 && touch_x < L_BUTTON_R3_TOUCH_X2) {                  // Increase left sensitivity
                    updateSensitivityReadout(3, 'U');
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE3_Y0, 'U', BLACK, L_YELLOW);    // Flash button
                    delay(100);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE3_Y0, 'U', BLACK, GREY);
                    
                } else if (touch_x > L_BUTTON_R4_TOUCH_X1 && touch_x < L_BUTTON_R4_TOUCH_X2) {                  // Increase right sensitivity
                    updateSensitivityReadout(4, 'U');
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE4_Y0, 'U', BLACK, L_YELLOW);    // Flash button
                    delay(100);
                    drawButton(L_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, L_MENU_LINE4_Y0, 'U', BLACK, GREY);                
                }
                tft.fillRect(5, 293, 228, 27, WHITE);   // Clear the *SAVED* or *SAVING DISABLED* message area by filling with white rectangle
                
            } else if (touch_y < R_BUTTON_C1_TOUCH_Y1 && touch_y > R_BUTTON_C1_TOUCH_Y2) {      // A decrease button was pressed in the motion characteristics menu           
                if (touch_x > R_BUTTON_R1_TOUCH_X1 && touch_x < R_BUTTON_R1_TOUCH_X2) {             // Decrease maximum speed
                    updateMotionCharReadout(1, 'D');
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE1_Y0, 'D', BLACK, L_YELLOW);             // Flash button
                    delay(100);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE1_Y0, 'D', BLACK, GREY);
                    
                } else if (touch_x > R_BUTTON_R2_TOUCH_X1 && touch_x < R_BUTTON_R2_TOUCH_X2) {      // Decrease acceleration rate
                    updateMotionCharReadout(2, 'D');
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE2_Y0, 'D', BLACK, L_YELLOW);             // Flash button
                    delay(100);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE2_Y0, 'D', BLACK, GREY);
                    
                } else if (touch_x > R_BUTTON_R3_TOUCH_X1 && touch_x < R_BUTTON_R3_TOUCH_X2) {      // Decrease deceleration rate
                    updateMotionCharReadout(3, 'D');
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE3_Y0, 'D', BLACK, L_YELLOW);             // Flash button
                    delay(100);
                    drawButton(R_MENU_LINES_X0, R_MENU_LINE3_Y0, 'D', BLACK, GREY);
                } 
                tft.fillRect(5, 293, 228, 27, WHITE);   // Clear the *SAVED* or *SAVING DISABLED* message area by filling with white rectangle
                
            } else if (touch_y < R_BUTTON_C2_TOUCH_Y1 && touch_y > R_BUTTON_C2_TOUCH_Y2) {                      // An increase button was pressed in the motion characteristics menu
                if (touch_x > R_BUTTON_R1_TOUCH_X1 && touch_x < R_BUTTON_R1_TOUCH_X2) {                             // Increase maximum speed
                    updateMotionCharReadout(1, 'U');
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE1_Y0, 'U', BLACK, L_YELLOW);        // Flash button
                    delay(100);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE1_Y0, 'U', BLACK, GREY);
                    
                } else if (touch_x > R_BUTTON_R2_TOUCH_X1 && touch_x < R_BUTTON_R2_TOUCH_X2) {                  // Increase acceleration rate
                    updateMotionCharReadout(2, 'U');
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE2_Y0, 'U', BLACK, L_YELLOW);        // Flash button
                    delay(100);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE2_Y0, 'U', BLACK, GREY);
                    
                } else if (touch_x > R_BUTTON_R3_TOUCH_X1 && touch_x < R_BUTTON_R3_TOUCH_X2) {                  // Increase deceleration rate
                    updateMotionCharReadout(3, 'U');
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE3_Y0, 'U', BLACK, L_YELLOW);        // Flash button
                    delay(100);
                    drawButton(R_MENU_LINES_X0 + UP_BUTTON_X_OFFSET, R_MENU_LINE3_Y0, 'U', BLACK, GREY);
                } 
                tft.fillRect(5, 293, 228, 27, WHITE);   // Clear the *SAVED* or *SAVING DISABLED* message area by filling with white rectangle
            }
            
            if (touch_x > SAVE_VAL_TOUCH_X1 && touch_x < SAVE_VAL_TOUCH_X2 && touch_y < SAVE_VAL_TOUCH_Y1 && touch_y > SAVE_VAL_TOUCH_Y2) {     // The save values button was pressed           
                // Flash the touch box momentarily
                drawMenuBox(L_MENU_LINES_X0, SAVE_BOX_Y0, MENU_HEADER_WIDTH, MENU_LINE_HEIGHT, BLACK, L_YELLOW);
                tft.setFont(&FreeMonoBold12pt7b);
                tft.setCursor(44, 284);
                tft.println("Save Values");
                delay(150);
                drawMenuBox(L_MENU_LINES_X0, SAVE_BOX_Y0, MENU_HEADER_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);
                tft.setCursor(44, 284);
                tft.println("Save Values");    
    
                // Save values to EEPROM
                // *CAREFUL* The EEPROM has a limited lifespan of 100,000 writes. Do not overuse this function! 
                if (allow_EEPROM_writes) {
                    tft.setFont(&FreeMonoBold9pt7b);
                    tft.setCursor(78, 308);
                    tft.println("*SAVED*");
                    EEPROM.update(FORWARD_SENS_ADDRESS, forwardSens);
                    EEPROM.update(REVERSE_SENS_ADDRESS, reverseSens);
                    EEPROM.update(LEFT_SENS_ADDRESS, leftSens);
                    EEPROM.update(RIGHT_SENS_ADDRESS, rightSens);
                    EEPROM.update(MAX_SPEED_ADDRESS, maxSpeed);
                    EEPROM.update(ACC_RATE_ADDRESS, accRate);
                    EEPROM.update(DEC_RATE_ADDRESS, decRate);
                } else {
                    tft.setFont(&FreeMonoBold9pt7b);
                    tft.setCursor(25, 308);
                    tft.println("*SAVING DISABLED*");
                }
            }
        } 
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    } // End if(USE_LCD)

    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Vehicle operation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // Convert the joystick axis voltages to directions
    int x_p = analogRead(JOYSTICK_X) > RIGHT_THRESH ? 1 : 0;
    int x_n = analogRead(JOYSTICK_X) < LEFT_THRESH ? 1 : 0;
    int y_p = analogRead(JOYSTICK_Y) > FORWARD_THRESH ? 1 : 0;
    int y_n = analogRead(JOYSTICK_Y) < REVERSE_THRESH ? 1 : 0;
    
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
    int x = x_p ? 1023 : (x_n ? 0 : 512);
    int y = y_p ? 1023 : (y_n ? 0 : 512);

    // TODO: May be able to remove map function here
    // Map speeds to within speed limit
    x = map(x, 0, 1023, 512 - SPEED_LIMIT, 512 + SPEED_LIMIT);
    y = map(y, 0, 1023, 512 - SPEED_LIMIT, 512 + SPEED_LIMIT);
  
    int moveValue = 0;        
    if (y > 512) {
        moveValue = y - 512;
    } else {
        moveValue = -(512 - y);
    }

    int rotateValue = 0;
    if (x > 512) {
        rotateValue = x - 512;
    } else {
        rotateValue = -(512 - x);
    }

    setLeftRightMotorSpeeds(moveValue, rotateValue);    
    delay(30); // Make loop run approximately 50hz  
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
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
    tft.setCursor(x0 + SENS_TYPE_TEXT_X_OFFSET, y0 + SENS_TYPE_TEXT_Y_OFFSET);                                          // Set cursor to proper position in data type textBox
    tft.println(menuDataType);                                                                                          // Print menuDataType string to screen
            
    drawButton(x0 + UP_BUTTON_X_OFFSET, y0, 'U', border_color, fill_color);                                         // Draw increase button 
    
    drawMenuBox(x0 + READOUT_BOX_X_OFFSET, y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, border_color, fill_color);      // Draw value readout textBox
    tft.setCursor(x0 + READOUT_TEXT_X_OFFSET, y0 + READOUT_TEXT_Y_OFFSET);                                              // Set cursor to proper position in data readout textBox
    tft.println(String(menuDataVal));                                                                                   // Convert menuDataVal to string and print to screen 
    
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
    tft.setCursor(x0 + READOUT_TEXT_X_OFFSET, y0 + READOUT_TEXT_Y_OFFSET);                                          // Set cursor to proper position in data readout textBox
    tft.println(String(menuDataVal));                                                                               // Print menuDataVal string to screen 
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
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateSensitivityReadout(uint8_t row, char upOrDown) {
    tft.setTextColor(BLACK);
    tft.setFont(&FreeMonoBold9pt7b);
    switch (row) {
        // Update forward sensitivity readout
        case 1:   
            if (upOrDown == 'U' && forwardSens != 100) {    // If the data should be increased
                forwardSens += 1;                           // Increase by 1% with maximum of 100%
            }
            else if (upOrDown == 'D' && forwardSens != 0) { // If the data should be decreased
                forwardSens -= 1;                           // Decrease by 1% with minimum of 0%
            }            
            drawMenuBox(L_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, L_MENU_LINE1_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(L_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, L_MENU_LINE1_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(forwardSens);  // Print new data to the LCD        
            break;
            
        // Update reverse sensitivity readout
        case 2:         
            if (upOrDown == 'U' && reverseSens != 100) {    // If the data should be increased
                reverseSens += 1;                           // Increase by 1% with maximum of 100%
            }
            else if (upOrDown == 'D' && reverseSens != 0) { // If the data should be decreased
                reverseSens -= 1;                           // Decrease by 1% with minimum of 0%
            }            
            drawMenuBox(L_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, L_MENU_LINE2_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(L_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, L_MENU_LINE2_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(reverseSens);  // Print new data to the LCD        
            break; 

        // Update left sensitivity readout
        case 3:      
            if (upOrDown == 'U' && leftSens != 100) {       // If the data should be increased
                leftSens += 1;                              // Increase by 1% with maximum of 100%
            }
            else if (upOrDown == 'D' && leftSens != 0) {    // If the data should be decreased
                leftSens -= 1;                              // Decrease by 1% with minimum of 0%
            }            
            drawMenuBox(L_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, L_MENU_LINE3_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(L_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, L_MENU_LINE3_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(leftSens);  // Print new data to the LCD        
            break;

        // Update right sensitivity readout
        case 4:       
            if (upOrDown == 'U' && rightSens != 100) {      // If the data should be increased
                rightSens += 1;                             // Increase by 1% with maximum of 100%
            }
            else if (upOrDown == 'D' && rightSens != 0) {   // If the data should be decreased
                rightSens -= 1;                             // Decrease by 1% with minimum of 0%
            }            
            drawMenuBox(L_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, L_MENU_LINE4_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(L_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, L_MENU_LINE4_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(rightSens);  // Print new data to the LCD        
            break;
    } // End Switch
} // End updateSensitivityReadout()




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   Increases or decreases motion characteristic value and updates it cooresponding readout on the screen. The Magnitude bar of the chosen row is also 
 *          updated to reflect the new value.
 *@param:   row -> Which row of the motion characteristics menu to update (1=max speed, 2=acceleration rate, 3=deceleration rate)
 *@param:   upOrDown -> Whether to increase or decrease the data at row ('U' = increase, 'D' = decrease)
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateMotionCharReadout(uint8_t row, char upOrDown) {
    tft.setTextColor(BLACK);
    tft.setFont(&FreeMonoBold9pt7b);

    switch (row) {
        // Update maximum speed readout
        case 1:
            if (upOrDown == 'U' && maxSpeed != 100) {    // If the data should be increased
                maxSpeed += 1;                           // Increase by 1% with maximum of 100%
            }
            else if (upOrDown == 'D' && maxSpeed != 0) { // If the data should be decreased
                maxSpeed -= 1;                           // Decrease by 1% with minimum of 0%
            }            
            drawMenuBox(R_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, R_MENU_LINE1_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text                
            tft.setCursor(R_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, R_MENU_LINE1_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(maxSpeed);  // Print new data to the LCD    
            drawMagnitudeBar(R_MENU_LINES_X0 + MAG_BAR_BOX_X_OFFSET, R_MENU_LINE1_Y0, maxSpeed);    // Update the maxSpeed magnitude bar    
            break;
            
        // Update acceleration rate readout
        case 2:       
            if (upOrDown == 'U' && accRate != 100) {    // If the data should be increased
                accRate += 1;                           // Increase by 1% with maximum of 100%
            }
            else if (upOrDown == 'D' && accRate != 0) { // If the data should be decreased
                accRate -= 1;                           // Decrease by 1% with minimum of 0%
            }            
            drawMenuBox(R_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, R_MENU_LINE2_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(R_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, R_MENU_LINE2_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(accRate);  // Print new data to the LCD
            drawMagnitudeBar(R_MENU_LINES_X0 + MAG_BAR_BOX_X_OFFSET, R_MENU_LINE2_Y0, accRate);    // Update the maxSpeed magnitude bar        
            break; 

        // Update deceleration rate readout
        case 3:        
            if (upOrDown == 'U' && decRate != 100) {       // If the data should be increased
                decRate += 1;                              // Increase by 1% with maximum of 100%
            }
            else if (upOrDown == 'D' && decRate != 0) {    // If the data should be decreased
                decRate -= 1;                              // Decrease by 1% with minimum of 0%
            }            
            drawMenuBox(R_MENU_LINES_X0 + READOUT_BOX_X_OFFSET, R_MENU_LINE3_Y0, READOUT_BOX_WIDTH, MENU_LINE_HEIGHT, BLACK, GREY);  // Clear text
            tft.setCursor(R_MENU_LINES_X0 + READOUT_TEXT_X_OFFSET, R_MENU_LINE3_Y0 + READOUT_TEXT_Y_OFFSET);     // Set curser to correct location
            tft.println(decRate);  // Print new data to the LCD 
            drawMagnitudeBar(R_MENU_LINES_X0 + MAG_BAR_BOX_X_OFFSET, R_MENU_LINE3_Y0, decRate);    // Update the maxSpeed magnitude bar       
            break;  
    } // End switch
} // End updateMotionCharReadout()




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*@brief:   ISR called when touch is detected. Finds the time at which the touch occured
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void touchDetected() {  
    timePressed = millis();   
} // End touchDetected()




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief:  Get left and right motor speeds
 * @desc :  Convert moveValue and rotateValue to left and right motor speeds to send to the drive function
 * @param:  moveValue   -> Value indicating speed and direction of movement, + value indicates moving forward, - value indicates moving backwards
 *                         ranges from [512 - SPEED_LIMIT, 512 + SPEED_LIMIT]
 * @param:  rotateValue -> Value indicating speed and direction of rotation, + value indicates CW rotation, - value indicates CCW rotation
 *                         ranges from [512 - SPEED_LIMIT, 512 + SPEED_LIMIT]
 * @return: None
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: Try making these values statics (if they're only changed by the setLeftRightMotorSpeeds function)
// These variables are used to keep track of changes in direction in order to reset the speeds sent to the motor before allowing additional input
int prevLeftMotorSpeed = 0;      
int prevRightMotorSpeed = 0;
bool directionChanged = false;
bool leftResetComplete = true;
bool rightResetComplete = true;

void setLeftRightMotorSpeeds(int moveValue, int rotateValue) {
    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;
    
    // Driving forward
    if (moveValue > 0.0) {
        leftMotorSpeed = moveValue;
        rightMotorSpeed = moveValue;
    } 
    // Driving backwards
    else if (moveValue < 0.0) {
        leftMotorSpeed = moveValue;
        rightMotorSpeed = moveValue;
    }
    // Turning left
    else if (rotateValue < 0.0) {
        leftMotorSpeed = rotateValue;
        rightMotorSpeed = -rotateValue;        
    }
    // Turning right
    else if (rotateValue > 0.0) {
        leftMotorSpeed = rotateValue;
        rightMotorSpeed = -rotateValue;
    }
    
    // Find out if a direction change occured, then store the previous speed values
    if (leftMotorSpeed != prevLeftMotorSpeed || rightMotorSpeed != prevRightMotorSpeed) {
        directionChanged = true;
        leftResetComplete = false;
        rightResetComplete = false;
    }
    prevLeftMotorSpeed = leftMotorSpeed;
    prevRightMotorSpeed = rightMotorSpeed;    

    // Remap Motor Speed values to range [0, 1023] (Negative values map to [0, 512], positive values map to [513, 1023])
    int remapLeftMotorSpeed = map(leftMotorSpeed, -512, 512, 0, 1023);
    int remapRightMotorSpeed = map(rightMotorSpeed, -512, 512, 0, 1023);
    
    // Drive the motors at the given speeds
    drive(remapLeftMotorSpeed, remapRightMotorSpeed);

} // End setLeftRightMotorSpeeds




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* @brief: Send info to motors to drive the vehicle
 * @desc:  Add or subtract nextLeftSpeed and nextRightSpeed pulse width offsets from default pulse widths to control speed and direction of each 
 *         motor. See table above for coorelation between pulse width and motor direction/speed. Acceleration is limited by a ramping constant.
 * @param: nextLeftSpeed  -> The pulse width offset representing the next speed the left motor should drive at
 * @param: nextRightSpeed -> The pulse width offset representing the next speed the right motor should drive at
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: Try making these values statics (if they're only changed by the drive function)
// These two values are used by the drive function to track the previous cycle speeds in order to ramp the speed up or down
int prevLeft = 500;
int prevRight = 500;
const int NEUTRAL_SPEED = 500;    // Speed value that represents motor neutral (no motion)

void drive(int nextLeftSpeed, int nextRightSpeed) {
    int leftResetSpeed = prevLeft;      // The current speed of the left motor while it is in its reset cycle
    int rightResetSpeed = prevRight;    // The current speed of the right motor while it is in its reset cycle
    int leftMotorPulse = 0;             // Pulsewidth (in us) to send to left motor controller
    int rightMotorPulse = 0;            // Pulsewidth (in us) to send to right motor controller

    // If a direction change has occured, reset motor speeds to neutral before accepting new input from the joystick
    if (directionChanged) {
        resetMotorSpeeds();
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LEFT MOTOR CONTROL
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // variable controls the next speed the left motor should drive at
    int leftDriveSpeed = map(nextLeftSpeed, 0, 1023, 0, FORWARD_PULSE - REVERSE_PULSE);
    
    // If left motor is signaled to increase from its previous speed by at least RAMPING constant, increase its speed by RAMPING constant 
    if(leftDriveSpeed > prevLeft + INCREASE_RAMPING) {
        leftDriveSpeed = prevLeft + INCREASE_RAMPING;
    }
    // If left motor is signaled to decrease from its previous speed by at least RAMPING constant, decrease its speed by RAMPING constant 
    else if(leftDriveSpeed < prevLeft - INCREASE_RAMPING) {
        leftDriveSpeed = prevLeft - INCREASE_RAMPING;
    }

    // Calculate pulse width to send to right motor controller       
    if(INVERT_LEFT_MOTOR) {  // If left motor is inverted
        leftMotorPulse = FORWARD_PULSE - leftDriveSpeed;
    } else {
        leftMotorPulse = REVERSE_PULSE + leftDriveSpeed;
    }
    
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
    int rightDriveSpeed = map(nextRightSpeed, 0, 1023, 0, FORWARD_PULSE - REVERSE_PULSE);
    
    // If right motor is signaled to increase from previous speed by at least RAMPING constant, increase speed by RAMPING constant  
    if(rightDriveSpeed > prevRight + INCREASE_RAMPING) {
        rightDriveSpeed = prevRight + INCREASE_RAMPING;
    }
    // If right motor is signaled to decrease from previous speed by at least RAMPING constant, decrease speed by RAMPING constant 
    else if(rightDriveSpeed < prevRight - INCREASE_RAMPING) {
        rightDriveSpeed = prevRight - INCREASE_RAMPING;
    }    

    // Calculate pulse width to send to right motor controller
    if(INVERT_RIGHT_MOTOR) {  // If right motor is inverted
        rightMotorPulse = FORWARD_PULSE - rightDriveSpeed;
    } else {
        rightMotorPulse = REVERSE_PULSE + rightDriveSpeed;
    }        

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
    int leftResetSpeed = prevLeft;      // The current speed of the left motor while it is in its reset cycle
    int rightResetSpeed = prevRight;    // The current speed of the right motor while it is in its reset cycle
    
    // While direction changed flag is true, reset left and right motors to zero speed before allowing more speed input
    while (directionChanged) {     

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // RESET LEFT MOTOR
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
        if (leftResetSpeed > NEUTRAL_SPEED + DECREASE_RAMPING) {
            leftResetSpeed = prevLeft - DECREASE_RAMPING;
        } else if (leftResetSpeed < NEUTRAL_SPEED - DECREASE_RAMPING) {
            leftResetSpeed = prevLeft + DECREASE_RAMPING;
        } else {
            leftResetComplete = true;   // Signal that the left motor has come to a stop
        }
        // Calculate pulse width to send to right motor controller
        int leftMotorPulse = 0;    
        if(INVERT_LEFT_MOTOR) {  // If left motor is inverted
            leftMotorPulse = FORWARD_PULSE - leftResetSpeed;
        } else {
            leftMotorPulse = REVERSE_PULSE + leftResetSpeed;
        }
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
        if (rightResetSpeed > NEUTRAL_SPEED + DECREASE_RAMPING) {
            rightResetSpeed = prevRight - DECREASE_RAMPING;
        } else if (rightResetSpeed < NEUTRAL_SPEED - DECREASE_RAMPING) {
            rightResetSpeed = prevRight + DECREASE_RAMPING;
        } else {
            rightResetComplete = true;  // Signal that the right motor has come to a stop
        }
        // Calculate pulse width to send to right motor controller
        int rightMotorPulse = 0;    
        if(INVERT_RIGHT_MOTOR) {  // If right motor is inverted
            rightMotorPulse = FORWARD_PULSE - rightResetSpeed;
        } else {
            rightMotorPulse = REVERSE_PULSE + rightResetSpeed;
        }
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