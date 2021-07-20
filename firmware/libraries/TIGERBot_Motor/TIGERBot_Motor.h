/*
 -------------------------------------------------------------------------------
 TIGERBot Motor Board class

 METHODS:

 NOTES:

 EXAMPLES:

 Initially created by Daniel Pickem 7/18/14.
 Modified by Victor Fernandez-Kim 6/13/18.
	Search "VFK" for comments, edits, etc.
 -------------------------------------------------------------------------------
 */

#ifndef _TIGERBOT_MOTOR_h_
#define _TIGERBOT_MOTOR_h_

//------------------------------------------------------------------
// Includes
//------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/* Include message protocol and I2C interface headers */
#include "TIGERBot_Messages.h"
#include "I2CInterface.h"

/* Include utility headers */
#include "include/utilities/average.h"

//------------------------------------------------------------------
// CPU frequency (8 MHz)
//------------------------------------------------------------------
#define F_CPU 8000000UL

//------------------------------------------------------------------
// Macros
//------------------------------------------------------------------
#define sbi(a, b) (a) |= (1 << (b))
#define cbi(a, b) (a) &= ~(1 << (b))

//------------------------------------------------------------------
// Defines
//------------------------------------------------------------------
#define I2C_TX_LEN_MOTOR    3
#define STEPS_PER_REVOLUTION 4076 // orig: 4076, 1026

//------------------------------------------------------------------
// Define firmware version
//------------------------------------------------------------------
#define FIRMWARE_VERSION 20180913

//------------------------------------------------------------------
// Define digital pin assignments
//------------------------------------------------------------------
/* Black Seeedstudio robots are     V3 */
/* Blue Seeedstudio robots are     V2 */
/* Green Seeedstudio robots are     V1 */

#define VERSION 3
// VFK: Originally had else options for other board version pin config

#if VERSION == 3 // v2 and v3 have similar pin layouts
/* LED pins */
#define LED_LEFT_PORT   PORTD
#define LED_LEFT_PIN        2 // Arduino Pin 2
#define LED_RIGHT_PORT  PORTD
#define LED_RIGHT_PIN       3 // Arduino Pin 3

/* Main board reset pin. VFK: No longer needed. The chips share a reset line*/
//#define MAINBOARD_RESET_PORT   PORTD
//#define MAINBOARD_RESET_PIN        3

/* Motor 1 pin and port definitions */
#define M11_PORT PORTB
#define M12_PORT PORTB
#define M13_PORT PORTB
#define M14_PORT PORTD
#define M11_PIN 1 // Arduino Pin 9
#define M12_PIN 0 // Arduino Pin 8
#define M13_PIN 2 // Arduino Pin 10
#define M14_PIN 7 // Arduino Pin 7

/* Motor 2 pin and port definitions */
#define M21_PORT PORTB
#define M22_PORT PORTD
#define M23_PORT PORTB
#define M24_PORT PORTD
#define M21_PIN 6 // Arduino Pin 20
#define M22_PIN 5 // Arduino Pin 5
#define M23_PIN 7 // Arduino Pin 21
#define M24_PIN 6 // Arduino Pin 6
#endif

class TIGERBotMotor {
    public:
    //--------------------------------------------------------------
    // Lifecycle
    //--------------------------------------------------------------
    /* Constructors */
    TIGERBotMotor();

    /* Destructor */
    ~TIGERBotMotor();

    //--------------------------------------------------------------
    // Public Member Functions
    //--------------------------------------------------------------
    /* Setup functions */
    /* Note that address = 2 is the default address for the motor board
     * The main board expects that address for communication.
     */
/*01*/    void initialize(uint8_t address = 2);

    /* Main motor function */
/*02*/    void step();
/*03*/    void stopMotors();

    /* I2C-related functions */
/*04*/    void requestEvent();
/*05*/    void receiveEvent();
/*06*/    void processI2CMessage(I2CMessage* msg);

    /* Setters */
/*07*/    void setVelocities(float v, float w); //includes pvt.f10, pub.f9, pub.f27
/*08*/    void setVelocitiesMax(float vMax, float wMax);
/*09*/    void setRPS(float rpsLeft, float rpsRight);
/*10*/    void setRPSMax(float rpsMax);
/*11*/    void setStepsPerRevolution(uint16_t steps);

    /* LED functions */
/*12*/    void toggleLeds();
/*13.a*/  void toggleLedLeft();
/*13.b*/  void toggleLedRight();
/*14*/    void ledsOn();
/*15*/    void ledsOff();
/*16*/    void ledOnLeft();
/*17*/    void ledOnRight();
/*18*/    void ledOffLeft();
/*19*/    void ledOffRight();

    /* Status functions */
/*20*/    bool isMaster() { return i2c_.isMaster(); };

    /* Sleep mode functions */
/*21*/    void enableDeepSleep();
/*22*/    void enableDeepSleep(uint32_t sec);
/*23*/    void sleep1Sec();
/*24*/    void sleep8Sec();
/*25*/    void sleepNSec(const byte interval);
/*26*/    //void resetMainBoard();

    /* Data collection functions */
/*27*/    void collectData();

    /* Version functions */
/*28*/    //bool setFirmwareVersion(uint32_t version);
/*29*/    //bool setHardwareVersion(uint32_t version);
/*30*/    //uint32_t getFirmwareVersion();
/*31*/    //uint32_t getHardwareVersion();
	

    //--------------------------------------------------------------
    // Public Member Variables
    //--------------------------------------------------------------
    public:
    /* I2C communication-related variables */
    I2CInterface i2c_;
    I2CMessage I2CBuffer_;

    //--------------------------------------------------------------
    // Private Member Functions
    //--------------------------------------------------------------
    private:
    /* Motor control */
    void stepLeft();							//f1
    void stepLeft(int step);					//f2
    void stepRight();							//f3
    void stepRight(int step);					//f4

    void nextStepLeft();						//f5
    void nextStepRight();						//f6

    bool isStopped();							//f7
    void stopMotorLeft();						//f8
    void stopMotorRight();						//f9

    void saturateVelocities();					//f10
    float map(float x, float inMin, float inMax, float outMin, float outMax); //f10
	
	


    //--------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------
    float rpsMax_;							//v1
    static const float rWheel_;				//v2
    static const float cWheel_;				//v3
    static const float rTrack_;				//v4
    static const float cTrack_;				//v5
    uint16_t    stepsPerRevolution_;		//v6

    /* Velocities */
    float rpsLeft_;							//v7
    float rpsRight_;					    //v8
    float v_;							    //v9
    float w_;							    //v10
    float vMax_;					    	//v11
    float wMax_;					    	//v12

    /* Motor control parameters */
    unsigned long delayLeft_;		    	//v13
    unsigned long delayRight_;		    	//v14
    uint8_t curStepLeft_;		    		//v15
    uint8_t curStepRight_;		    		//v16
    unsigned long lastStepLeft_;		    //v17
    unsigned long lastStepRight_;		    //v18

    /* Deep sleep parameters */
    uint32_t sleepDuration_;		    	//v19
    bool     sleepNow_;		    			//v20

    /* Data collection */
    Average rpsLeftAvg_;		    		//v21
    Average rpsRightAvg_;		    		//v22
    Average tempLeftAvg_;		    		//v23
    Average tempRightAvg_;		    		//v24
    Average currentLeftAvg_;		    	//v25
    Average currentRightAvg_;		    	//v26

    /* LED parameters */
    bool ledStateLeft_;		    			//v27
    bool ledStateRight_;		    		//v28
};
#endif