/**
 * @file    TIGERBot_Main_ESP8266.h
 * @brief   TIGERBot main-board class interface for ESP8266.
 *
 * Provides TIGERBotMain for wireless communication (UDP + ESP-NOW),
 * I2C motor control, sensor sampling, and on-board formation control algorithms.
 *
 * Originally created by Daniel Pickem (2014).
 * Extended by Victor Fernandez-Kim (2018).
 * ESP8266 port and formation algorithm C++ translation by Tonmoy Sarker (2021).
 */

#ifndef _TIGERBOT_MAIN_h_
#define _TIGERBOT_MAIN_h_

//------------------------------------------------------------------
// Define firmware and hardware version
//------------------------------------------------------------------
#define FIRMWARE_VERSION 210831
#define HARDWARE_VERSION 210831
#define FIRMWARE_ADDRESS 10
#define HARDWARE_ADDRESS 30
#define DEBUG_LEVEL 1  /* TO DO: Increase to > 1 to enable verbose Serial output. */

//------------------------------------------------------------------
// Defines
//------------------------------------------------------------------
#define LED_PIN          13

// IMU I2C Address; SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination to obtain declination
// https://data.aad.gov.au/aadc/calc/dms_decimal.cfm to convert to decimal degrees, W(-), E(+)
#define DECLINATION -0.233 // Current declination (degrees) in Baton Rouge, LA.

//------------------------------------------------------------------
// Includes
//------------------------------------------------------------------
/* Include basic Arduino libraries */
#include <ArduinoTrace.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <espnow.h>

/* JSON library */
#include <ArduinoJson.h>

/* Include current sensor */
#include <Adafruit_INA219.h>

/*Include IMU sensor */ 
#include <TIGERBot_LSM9DS1.h>

/* Include NeoPixel library for WS2812 LEDs */
#include <Adafruit_NeoPixel.h>

/* Include message definitions */
#include "TIGERBot_Messages.h"

/* Include I2C and wireless interfaces */
#include "I2CInterface.h"

/* Include TIGERBot interface definitions for WiFi and credentials */
#include "wirelessInterfaceESP8266.h"
#include "wifiConfig.h"

/* Include estimator and controller libraries */
#include "include/controllerBase.h"
#include "include/controllerTarget.h"
#include "include/estimatorBase.h"

/* Include averaging class */
#include "include/average.h"

/* Include EEPROM interface */
#include "include/EEPROM_Interface.h"


#include "TIGERBot_Utility.h"

/* Include low-level ESP8266 headers */
extern "C" {
#include "c_types.h"
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "smartconfig.h"
#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/dns.h"
}

//------------------------------------------------------------------
// ENUM definitions
//------------------------------------------------------------------
enum MODE {MANUAL_MODE, CONTROLLER_MODE};


/* ── C++ standard library and linear algebra ──────────────────────────────── */
#include <bits/stdc++.h>

#include <iostream>
//#include <algorithm> 		// for *min_element(arr, arr + n)
#include<ctime>				//srand()
#include<cstdlib>
//#include "FS.h"

#define _USE_MATH_DEFINES
#include <cmath>  			// for using M_PI, trigonometric functions (sin/cos/tan)
#include <vector> 			

using namespace std;

//#include <BasicLinearAlgebra.h>
//using namespace BLA;

//#include <ArduinoSTL.h>
#include "Eigen313.h"
using namespace Eigen;
//#include "eigen-qp.hpp"
//using namespace EigenQP;
#include "eiquadprog.hpp"
#define PI 3.1416


class TIGERBotMain {
  //----------------------------------------------------------------
  // Public Member Functions
  //----------------------------------------------------------------
  public:
    /* Constructors */
    TIGERBotMain(WirelessInterfaceESP8266*  radio,
                 I2CInterface*              i2c,
                 Adafruit_INA219*           ina219,
				 LSM9DS1* 					imu,
				 TIGERBotUtility*			utility,
                 ControllerBase*            controller = NULL,
                 EstimatorBase*             estimator = NULL);			
														
    /* Destructor */
    ~TIGERBotMain();

    /* S1- Setup function */
/*00*/    void initialize(); 			/** Note: Initializes -> timestamps ->radio_ -> I2C
						** get initial batteryVoltage_ , current_ 
						** retrieve MACAddress_(= radio_->getMACaddress()), set ID_ (= radio_->getMACID())
						** Set initial mode_ (=MANUAL_MODE or CONTROLLER_MODE)
						** Set initial step duration (500 ms)
						** Set firmware & hardware version ; begin EEPROM
						*/

    /* S2- Wireless UDP communication functions */
/*01*/    void updateWireless();     	/** Note: runs f.03 > f.04 > f.02	*/										
/*02*/	  bool processUDPMessage();					  
/*03*/    void sendHeartbeatMessage();	/** Description below:			
	
	* If no Host IP (uses wI.pub.f.19) ; uses /*08*//** to send following data:
	No.	   Item			String fields[] 	String data[]
	-- --------------   ---------------		------------------
	1. Robot_ID			ID					String(ID_)
	2. Mac_address  	MAC					MACAddress_
	
	* If robot has a Host IP, uses /*07*//** to send following data :
																String		float 
	No.	   Item					Description				  Unit	fields[] 	data []
	-- --------------   --------------------------------- ----  ---------	--------------------- 
	1. Heartbeat_msg	msg value 97 in TIGERBot_Messages  97	msgType		MSG_HEARTBEAT
	2. V_bat            battery voltage                   [V]	vBat		batteryVoltage_
    3. I_bat            battery current                   [A]	iBat		current_.getAverage()
    4. rps_left_avg     average left motor velocity      [RPS]	rpsL		rpsL
    5. rps_right_avg    average right motor velocity     [RPS]	rpsR		rpsR
    6. temp_left_avg    average left motor temperature    [C]	tempL		tempL
    7. temp_right_avg   average right motor temperature   [C]	tempR		tempR
    8. I_left_avg       average left motor current        [A]	iMotorL		curL
    9. I_right_avg      average right motor current       [A]	iMotorR		curR
	10. messageCounter  number of messages received/sec [/sec]	msgRecRate	messageCounter_
    11. V_5V            step-up converter output voltage  [V]	vBoost		voltageStepUp
	12. Charging_state	return false. TO DO...			  T/F   charging	isChargingVal
*/
/*04*/    void sendRFMessage();			/** Description below:				
	* If robot has a Host IP, uses /*08*//** to send following data :
															String		String 
	No.	   Item					Description				  	fields[] 	data []
	-- ----------------  ---------------------------------  ---------	------------
	1. RF_msg			 msg value 92 in TIGERBot_Messages  msgType		MSG_RF_DATA
	2. Wifi_Channel_No.  ?????????????????????????????????	channel		channel
    3. Wifi_Physical_     (802.11 B/G/N ... 1/2/3)          phyMode		phyMode
	   mode				  WIFI_PHY_MODE_11B = 1, 
						  WIFI_PHY_MODE_11G = 2, 
						  WIFI_PHY_MODE_11N = 3
    4. RSSI          	??????????????????????????????????	rssi		rssi
    5. SSID         	??????????????????????????????????	ssid		ssid
*/
/*05*/    void sendErrorMessage(String error, String parameters="");
/*06*/    void sendStatusMessage(String status, String parameters="");

    /* S3- JSON communication primitives */
/*07*/    void JSONSendMessage(String* fields, float* data, int len);  // data[] is float ;includes /*11*/
/*08*/    void JSONSendMessage(String* fields, String* data, int len); // data[] is string;includes /*11*/
/*09*/    void JSONSendMessage(String field, String data);            // includes /*08*/&/*11*/
/*10*/    void JSONSendMessage(String field, float data);			  // includes /*07*/&/*11*/
/*11*/    void JSONSendMessage(String JSONData); 				// Sends message via UDP
         /** Description: 
		 JSONSendMessage(String JSONData)> sendMessage(String data) >
		 sendUdpPacket(data) > 
		 if receivedHostIP_ is true, send udp packet or 
		 use default udp broadcasting
		 */
    /* S4- Get field values from JsonObject */
/*11-a*/  template <typename T> bool JSONGetNumber(JsonObject& root, String field, T& output);
/*12*/    String JSONGetString(JsonObject& root, String field);

    /* S5- Controls-related functions */
/*13*/    void updateController();
/*14*/    void updateEstimator();

    /* S6- Visual output(LED) functions */
/*15*/    void toggleLed();
/*16*/    void ledOn();
/*17*/    void ledOff();

    /* S7- Data collection functions */
/*18*/    void updateMeasurements();
/*19*/	  bool sampleMotorBoardAverageRPS(float& rpsL, float& rpsR);
/*20*/	  bool sampleMotorBoardAverageTemperatures(float& tempL, float& tempR);
/*21*/	  bool sampleMotorBoardAverageCurrents(float& curL, float& curR);
/*22*/	  bool testMotorBoardI2CCommunications();
/*23*/	  void readIMU();
/*24*/	  void sampleAccel();
/*25*/	  void sampleGyro();
/*26*/	  void sampleAttitude(float ax_, float ay_, float az_, float mx_, float my_, float mz_);

    /* S8- Sample functions - no return value, stored in class variables */
/*27*/    bool sampleMotorBoardVelocities();
/*28*/    bool sampleMotorBoardRPS();
/*29*/    bool sampleMotorBoardRPSMax();
/*30*/	//bool sampleIMUVelocities();

    /* S9- Status functions */
/*31*/	  bool isBatteryEmpty();
/*32*/	  bool isCharging();
/*33*/	  bool isCharged(bool disconnected = false);
/*34*/    void chargingStatusNotification();
/*35*/    void chargerConnectedNotification();
/*36*/    void printChargeStatus();

    /* S10- Power management functions */
/*37*/    void enableMotorVoltage();
/*38*/    void disableMotorVoltage();

    /* S11- Sleep functions */
/*39*/    void enableDeepSleep();
/*40*/    void enableDeepSleep(uint32_t duration);
/*41*/    void enableCurrentSensorSleep();
/*42*/    void disableCurrentSensorSleep();

    /* S12- Get functions */
/*43*/    State getCurrentPosition();
/*44*/    State getTargetPosition();
/*45*/    float getBatteryVoltage();
/*46*/    float getCurrentConsumption();
/*47*/    float getStepUpVoltage();
/*48*/	  String getIPAddress();

    /* S13- Set functions */
/*49*/    void setCurrentPosition(float x, float y, float theta);
/*50*/    void setTargetPosition(float x, float y, float theta);
/*51*/    void setVelocities(float v, float w);
/*52*/    void setVelocitiesMax(float v, float w);
/*53*/    void setRPS(float rpsLeft, float rpsRight);
/*54*/    void setRPSMax(float rpsMax);
/*55*/    void setStepsPerRevolution(float steps);
/*56.a*/  void setIMUSensor();						//added : 4/20/20.Tonmoy Sarker
/*56.b*/  void setIMUOffsets(float axOff, float ayOff, float gzOff);
/*58*/	  void setStepDuration(uint16_t stepDur);

    /* S14- Utility functions */
/*59*/    float map(float x, float inMin, float inMax, float outMin, float outMax);

    /* S15- Versioning functions */
/*60*/    bool setMainBoardFirmwareVersion(uint32_t version);
/*61*/    bool setMainBoardHardwareVersion(uint32_t version);
/*62*/    uint32_t getMainBoardFirmwareVersion();
/*63*/    uint32_t getMotorBoardFirmwareVersion();
/*64*/    uint32_t getMainBoardHardwareVersion();
/*65*/    uint32_t getMotorBoardHardwareVersion();


    /* ── Formation control algorithms (C++ ports) ──────────────────────────── */
    int Directed_Distance_Area_3();
    int Directed_Distance_Area_4();
    int Directed_Distance_Area_6();
    int TS_UndirectedManeuvering();

    /* ── Utility and diagnostics ─────────────────────────────────────────── */
    void  memoryCheck();
    void  randomWalk();
    void  sendConstantVelocity();
    float applyThreshold(float value, float limit);

    /* ── Robot ID and pose management ───────────────────────────────────── */
    void     robIDs(int idarray[], int len);
    int      getNumberOfRobots();
    void     showIDs();
    MatrixXf getPoses();
    int      dock();
    int      moveToPositions();
    void     runAlgorithm();
    void     resetAlgorithm();

    /* ── Hardware validation test ─────────────────────────────────────────
     * TO DO: The stubs below were planned to verify individual subsystems
     * (barrier cert, QP solver, velocity round-trip) before full algorithm runs.
    int checkingUniBarr();
    int checkingeiquadprog();
    int checkingVel();
    int checkingdock();
    int checkingMoveToPositions();
    int checkingTS_UndirectedManeuvering();
    */


    //--------------------------------------------------------------
    // Public Member Variables
    //--------------------------------------------------------------
	public:
    WirelessInterfaceESP8266* radio_;

	TIGERBotUtility* utility_;

	int NN;
	int numOfRob;
	Eigen::Matrix<int,1,Dynamic> EigenID;
	int	desiredAlgorithm;
	bool  doRunAlgorithm;
	int algorithmRunTime; 
	int stepTime;
	int	delayTime;
	int	graph; // 0:irrelevent or default(directed), 1:directed, 2: undirected	
	uint8_t opMode;		// 1:cen ; 2:semi-cen ; 3:semi-decen ; 4: decen
	uint8_t inputMode;	// 1: developer(default) ; 2: user (CS)  inputs 

    /* Decentralized sensing coordination */
    int publicID;
    bool decentMessage  = false;
    int  decentralized  = 0;
    int  director[3][4];     /* neighbor IDs (col 0) and distances (col 1) for directed graph */
    float controlGain[2];    /* [distance gain, angular gain] */
    private:
    //--------------------------------------------------------------//
    // Private Member Variables										//
    //--------------------------------------------------------------//
															// pvt.
	/* S16- Controls-related */
    uint8_t           mode_;								//v1
    floatUnion        v_;									//v2
    floatUnion        w_;									//v3
    floatUnion        rpsLeft_;								//v4
    floatUnion        rpsRight_;							//v5
    floatUnion        rpsMax_;								//v6
    ControllerBase*   controller_;							//v7
    EstimatorBase*    estimator_;							//v8
	uint16_t      	  stepDur_;								//v9

    /* S17- Wireless communication */
    String MACAddress_;										//v10
	String IPAddress_;										//v11
    int ID_;												//v12

    /* S18- I2C communication */
    I2CInterface* I2C_;										//v13
    I2CMessage I2CBuffer_;									//v14
    uint8_t I2CRequestTimeout_;								//v15

    /* S19- Current/voltage sensing */
    Adafruit_INA219* ina219_;								//v16
    float   batteryVoltage_;								//v17
    Average current_;										//v18

	/* S20- IMU variables */
	LSM9DS1* imu_;											//v19
	float ax_; float ay_; //float az_;						//v20 v21
	//float gx_; float gy_; 
	float gz_;												//v22
	//float mx_; float my_; float mz_;
	float heading_;											//v23
	float axOff_; float ayOff_; float gzOff_;				//v24 v25 v26
				
    /* S21- Charge status */
    bool chargerConnected_;									//v27
    bool chargerConnectedPrev_;								//v28

    /* S22- Time stamps */
    uint32_t lastEstimatorUpdate_;							//v29
    uint32_t lastControllerUpdate_;							//v30
    uint32_t lastMessage_;									//v31
    uint32_t messageCounter_;								//v32
    uint32_t lastHeartbeat_;								//v33
    uint32_t lastRFMessage_;								//v34
    uint32_t lastCurrentMeasurement_;						//v35
	uint32_t lastIMURead_;									//v36
    uint32_t lastI2CTest_;									//v37

    uint32_t lastChargeStatusCheck_;						//v38
    uint32_t lastChargedStatusMessage_;						//v39
    uint32_t lastBatteryEmpytCheck_;						//v40
    uint32_t lastDataTest_;									//v41
	uint32_t lastTransition_;

	/* ── Formation algorithm state ─────────────────────────────────────────── */
	int 	mode;
	int 	*roboids;
	float 	*initialPose1D;
	float 	*pose1D;
	MatrixXf poses;
	MatrixXf initialPoses;
	bool 	receivedPoses;
	bool	receivedInitialPoses;
	//float *velocity1D;	

};
#endif