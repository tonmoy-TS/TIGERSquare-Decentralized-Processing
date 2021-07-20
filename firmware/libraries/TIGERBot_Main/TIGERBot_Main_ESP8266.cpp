/**
 * @file    TIGERBot_Main_ESP8266.cpp
 * @brief   TIGERBot main-board class implementation for ESP8266.
 *
 * Handles wireless communication (UDP + ESP-NOW), I2C motor control,
 * sensor sampling, and on-board formation control algorithms.
 *
 * Originally created by Daniel Pickem (2014).
 * Extended by Victor Fernandez-Kim (2018).
 * ESP8266 port and formation algorithm C++ translation by Tonmoy Sarker (2021).
 */


//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
#include "TIGERBot_Main_ESP8266.h"

// Constructors
TIGERBotMain::TIGERBotMain(WirelessInterfaceESP8266*  radio,
                           I2CInterface*              i2c,
                           Adafruit_INA219*           ina219,
						   LSM9DS1*         		  imu,
                           TIGERBotUtility*           utility,
                           ControllerBase*            controller,
                           EstimatorBase*             estimator) {

  radio_      = radio;
  I2C_        = i2c;
  ina219_     = ina219;
  imu_        = imu;
  utility_    = utility;
  controller_ = controller;
  estimator_  = estimator;
}

// Destructor
TIGERBotMain::~TIGERBotMain() {}

//-----------------------------------------------------------------
// Public Member Functions
//-----------------------------------------------------------------
void TIGERBotMain::initialize() {									/*00*/
  /* Set initial time stamps */
  lastEstimatorUpdate_    = millis(); 
  lastControllerUpdate_   = millis(); 
  lastMessage_            = millis(); 
  lastHeartbeat_          = millis();
  lastRFMessage_          = millis();
  lastCurrentMeasurement_ = millis();
  lastIMURead_            = millis();
  lastI2CTest_            = millis();

  lastChargeStatusCheck_  = millis();
  lastChargeStatusCheck_  = millis();
  lastBatteryEmpytCheck_  = millis();
  lastDataTest_           = millis();
  lastTransition_		  = millis();  //Serial.println("Mainboard time stamps initialized");
  resetAlgorithm();

  /* Set LED pin */
  pinMode(LED_PIN, OUTPUT);
 
	  /* Retrieve MAC address from WiFi device */
  MACAddress_ = radio_->getMACaddress();
  Serial.print("MAC address: "); Serial.println(MACAddress_);

  ID_ = radio_->getMACID();
  Serial.print("MAC ID: "); Serial.println(ID_);
  
  /* Initialize radio */
  radio_->initialize();
  Serial.println("Mainboard radio initialized");
  /* Initialize the message counter */
   messageCounter_ = 0;
  /* Set up mainboard as I2C master */
  I2C_->initialize(D5, D4);

  /* Set I2C communication parameters */
  I2CRequestTimeout_ = 1;               /* Wait time for I2C request in ms */
  Wire.setClockStretchLimit(15000);     /* ESP8266 wait time for messages in us */
  Serial.println("Mainboard I2C initialized");

  /* Set up current sensor */
  /* NOTE: ina219_->begin() does not need to be called since
   * I2C is already set up.
   */
  ina219_->setCalibration_32V_2A();
  Serial.println("Mainboard current monitor initialized"); 

  /*Set up IMU sensor */
//  setIMUSensor(); /* TO DO: IMU calibration disabled — re-enable if heading data is needed. */
  
  /* Get initial battery voltage and current reading */
  batteryVoltage_ = ina219_->getBusVoltage_V();
  current_.addData(ina219_->getCurrent_mA());
  Serial.print("Battery voltage: ");
  Serial.println(batteryVoltage_);
  
  /* Initialize controller */
  //controller_->setL(0.05);
  //controller_->setGain(20);
  //Serial.println("Mainboard controller initialized");
  
  /* Set initial mode */
  mode_ = MANUAL_MODE; // in .h file #define section
  Serial.print("Selected Controller Mode [0-Manual; 1-Controller mode] : "); 
  Serial.println(mode_);
  /* Set initial step duration */
  setStepDuration(500);
	
  /* Set firmware version */
  EEPROM.begin(512);
  Serial.println("EEPROM initiated");  setMainBoardFirmwareVersion(FIRMWARE_VERSION);
  setMainBoardHardwareVersion(HARDWARE_VERSION);
  
  Serial.print("MAC ID: "); Serial.println(ID_);

}

void TIGERBotMain::updateWireless() {					/*01*/
  yield(); /* prevents watchdog resets during long loop iterations */

  if(radio_->isConnected()) {												/*wI.pub.f.05*/
    /* Send heartbeat message */
    sendHeartbeatMessage();								/*03*/

    /* Send RF measurement message, only sent if available Host IP */
    //sendRFMessage();									/*04*/

    /* Receive and process UDP messages */
    if(radio_->receiveMessage() > 0) {										/*wI.pub.f.04*/
		/* Update time stamp of last message */
		lastMessage_ = millis();												/*pvt.v31*/
		messageCounter_ = messageCounter_ + 1;								/*pvt.v32*/

		/* Visual output */
		//toggleLed(); // disabling since causing issues with RGB LED illumination
		Serial.print("Message received (ct(counts): ");
		Serial.print(messageCounter_);
		Serial.println(")");
	  
		/* Process message */
		yield();
		processUDPMessage(); 								/*02*/ 	
	}
  }
}

bool TIGERBotMain::processUDPMessage() { 									/*02*/			
//	Note, processUDPMessage() is accessed inside updateWireless() 
					
  yield();
  uint8_t msgType; 	// switch (cases)
  bool forMe; 		// true or false. Used for checking on message direction
  int desiredID; 
  String error;
  int n = numOfRob;
  	// initialPose1D = new float[n*3]; //1D array 
	// for(int i =0;i<3*n;i++)
		// initialPose1D[i]= 0;

  /* Parse JSON message stored in radio_.msg */
  StaticJsonBuffer<2048> jsonBuffer; 	// 'jsonBuffer' allocates 2048 bytes on stack.
		/** Note: StaticJsonBuffer allocates memory on the stack, it can be
		   replaced by DynamicJsonBuffer which allocates in the heap. Example:
		 
		   DynamicJsonBuffer  jsonBuffer(512);
		 
		   Note that, you need to adjust the capacity according to your JSON data
		 */
		 
  /* Parse JSON data into buffer */
  String msg = radio_->getMessage(); 										/*wI.pub.f.22*/
         // getMessage(){return msg_} outputs msg_, which is used inside  receiveUdpPacket()...
  JsonObject& root = jsonBuffer.parseObject(msg); 	//parse the root object
		
		/** JSON Note:
		To create a root object inside Buffer and add value :
		  { JsonObject& root = jsonBuffer.createObject()
			 root["val_name1"]= "abcd" ;
			 root["val_name2"]=  9999;
			}
		To create a nested array and add value :
		  { JsonArray& data = root.createNestedArray("data");
			 data.add(0000);
			 data.add(5678);
			} 
		Final output : {"val_name1":"abcd","val_name2":9999,"data":[0000,5678]}
		
		To parse the root object :
		 * { JsonObject& root = jsonBuffer.parseObject(file_name);
			}
		 */
		 
  //if(DEBUG_LEVEL > 1) 
		Serial.println(msg);

  /* Parse message direction (to whom it was sent) */
	if (JSONGetNumber<int>(root, String("ID"), desiredID)){
		if (desiredID==ID_||desiredID==0) // In Matlab constructMsg.m, the msgOut string has 'ID' inside them
			forMe = true; 
		else 
			forMe = false;
	}
	else{
		Serial.println("Failed to parse ID target");
		return false;
	}

	if (forMe) {  
	/*Apparently, all SET messages are received by the bots; all GET messages are sent by the bots */ 
	/* Parse message type */
	  if(JSONGetNumber<uint8_t>(root, String("msgType"), msgType)) {			/*11*/
		switch(msgType) {				//18 cases used		
		case(MSG_SET_CURRENT_POSE):		
			{
			if(estimator_ != NULL) {
				float x = 0, y = 0, theta = 0;
				if(JSONGetNumber<float>(root, "x", x) && 
				    JSONGetNumber<float>(root, "y", y) &&	
				     JSONGetNumber<float>(root, "theta", theta)) {	/*11*/
				        /* Set current pose in estimator */
						estimator_->setState(State(x,y,theta));		/* */
					} 
				else {
					error = "MSG_SET_CURRENT_POSE: Failed to parse x,y, or theta: " + msg;
					sendErrorMessage(error);							/*05*/
				 }
				}
				  break;
			}
		case(MSG_SET_TARGET_POSE):
			{
				if(controller_ != NULL) {
					float x = 0, y = 0, theta = 0;
					if(JSONGetNumber<float>(root, "x", x) &&
						JSONGetNumber<float>(root, "y", y) &&
						JSONGetNumber<float>(root, "theta", theta)) {
						/* Update controller variables */
							controller_->setTargetPosition(State(x,y,theta));		/* */

						/* Set operation mode */
						    mode_ = CONTROLLER_MODE;
					} 
					else {
						error = "MSG_SET_TARGET_POSE: Failed to parse x,y, or theta: " + msg;
						sendErrorMessage(error);
						}
					}
				  break;
			}
		case(MSG_SET_VELOCITIES):
			{
			float v = 0, w = 0;
		    /* NOTE: This message type is used for remote control applications */
			/* Update velocities in the controller */
			if(JSONGetNumber<float>(root, "v", v) && JSONGetNumber<float>(root, "w", w)){
					/* Update velocities of the main board */
					setVelocities(v, w);						/*51*/

					/* Set operation mode */
					  mode_ = MANUAL_MODE;
				  } else {
					error = "MSG_SET_VELOCITIES: Failed to parse v or w: " + msg;
						sendErrorMessage(error);
				  }
				  break;
			}
		case(MSG_SET_ALL_VELOCITIES):
			{
				String strV = "v" + String(ID_);// produces a string named "v1" or "v4" etc. based on ID_
				String strW = "w" + String(ID_);// produces a string named "w1" or "w7" etc. based on ID_
				// Serial.print("looking for velocities: ");
				// Serial.print(strV); Serial.print(", ");
				// Serial.println(strW);
				float v = 0, w = 0;
				/* NOTE: This message type is used for remote control applications */
				/* Update velocities in the controller */
				if(JSONGetNumber<float>(root, strV, v) && JSONGetNumber<float>(root, strW, w)){
					/* Update velocities of the main board */
					setVelocities(v, w);							/*51*/
//memoryCheck();					
					/* Set operation mode */
					mode_ = MANUAL_MODE;
				} 
				else {
					error = "MSG_SET_ALL_VELOCITIES: Failed to parse v or w: " + msg;
					sendErrorMessage(error);
					Serial.println("Failed to parse v or w");
				}
				break;
			}
		case(MSG_SET_VELOCITIES_MAX):
			{
				float vMax = 0, wMax = 0;
				  if(JSONGetNumber<float>(root, "vMax", vMax) &&
				  JSONGetNumber<float>(root, "wMax", wMax)) {
					/* Update maximum velocities of the motor board */
							setVelocitiesMax(vMax,wMax);
				  } else {
					error = "MSG_SET_VELOCITIES_MAX: Failed to parse vMax or wMax: " + msg;
						sendErrorMessage(error);
				  }
				  break;
			}
		case(MSG_SET_RPS):
			  {
				float rpsL = 0, rpsR = 0;
				  if(JSONGetNumber<float>(root, "rpsL", rpsL) &&
				  JSONGetNumber<float>(root, "rpsR", rpsR)) {
					/* Update RPS values of the motor board */
							setRPS(rpsL, rpsR);
				  } else {
					error = "MSG_SET_RPS: Failed to parse rpsL or rpsR: " + msg;
						sendErrorMessage(error);
				  }
				  break;
			}
		case(MSG_SET_RPS_MAX):
			{
				float rpsMax = 0;
				  if(JSONGetNumber<float>(root, "rpsMax", rpsMax)) {
					/* Update maximum RPS values of the motor board */
							setRPSMax(rpsMax);
				  } else {
					error = "MSG_SET_RPS_MAX: Failed to parse rpsMax: " + msg;
						sendErrorMessage(error);
				  }
				  break;
			}
		case(MSG_SET_STEP_DURATION):
			{
				uint16_t stepDur = 0;
				  if(JSONGetNumber<uint16_t>(root, "stepDur", stepDur)) {
					/* Update steps duration value for the motor
					 * board's stepper motors */
							setStepDuration(stepDur);
				  } else {
					error = "MSG_SET_STEP_DURATION: Failed to parse step duration: " + msg;
						sendErrorMessage(error);
				  }
				  break;
			}
		case(MSG_SET_STEPS_PER_REV):
			{
				float steps = 0;
				  if(JSONGetNumber<float>(root, "steps", steps)) {
					/* Update steps per revolution value for the motor
				 * board's stepper motors */
							setStepsPerRevolution(steps);
				  } else {
					error = "MSG_SET_STEPS_PER_REV: Failed to parse steps: " + msg;
						sendErrorMessage(error);
				  }
				  break;
			}
		case(MSG_SET_IMU_OFFSETS):
			{
				float axOff = 0; float ayOff = 0; float gzOff = 0;
				  if(JSONGetNumber<float>(root, "axOff", axOff) && 
							JSONGetNumber<float>(root, "ayOff", ayOff) &&
								JSONGetNumber<float>(root, "gzOff", gzOff)) {
					/* Update the IMU offsets to zero the output */
							setIMUOffsets(axOff, ayOff, gzOff);
				  } else {
					error = "MSG_SET_IMU_OFFSETS: Failed to parse offsets: " + msg;
						sendErrorMessage(error);
				  }
				  break;
			}
		case(MSG_GET_BATT_VOLT):
			{
				  String fields[4] = {"ID","msgType", "vBat", "iBat"};
				  float data[4] = {ID_,MSG_GET_BATT_VOLT,batteryVoltage_,-current_.getAverage()};	
				  
				  JSONSendMessage(fields, data, 4);
				  break;
			}
		case(MSG_GET_BATT_EMPTY):
			{
				  String fields[3] = {"ID","msgType", "battEmpty"};
				  float data[3]    = {ID_,MSG_GET_BATT_EMPTY, isBatteryEmpty()};
				  JSONSendMessage(fields, data, 3);
				  break;
			}
		case(MSG_GET_RPS_MAX):
			{
				  String fields[3] = {"ID","msgType", "rpsMax"};
				  float data[3]    = {ID_,MSG_GET_RPS_MAX, sampleMotorBoardRPSMax()};
				  JSONSendMessage(fields, data, 3);
				  break;
			}
		case(MSG_GET_FIRMWARE_VERSION):
			{
				  String fields[4] = {"ID","msgType",
								      "versionMain",
									  "versionMotor"};
					  float data[4] = {ID_,MSG_GET_FIRMWARE_VERSION,
								  getMainBoardFirmwareVersion(),
								  getMotorBoardFirmwareVersion()};
					  JSONSendMessage(fields, data, 4);
				  break;
			}
		case(MSG_GET_HARDWARE_VERSION):
			{
				  String fields[4] = {"ID","msgType","versionMain","versionMotor"};
					  float data[4] = {ID_,MSG_GET_HARDWARE_VERSION,
								  getMainBoardHardwareVersion(),
								  getMotorBoardHardwareVersion()};
					  JSONSendMessage(fields, data, 4);
				  break;
			}
		case(MSG_HOST_IP):
			{
				  /* Parse host IP, incoming and outgoing ports, and WiFi channel */
				  int portIncoming, portOutgoing, setChannelTo;
				  String server_SSID, server_password;

				  if(root.containsKey("host")) {
					radio_->setHostIP(root["host"].as<String>());
					sendStatusMessage("MSG_HOST_IP received");
				  } else {
					error = "MSG_HOST_IP: Failed to parse host IP: " + msg;
					sendErrorMessage(error);
				  }

				  /* Parse incoming port */
				  if(JSONGetNumber<int>(root, "receive_on", portIncoming)) {
					radio_->setPortIncoming(portIncoming);
				  } else {
					error = "MSG_HOST_IP: Failed to parse incoming port: " + msg;
					sendErrorMessage(error);
				  }

				  /* Parse outgoing port */
				  if(JSONGetNumber<int>(root, "send_to", portOutgoing)) {
					radio_->setPortOutgoing(portOutgoing);
				  } else {
					error = "MSG_HOST_IP: Failed to parse outgoing port: " + msg;
					sendErrorMessage(error);
				  }

			  /* Parse the server SSID and password */
			  server_SSID = JSONGetString(root, "ap");
				  radio_->setServerSSID(server_SSID);

			  server_password = JSONGetString(root, "pass");
				  radio_->setServerPassword(server_password);

			  /* Set the channel on which the ESP must connect */
			  if(JSONGetNumber<int>(root, "cha", setChannelTo)) {
					radio_->setWifiChannel(setChannelTo);
				  } else {
					error = "MSG_HOST_IP: Failed to parse channel number : " + msg;
					sendErrorMessage(error);
				  }

				  /* Send status message based on EEPROM sleep flag */
				  bool sleepFlag = EEPROM.read(0);
				  if(sleepFlag) {
					/* Send status message: wake up after sleep */
					sendStatusMessage("woke up");

					/* Reset sleep bit in EEPROM */
					EEPROM.write(0, false);
				  } else {
					/* Send status message: boot up */
					sendStatusMessage("powered up");
				  }

			  /* Disconnect from ap_setup and connect to server_SSID. */
			  radio_->reconnectToMainHost();
				  break;
			}
		case(MSG_DEEP_SLEEP):
			{
					uint32_t sleepTime;

				  /* Activate deep sleep */
				  if(JSONGetNumber<uint32_t>(root, "sleepDuration", sleepTime)) {
					enableDeepSleep(sleepTime);
				  } else {
					  error = "MSG_DEEP_SLEEP: Failed to parse 'sleepDuration': " + msg;
					sendErrorMessage(error);
				  }
				  break;
			}
		case(MSG_GET_IP):
			{
				  String fields[2] = {"msgType", "IP Address"};
					  String data[2]    = {String(MSG_GET_IP), getIPAddress()};
					  JSONSendMessage(fields, data, 2);
				  break; 
			}
		/* Decentralized sensing and coordination messages */
		  case(MSG_SET_DECENTRALIZED):{		//154	
			  JSONGetNumber<int>(root, String("decent"), decentralized);
		  	  decentMessage=true;
			  Serial.println("Got Decen command\n");
		  	  break;
		  }
		  case(MSG_SET_CONTROL_GAIN):{		//156
		  			  JSONGetNumber<float>(root, String("K"), controlGain[0]); //distance control gain
		  			  JSONGetNumber<float>(root, String("B"), controlGain[1]); //angular control gain
		  		  	  String fields[4] = {"ID","msgType", "K","B"};
		  		  	  float data[4]  = {ID_,MSG_SET_CONTROL_GAIN, controlGain[0], controlGain[1]};
		  		  	  JSONSendMessage(fields, data,4);
		  		  	  break;
		  }
		  case(MSG_SET_DIRECTOR):{		//155
			  JSONGetNumber<int>(root,"dir1",director[0][0]);
			  JSONGetNumber<int>(root,"dis1",director[0][1]);
			  JSONGetNumber<int>(root,"dir2",director[1][0]);
			  JSONGetNumber<int>(root,"dis2",director[1][1]);
			  JSONGetNumber<int>(root,"dir3",director[2][0]);
			  JSONGetNumber<int>(root,"dis3",director[2][1]);
			  String fields[5] = {"ID","msgType", "dir1", "dir2", "dir3"};
			  float data[5]  = {ID_,MSG_SET_DIRECTOR, director[0][0], director[1][0], director[2][0]};
			  JSONSendMessage(fields, data,5);
			  break;
		  }
		  case(MSG_GET_DIRECTOR):{		//157
			  int row=0;
			  JSONGetNumber<int>(root,"row",row);
			  String fields[6] = {"ID","msgType", "dirID", "d", "r", "th"};
			  float data[6]  = {ID_,MSG_GET_DIRECTOR, director[row][0], director[row][1], director[row][2],director[row][3]};
			  JSONSendMessage(fields, data, 6);
			  break;
		  }
/* TO DO: MSG_SET_ROBOT_IDS (163) — robot ID push from host, not yet implemented.
		case(MSG_SET_ROBOT_IDS): //163
			{ 
				// JSONGetNumber<int>(root, String("numOfRob"), numOfRob);								
				
				// roboids = new int[len];
				// for(int i=0;i<len;i++){
					// roboids[i]=idarray[i];
				// }	
				//showIDs();
				
				//delete roboids; // clear the memory
				break;
			}
*/
		case(MSG_SET_ALGORITHM_PARAMETERS): //167			{ 
				JSONGetNumber<int>(root, String("desiredAlgorithm"), desiredAlgorithm);								
				JSONGetNumber<int>(root, String("runTime"), algorithmRunTime);
				JSONGetNumber<int>(root, String("stepTime"), stepTime);
				JSONGetNumber<int>(root, String("delayTime"), delayTime);
				//JSONGetNumber<int>(root, String("graph"), graph);//0:irrelevent or default(directed), 1:directed, 2: undirected

				Serial.printf("\nAlgorithm-ID:%d; Runtime:%d mil, Step-time:%d mil, delay:%d mil",
								desiredAlgorithm, algorithmRunTime,stepTime,delayTime);
				doRunAlgorithm = true;
				
				break;
			}
		case(MSG_RESET_ALGORITHM_PARAMETERS): //168			{ 
				resetAlgorithm();
				
				break;
			}
		
		case(MSG_CONSTANT_VELOCITY): //200
			{ /* Received by follower robots; sets velocities without algorithm dispatch */
				float v = 0, w = 0;
				if(JSONGetNumber<float>(root, "v", v) && JSONGetNumber<float>(root, "w", w)){
						/* Update velocities of the main board */
						Serial.println("I got the velocities");
						setVelocities(v, w);						/*51*/
						/* Set operation mode */
						mode_ = MANUAL_MODE; // don't know if it needs yet?
					  } 
				break;	  
			}	
		case(MSG_SET_ALL_CURRENT_POSES): //150
			{	
				yield();
				//n =(root.size()/6)+1; // no need if the class variable already knows the length
				int n = numOfRob;
				pose1D = new float[n*3];

				for(int ii=0;ii<n;ii++){
					String strX  = "x" + String(roboids[ii]); 	 // produces a string "x1" or "x5" etc.
					String strY  = "y" + String(roboids[ii]); 	 // produces a string "y1" or "y3" etc.
					String strTh = "theta" + String(roboids[ii]); // produces a string "theta1" or "theta2" etc.

					float x = 0, y = 0, theta =0;
	
					if(JSONGetNumber<float>(root,strX,x) && JSONGetNumber<float>(root,strY,y)
						&& JSONGetNumber<float>(root,strTh,theta)) {
							
						receivedPoses = true; 
			
						/* Set operation mode */
						//mode_ = MANUAL_MODE;
		
						/* Assign x,y,theta values in the global pose matrix */
						// C++ style 1D array :(x1,y1,th1,x2,y2,th2,x3,....) 			
						pose1D[3*ii] 	= x; 
						pose1D[3*ii+1] 	= y; 
						pose1D[3*ii+2] 	= theta;									
					} 
					else {
						error = "MSG_SET_ALL_CURRENT_POSES: Failed to parse x or y or theta: " + msg;
						sendErrorMessage(error);
						Serial.println("Failed to parse x,y,theta");
						
						receivedPoses = false;
						poses.fill(0);
					}
				}
				/* print the 1D Pose Array */
				Serial.println("\nThe pose vector is : ");	
				for(int k=0;k<(n*3);k++)
					Serial.printf("%0.3f ",pose1D[k]);

				/* Map pose1D into Eigen style matrix */
				// Map<MatrixXf> poses(pose1D,3,n); // This works			
				
				//delete pose1D; // clear the memory				
				//return poses;	// ???
				break;
			}
		case(MSG_SET_ALL_INITIAL_POSES): //162// not complete
			{	
				yield();
				int n = numOfRob;
				initialPose1D = new float[n*3]; //1D array 

				for(int ii=0;ii<n;ii++){
					String strX  = "x" + String(roboids[ii]); 	 // produces a string "x1" or "x5" etc.
					String strY  = "y" + String(roboids[ii]); 	 // produces a string "y1" or "y3" etc.
					String strTh = "theta" + String(roboids[ii]); // produces a string "theta1" or "theta2" etc.

					float x = 0, y = 0, theta =0;
	
					if(JSONGetNumber<float>(root,strX,x) && JSONGetNumber<float>(root,strY,y)
						&& JSONGetNumber<float>(root,strTh,theta)) {
						
						receivedInitialPoses = true;
						
						/* Assign x,y,theta values in the global pose matrix */
						// C++ style 1D array :(x1,y1,th1,x2,y2,th2,x3,....) 			
						initialPose1D[3*ii] 	= x; 
						initialPose1D[3*ii+1] 	= y; 
						initialPose1D[3*ii+2] 	= theta;									
					} 
					else {
						error = "MSG_SET_TARGET_POSES: Failed to parse x or y or theta: " + msg;
						sendErrorMessage(error);
						Serial.println("Failed to parse x,y,theta");
						
						receivedInitialPoses = false;
						initialPoses.fill(0);
					}
				}
				/* print the 1D Pose Array */
				Serial.println("\nReceived initial pose vector: ");	
				for(int k=0;k<(n*3);k++)
					Serial.printf("%0.3f ",initialPose1D[k]);

				/* Map initialPose1D into Eigen style matrix */
				// Map<MatrixXf> initialPoses(initialPose1D,3,n); // This works			
				
				//delete initialPose1D; // clear the memory				
				break;					
			}
			
		 
		default:
			return false;

		}
	  } else {
		  error = "Message contains no message type field: " + msg;
		sendErrorMessage(error);
	  }
	  return true;
    }
	else {
	  Serial.print("Message ignored. Message was sent to robot #");
	  Serial.print(desiredID);
	  Serial.print(", and I am robot #");
	  Serial.println(ID_);
	  return false;
	}
}

void TIGERBotMain::sendHeartbeatMessage() {									/*03*/
  bool printChargeStatusInfo = false;
  bool printBatteryVoltage = false;
  yield();

  /* Send heartbeat message once every two seconds */
  if( (millis() - lastHeartbeat_) > 2000) {									/*pvt.v33*/
    /* Charge status debugging output */
    if(printChargeStatusInfo) {
      printChargeStatus();													/*36*/
    }

    if(printBatteryVoltage) {
      batteryVoltage_  = ina219_->getBusVoltage_V();
      Serial.print("Battery voltage: "); 
	  Serial.println(batteryVoltage_);
    }

    if (!radio_->getHostIPStatus()) {						/*wI.pub.f.19*/ 
	/* Description :  bool getHostIPStatus() { return receivedHostIP_; }; */

	  /* //The following block outputs the MAC & IP Address //
	  // String fields[2] = {"MAC", "IP"};
	  // String data[2] = {MACAddress_, radio_->getLocalIP().toString().c_str()};
	  // JSONSendMessage(fields, data, 2);
	  /* This block outputs the MAC and ID of the robot */
	  String fields[2] = {"ID","MAC"};
	  String data[2] = {String(ID_),MACAddress_};
	  JSONSendMessage(fields, data, 2);										/*08*/
	  } // This signals to the host that the robot has no Host IP
      else {    
	  /* Heartbeat message payload format:

	* If no Host IP (uses wI.pub.f.19) ; uses /*08*//* to send following data:
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
      /* Update battery voltage and current values */
      batteryVoltage_ = ina219_->getBusVoltage_V();//1						/*pvt.v17,v16*/
      current_.addData(ina219_->getCurrent_mA());//2						/*pvt.v18,v16*/

      /* Update average RPS values */
      float rpsL = 0.0;//3
      float rpsR = 0.0;//4
      sampleMotorBoardAverageRPS(rpsL, rpsR);									/*19*/

      /* Update average temperatures */
      float tempL = 0.0;//5
      float tempR = 0.0;//6
      sampleMotorBoardAverageTemperatures(tempL, tempR);						/*20*/

      /* Update average currents */
      float curL = 0.0;//7
      float curR = 0.0;//8
      sampleMotorBoardAverageCurrents(curL, curR);								/*21*/

      /* Update step-up converter voltage */
      float voltageStepUp = getStepUpVoltage();//9								/*47*/

      float isChargingVal = (float) isCharging();//10							/*32*/

      /* Create heartbeat message */
      String fields[12]  = {"msgType", "vBat", "iBat", "rpsL", "rpsR", "tempL", "tempR", 
							"iMotorL", "iMotorR", "msgRecRate", "vBoost", "charging"};
      float data[12]     = {MSG_HEARTBEAT, batteryVoltage_, -current_.getAverage(),
                            rpsL, rpsR, tempL, tempR, curL, curR, (float) messageCounter_, 
							voltageStepUp, isChargingVal}; //is iBat intentionally put negative?

		Serial.print(" <~Hbt~> ");

      /* Send heartbeat message via UDP */
      JSONSendMessage(fields, data, 12);										/*07*/ 
	}

    /* Update timestamp */
    lastHeartbeat_ = millis();								/*pvt.v33*/

    /* Reset the message counter */
    messageCounter_ = 0;									/*pvt.v32*/

    /* Visual output */
    //toggleLed(); // disabling since causing issues with RGB LED illumination	/*15*/
  }
}

void TIGERBotMain::sendRFMessage() {						/*04*/
  /* Retrieve basic WiFi channel data
   *
   * 1. Channel number
   * 2. Physical mode (802.11 B/G/N ... 1/2/3)
   *    WIFI_PHY_MODE_11B = 1, WIFI_PHY_MODE_11G = 2, WIFI_PHY_MODE_11N = 3
   * 3. RSSI
   * 4. SSID
   *
   */
  yield();

  if( (millis() - lastRFMessage_) > 1000) {							/*pvt.v34*/
    if (radio_->getHostIPStatus()) {						/*wI.pub.f.19
															// bool getHostIPStatus() { return receivedHostIP_; };*/
      /* Get channel number */
      uint8_t channel = wifi_get_channel();		/* library function  */

      /* Get physical mode */
      uint8_t phyMode = wifi_get_phy_mode();	/* library function  */

      /* Get SSID */
      struct station_config conf;
      wifi_station_get_config(&conf);			/* library function  */
      const char* ssid = reinterpret_cast<const char*>(conf.ssid);   /* ?? */

      /* Get RSSI */
      int32_t rssi = wifi_station_get_rssi();	/* library function  */

      /* Create heartbeat message */
      String fields[5]  = {"msgType", "channel", "phyMode", "rssi", "ssid"};
      String data[5]    = {String(MSG_RF_DATA), String(channel), String(phyMode),
                           String(rssi), String(ssid)};

      /* Send RF message via UDP */
      JSONSendMessage(fields, data, 5);								/*08*/

      /* Update time stamp */
      lastRFMessage_ = millis();									/*pvt.v34*/
    }
  }
}

void TIGERBotMain::updateMeasurements() {									/*18*/
  /* Set rates for all measurements (Hz) */
  float rateCurrent       = 10.0; //Hz
  float rateIMU			  =  4.0;
  float rateCurrentError  =  0.2;
  float rateI2C           =  0.2;
  float rateBatteryEmpty  =  0.1;
  float rateChargeStatus  =  2.0;

  /* Measure current consumption at 10 Hz */
  if( (millis() - lastCurrentMeasurement_) > 1000 / rateCurrent) {
    /* Add current consumption data */
    current_.addData(ina219_->getCurrent_mA());

    /* Update time stamp */
    lastCurrentMeasurement_ = millis();
  }

  /* Obtain IMU values at 4 Hz */
  // if( (millis() - lastIMURead_) > 1000 / rateIMU) {
		//// Update the sensor values whenever new data is available
		// readIMU();
		// sampleAccel();
		// sampleGyro();
		// sampleAttitude(imu_->ax,imu_->ay,imu_->az,-imu_->my,-imu_->mx,imu_->mz);

		// Serial.print("Ax: "); Serial.print(ax_,2); Serial.print(", Ay:"); Serial.println(ay_,2);
		// Serial.print("Gz: "); Serial.println(gz_,2);
		// Serial.print("Heading: "); Serial.println(heading_, 2); 

		// vo_ = vi_ + ax_ * 0.250;
		// Serial.println(vo_,2); Serial.println(vi_,2);  Serial.println();
		// vi_ = vo_;
		// /* Update time stamp */
    // lastIMURead_ = millis();
  // }
	
  /* Test for current measurement errors at 0.2 Hz */
  if( (millis() - lastCurrentMeasurement_) > 1000 / rateCurrentError) {
    /* Send error message to server */
    sendErrorMessage("I2C current measurement failed.");
    Serial.println("I2C current measurement failed.");
  }

  /* Test I2C communication at 0.2 Hz */
  if( (millis() - lastI2CTest_) > 1000 / rateI2C) {
    if(!testMotorBoardI2CCommunications()) {
      /* Send error message to server */
      sendErrorMessage("I2C communication with motor board failed.");
      Serial.println("I2C communication with motor board failed.");
    }

    /* Update time stamp */
    lastI2CTest_ = millis();
  }

  /* Test for empty battery at 0.1 Hz */
  if( (millis() - lastBatteryEmpytCheck_) > 1000 / rateBatteryEmpty) {
    /* Sample current sensor on main board */
    batteryVoltage_  = ina219_->getBusVoltage_V();

    /* Threshold the battery voltage */
    if(batteryVoltage_ < 3.5) {
      sendStatusMessage("battery warning", String(batteryVoltage_));
    } else if(batteryVoltage_ < 3.2) {
      sendStatusMessage("battery depleted", String(batteryVoltage_));
    } else if(batteryVoltage_ < 3.0) {
      sendStatusMessage("battery critically low", String(batteryVoltage_));
    }

    /* Update time stamp */
    lastBatteryEmpytCheck_ = millis();
  }

  /* Measure charge status at 0.5 Hz */
  if( (millis() - lastChargeStatusCheck_ ) > 1000 / rateChargeStatus) {
    /* Update charger connection status */
    chargerConnectedPrev_ = chargerConnected_;
    chargerConnected_     = 0; // TODO: VFK replaced with 0. See if all this is useful

    /* Transition between 3 states
     * 1. charging
     * 2. charged
     * 3. charging failed
     */
    if(chargerConnected_ != chargerConnectedPrev_) {
      /* Transition on charger connected pin detected */
      if(isCharging()) {													/*32*/
        /* Charger is now connected */
        sendStatusMessage("charging", String(batteryVoltage_));				/*06*/
			/** format:  (String status, String parameters=""); 
								pvt.v17: float batteryVoltage_   */			/*pvt.v17*/
      } else {
        /* Charger is now disconnected */
        if(!isCharged(true)) {												/*33*/
          sendStatusMessage("charging failed", String(batteryVoltage_));
        } else {
          sendStatusMessage("charged", String(batteryVoltage_));
        }
      }
    } else {
      /* If no transition is detected, only send a status update once
         charging is done */

      /* No change on charger connected pin */
      if(isCharging()) {
        if(isCharged()) {
          if( (millis() - lastChargedStatusMessage_ ) > 2000) {				/*pvt.v39*/
            sendStatusMessage("charged", String(batteryVoltage_));
            /* Update time stamp */
            lastChargedStatusMessage_ = millis();
          }
        } else {
          /* Sleep while waiting for charging to complete */
          //enableDeepSleep(15);
        }
      } else {
        /* No charger connected. Continue operation */
      }
    }

    /* Update time stamp */
    lastChargeStatusCheck_ = millis();										/*pvt.v39*/
  }
}
/* ************************************
 *    JSON COMMUNICATION FUNCTIONS
 **************************************/
void TIGERBotMain::JSONSendMessage(String* fields, float* data, int len) {		/*07*/
  /* Create JSON buffer */
  yield();
  StaticJsonBuffer<400> jsonBuffer;

  /* Create JSON root object */
  JsonObject& root = jsonBuffer.createObject();

  /* Fill message with data */
  for (int i = 0; i < len; i++) {
    root[fields[i]] = data[i];
  }

  /* Print to string */
  String msg;
  root.printTo(msg);		// Reference : https://arduinojson.org/v5/api/jsonobject/printto/

  /* Send message via UDP */
  JSONSendMessage(msg);										/*11*/
}

void TIGERBotMain::JSONSendMessage(String* fields, String* data, int len) {		/*08*/
  /* Create JSON buffer */
  yield();
  StaticJsonBuffer<256> jsonBuffer;

  /* Create JSON root object */
  JsonObject& root = jsonBuffer.createObject();

  /* Fill message with data */
  for (int i = 0; i < len; i++) {
    root[fields[i]] = data[i];
  }

  /* Print to string */
  String msg;
  root.printTo(msg);

  /* Send message via UDP */
  JSONSendMessage(msg);										/*11*/
}

void TIGERBotMain::JSONSendMessage(String field, String data) {		/*09*/
  String fields[1]  = {field};
  String d[1]       = {data};
  JSONSendMessage(fields, d, 1);							/*08*/
}

void TIGERBotMain::JSONSendMessage(String field, float data) {		/*10*/
  String fields[1]  = {field};
  float d[1]        = {data};
  JSONSendMessage(fields, d, 1);							/*07*/
}
     
void TIGERBotMain::JSONSendMessage(String JSONData) {		/*11*/
  radio_->sendMessage(JSONData);							/*wI.pub.f.03*/
 //Serial.println(JSONData);
}

void TIGERBotMain::sendErrorMessage(String error, String parameters) {		/*05*/
  if(parameters.length() > 0) {
    /* Send message via UDP */
    String fields[3]  = {"msgType", "msg", "parameters"};
    String data[3]     = {String(MSG_ERROR), error, parameters};

    /* Send charger connection message */
    JSONSendMessage(fields, data, 3);						/*08*/
  } else {
    /* Send message via UDP */
    String fields[2]  = {"msgType", "msg"};
    String data[2]     = {String(MSG_ERROR), error};

    /* Send charger connection message */
    JSONSendMessage(fields, data, 2);
  }
}

void TIGERBotMain::sendStatusMessage(String status, String parameters) {	/*06*/
  if(parameters.length() > 0) {
    /* Create message */
    String fields[3]  = {"msgType", "msg", "parameters"};
    String data[3]     = {String(MSG_STATUS), status, parameters};

    /* Send JSON status message via UDP*/
    JSONSendMessage(fields, data, 3);						/*08*/
  } else {
    String fields[2]  = {"msgType", "msg"};
    String data[2]     = {String(MSG_STATUS), status};

    /* Send JSON status message via UDP*/
    JSONSendMessage(fields, data, 2);						/*08*/
  }

  if(DEBUG_LEVEL > 1) {
    Serial.print("Status: ");
    Serial.print(status);
    Serial.print(", ");
    Serial.println(parameters);
  }
}

/* *****************************************************************
 *    S4- JSON PRIMITIVES - GET FIELD VALUES FROM JSONOBJECT	   *
 ******************************************************************/
template <typename T> bool TIGERBotMain::JSONGetNumber(JsonObject& root, String field, T& output) {		/*11-a*/
  if(root.containsKey(field)) {
    if(root[field].is<T>()) {
      output = root[field].as<T>();
      return 1;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}
String TIGERBotMain::JSONGetString(JsonObject& root, String field) {		/*12*/
  if(root.containsKey(field)) {
    return root[field].asString();
  } else {
    return String("");
  }
}

/* ****************************************
 *      S-5 CONTROLS-RELATED FUNCTIONS	  *
 *****************************************/
void TIGERBotMain::updateController() {										/*13*/
  /* Compute linear and rotational velocity
   *  v ... m/s
   *  w ... rad/sec
   */
  if(mode_ == CONTROLLER_MODE) {											/*pvt.v1*/
  /** Note : In initialization() the mode starts as Manual mode,
  * later,.....
  */
    if(controller_->distanceToTarget(estimator_->getState()) > 0.05) {		/*pvt.v7,v8*/
      /* Compute updated velocities */
      controller_->update(estimator_->getState());							/*pvt.v7,v8*/

      /* Update velocities */
      setVelocities(controller_->getV(), controller_->getW());				/*51*//*pvt.v7*/
    }
  }

  /* Stop robot if no messages were received within 500 ms (default)*/
  if(millis() - lastMessage_ > stepDur_){									/*pvt.v31,v9*/
    /* Update zero velocity settings at 5 Hz */
    if( (millis() - lastControllerUpdate_) > 1000 / 5) {					/*pvt.v30*/
      setVelocities(0.0, 0.0);												/*51*/
      controller_->setV(0.0);							/*	*/
      controller_->setW(0.0);							/*	*/

      /* Update time stamp */
      lastControllerUpdate_ = millis();										/*pvt.v30*/
    }
  }
}

void TIGERBotMain::updateEstimator() {
   /* NOTE: An update using overhead tracking feedback
    *       is done through the wireless update function
    */

  if( (millis() - lastEstimatorUpdate_) > 50) {								/*pvt.v29*/
    estimator_->update(controller_->getV(), controller_->getW());
    lastEstimatorUpdate_ = millis();
  }
}

/* ************************
 *    STATUS FUNCTIONS
 **************************/
bool TIGERBotMain::isBatteryEmpty() {										/*31*/
  /* Sample current sensor on main board */
  batteryVoltage_  = ina219_->getBusVoltage_V();							/*pvt.v17*/

  /* Threshold the battery voltage */
  if(batteryVoltage_ < 3.2) {
    return true;
  } else {
    return false;
  }
}

bool TIGERBotMain::isCharging() {											/*32*/
  /* TODO: VFK Fill in*/
  return false;
}

bool TIGERBotMain::isCharged(bool disconnected) {							/*33*/
  if(disconnected) {
    if(getBatteryVoltage() >= 3.95) {										/*45*/
      return true;
    } else {
      return false;
    }
  } else {
    if(getBatteryVoltage() >= 4.1) {										/*45*/
      return true;
    } else {
      return false;
    }
  }
}

void TIGERBotMain::printChargeStatus() {									/*36*/
  batteryVoltage_  = ina219_->getBusVoltage_V();							/*pvt.v17*/

  if(DEBUG_LEVEL > 1) {
    Serial.print("Bus voltage: ");
    Serial.println(batteryVoltage_);
  }
}

/* ***************************
 * SLEEP FUNCTIONS
 ****************************/
void TIGERBotMain::enableDeepSleep() {										/*39*/
  /* Put the robot to sleep indefinitely (both main and motor board)
   *
   * NOTE: Only a power cycle will wake the robot up
   */
  /* Set sleep bit in EEPROM */
  EEPROM.write(0, true);

  /* Activate sleep */
  enableDeepSleep(0);														/*40*/
}

void TIGERBotMain::enableDeepSleep(uint32_t duration) {						/*40*/
  /* NOTE: Waking up requires the motor board to reset the main board through a
   *       short HIGH - LOW - HIGH pulse on the reset pin.
   * NOTE: This function is called in updateWireless only in the
   *       case when the battery voltage drops below 3.2 V without
   *       a charger being connected to avoid damaging the battery
   *       through under-charging.
   * NOTE: If called with a value duration > 0, the motor board will wake up
   *       the main board through a RESET after duration seconds
   * NOTE: This function reduces power consumption to <10 mA
   * NOTE: The wirewriteregister has to be made public in the Adafruit_INA219
   *       library
   */
  /* Set sleep bit in EEPROM */
  EEPROM.write(0, true);

  /* Send status message to server */
  sendStatusMessage("sleeping", String(duration));							/*06*/
  delay(500);

  /* Enable motor board deep sleep */
  I2C_->sendMessage(MSG_DEEP_SLEEP, duration, 0.0);
  delay(10);

  /* Power down current sensor INA219 */
  /* NOTE: The sensor won't have to be woken up, since the board will be reset
   * instead of woken up. As such, the sensor will be reinitialized on reset.
   * See documentation of Adafruit_INA219 library and example on
   *   https://github.com/jarzebski/Arduino-INA219/blob/master/INA219_simple/INA219_simple.ino
   */
  //enableCurrentSensorSleep();

  /* Power off LED */
  ledOff();

  /* Power down ESP8266 chip on main board */
  ESP.deepSleep(duration);
}
/*
void TIGERBotMain::enableCurrentSensorSleep() {
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
    INA219_CONFIG_GAIN_1_40MV |
    INA219_CONFIG_BADCRES_12BIT |
    INA219_CONFIG_SADCRES_12BIT_1S_532US |
    INA219_CONFIG_MODE_POWERDOWN;
  ina219_->wireWriteRegister(INA219_REG_CONFIG, config);
}

void TIGERBotMain::disableCurrentSensorSleep() {
  ina219_->setCalibration_16V_400mA();
} */

/* *************************
 *      SET FUNCTIONS
 ***************************/
void TIGERBotMain::setVelocities(float v, float w) {							/*51*/
  /* NOTE: v is in [m/sec]
   *       w is in [rad/sec]
   *
   * NOTE: Motor board expects [deg/sec].
   */
  v_.fval = v;
  w_.fval = w;
  I2C_->sendMessage(MSG_SET_VELOCITIES, v, w * 180 / M_PI);						/*I2C.pub.f.03*/
}

void TIGERBotMain::setVelocitiesMax(float vMax, float wMax) {					/*52*/
  /* NOTE: vMax is in [m/sec]
   *       wMax is in [rad/sec]
   *
   * NOTE: Motor board expects [deg/sec]
   */
  I2C_->sendMessage(MSG_SET_VELOCITIES_MAX, vMax, wMax * 180/M_PI);				/*I2C.pub.f.05*/
}

void TIGERBotMain::setRPS(float rpsLeft, float rpsRight) {						/*53*/
  I2C_->sendMessage(MSG_SET_RPS, rpsLeft, rpsRight);							/*I2C.pub.f.03*/
}

void TIGERBotMain::setRPSMax(float rpsMax) {									/*54*/
  I2C_->sendMessage(MSG_SET_RPS_MAX, rpsMax, 0.0);								/*I2C.pub.f.03*/
}
void TIGERBotMain::setStepDuration(uint16_t stepDur){							/*58*/
	stepDur_ = stepDur;															/*pvt.v9*/
}
void TIGERBotMain::setStepsPerRevolution(float steps) {							/*55*/
  I2C_->sendMessage(MSG_SET_STEPS_PER_REV, steps, 0.0);							/*I2C.pub.f.03*/
}
void TIGERBotMain::setIMUSensor() { 											/*56.a*/
	/* Set up IMU sensor */
	/*Before initializing the IMU, there are a few settings we may need to adjust. 
	Use the settings struct to set the device's communication mode and addresses: */
	imu_->settings.device.commInterface = IMU_MODE_I2C;
	imu_->settings.device.mAddress = LSM9DS1_M;
	imu_->settings.device.agAddress = LSM9DS1_AG;
	
	/* The above lines will only take effect AFTER calling imu.begin(), 
	which verifies communication with the IMU and turns it on. */
	if (!imu_->begin()){ Serial.println("Failed to communicate with LSM9DS1.");}
	else { Serial.println("Mainboard IMU initialized");}
  
	/* Read the initial IMU measurements and create offset values */
	readIMU();
	axOff_ = imu_->calcAccel(imu_->ax);
	ayOff_ = imu_->calcAccel(imu_->ay);
	gzOff_ = imu_->calcGyro(imu_->gz);
	Serial.print("The initial IMU offsets are: axOff(");
	Serial.print(axOff_); Serial.print("), ayOff(");
	Serial.print(ayOff_); Serial.print("), gzOff(");
	Serial.print(gzOff_); Serial.println(")");
}

void TIGERBotMain::setIMUOffsets(float axOff, float ayOff, float gzOff) {		/*56.b*/
	axOff_ = axOff;																/*pvt.v24,v25,v26*/
	ayOff_ = ayOff;
	gzOff_ = gzOff;
}
/* *************************
 *    S-12  GET FUNCTIONS
 ***************************/
State TIGERBotMain::getCurrentPosition() {									/*43*/
  return estimator_->getState();
}

State TIGERBotMain::getTargetPosition() {									/*44*/
  return controller_->getTargetPosition();
}

float TIGERBotMain::getBatteryVoltage() {									/*45*/
  batteryVoltage_ = ina219_->getBusVoltage_V();
  return batteryVoltage_;
}

float TIGERBotMain::getCurrentConsumption() {								/*46*/
  return ina219_->getCurrent_mA();
}

float TIGERBotMain::getStepUpVoltage() {									/*47*/
  /* NOTE: The ADC input of the ADC returns a value 0 - 1023,
   *       which corresponds to 0 - 1 V
   * NOTE: A resistor divider maps the input voltage of 0 - 6 V
   *       to a range of 0 - 1 V, which the ESP8266 ADC expects.
   */
  uint16_t voltage = analogRead(A0);
  return map(voltage, 0, 1023, 0, 6);
}

String TIGERBotMain::getIPAddress(){										/*48*/
  return WiFi.localIP().toString().c_str();
}
/* ***********************************
 *  GET DATA FROM MOTORBOARD FUNCTIONS
 *************************************/
bool TIGERBotMain::sampleMotorBoardVelocities() {								/*27*/
  I2C_->sendMessage(MSG_GET_VELOCITIES, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  /* NOTE: Motor board expects [deg/sec].
   */
  if(I2C_->receiveMessage(&I2CBuffer_)) {
    v_.fval = I2CBuffer_.data_[0].fval;
    w_.fval = I2CBuffer_.data_[1].fval * M_PI / 180;
    return true;
  } else {
    return false;
  }
}

bool TIGERBotMain::sampleMotorBoardRPS() {										/*28*/
  I2C_->sendMessage(MSG_GET_RPS, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    rpsLeft_.fval = I2CBuffer_.data_[0].fval;
    rpsRight_.fval = I2CBuffer_.data_[1].fval;
    return true;
  }

  return false;
}

bool TIGERBotMain::sampleMotorBoardRPSMax() {									/*29*/
  I2C_->sendMessage(MSG_GET_RPS_MAX, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    rpsMax_.fval = I2CBuffer_.data_[0].fval;
    return true;
  }

  return false;
}

bool TIGERBotMain::sampleMotorBoardAverageRPS(float& rpsL, float& rpsR) {		/*19*/
  I2C_->sendMessage(MSG_GET_AVG_RPS, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    rpsL = I2CBuffer_.data_[0].fval;
    rpsR = I2CBuffer_.data_[1].fval;
    return true;
  }

  return false;
}

bool TIGERBotMain::sampleMotorBoardAverageTemperatures(float& tempL, float& tempR) {/*20*/
  I2C_->sendMessage(MSG_GET_AVG_TEMPERATURES, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    tempL = I2CBuffer_.data_[0].fval;
    tempR = I2CBuffer_.data_[1].fval;
    return true;
  }

  return false;
}

bool TIGERBotMain::sampleMotorBoardAverageCurrents(float& curL, float& curR) {		/*21*/
  I2C_->sendMessage(MSG_GET_AVG_CURRENTS, 0.0, 0.0);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    curL = I2CBuffer_.data_[0].fval;
    curR = I2CBuffer_.data_[1].fval;
    return true;
  }

  return false;
}

bool TIGERBotMain::testMotorBoardI2CCommunications() {								/*22*/
  /* This function checks if I2C communication between motor and main
   * board works correctly by sending 2 float values and checking their return
   * values for a match
   */
  float r1 = 19;
  float r2 = 83;
  I2C_->sendMessage(MSG_ECHO, r1, r2);
  delay(I2CRequestTimeout_);

  if(I2C_->receiveMessage(&I2CBuffer_)) {
    /* Check if the echoed data matches the sent data */
    if(I2CBuffer_.data_[0].fval == r1 && I2CBuffer_.data_[1].fval == r2) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

void TIGERBotMain::readIMU() {											/*23*/
		if ( imu_->accelAvailable() )
	{
		// To read from the accelerometer, first call the
		// readAccel() function. When it exits, it'll update the
		// ax, ay, and az variables with the most current data.
		imu_->readAccel();
	}
	if ( imu_->gyroAvailable() )
		{
			// To read from the gyroscope,  first call the
			// readGyro() function. When it exits, it'll update the
			// gx, gy, and gz variables with the most current data.
			imu_->readGyro();
		}
		if ( imu_->magAvailable() )
	{
		// To read from the magnetometer, first call the
		// readMag() function. When it exits, it'll update the
		// mx, my, and mz variables with the most current data.
		imu_->readMag();
	}
}

void TIGERBotMain::sampleAccel() {										/*24*/
	// Get ax and ay in g's, zero the value, and convert to SI units (m/s^2)
	ax_ = (imu_->calcAccel(imu_->ax) - axOff_) * 9.81;
	ay_ = (imu_->calcAccel(imu_->ay) - ayOff_) * 9.81;
	//az_ = imu_->calcAccel(imu_->az);
	}

void TIGERBotMain::sampleGyro() {										/*25*/
	// Get values in deg/sec
	//gx_ = imu_->calcGyro(imu_->gx);
	//gy_ = imu_->calcGyro(imu_->gy);
	gz_ = imu_->calcGyro(imu_->gz) - gzOff_;
} 

void TIGERBotMain::sampleAttitude(float ax_, float ay_, float az_, float mx_, float my_, float mz_) {
																		/*26*/
	// mx_ = imu_->calcMag(imu_->mx);
	// my_ = imu_->calcMag(imu_->my);
	// mz_ = imu_->calcMag(imu_->mz);
		
	float roll = atan2(ay_, az_);
  float pitch = atan2(-ax_, sqrt(ay_ * ay_ + az_ * az_));

  if (my_ == 0)
    heading_ = (mx_ < 0) ? PI : 0;
  else
    heading_ = atan2(mx_, my_);

  heading_ -= DECLINATION * PI / 180;
  if (heading_ > PI) heading_ -= (2 * PI);
  else if (heading_ < -PI) heading_ += (2 * PI);
  else if (heading_ < 0) heading_ += 2 * PI;

  // Convert everything from radians to degrees:
  heading_ *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
	}
/* *************************
 *      LED FUNCTIONS
 ***************************/
void TIGERBotMain::toggleLed() {							/*15*/
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void TIGERBotMain::ledOn() {								/*16*/
  digitalWrite(LED_PIN, HIGH);
}

void TIGERBotMain::ledOff() {								/*17*/
  digitalWrite(LED_PIN, LOW);
}


/* **************************
 *    UTILITY FUNCTIONS		*
 ***************************/
float TIGERBotMain::map(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void TIGERBotMain::memoryCheck() {
  SPIFFS.begin();
  FSInfo fs_info;
  SPIFFS.info(fs_info);
  float fileTotalKB = (float)fs_info.totalBytes / 1024.0; 
  float fileUsedKB = (float)fs_info.usedBytes / 1024.0; 
  Serial.println("Free Space: "); Serial.println(ESP.getFreeHeap(),DEC);
}

void TIGERBotMain::randomWalk() {
	/* Set velocities for forward, backward, rotate cw, rotate ccw */
	yield();
	//int mode;
	
	float vSet[] = { -0.6, 0.6 ,  0.0,   0.0};
	float wSet[] = {0.0,  0.0, -360.0 * M_PI / 180.0, 360.0 * M_PI / 180.0};
	String dir[] = {"forward", "backward", "CCW", "CW"};
	//lastTransition_ = millis();

    /* Random transitions every 5 seconds */
		if( millis()-lastTransition_ > 5000 ) {
			//Serial.println(lastTransition_);
			mode = random(0, 4); // random(min,max) returns a random number between (min) and (max-1)
			
			Serial.print("Transition to mode: "); Serial.println(dir[mode]);			
			setVelocities(vSet[mode], wSet[mode]);

			lastTransition_ = millis();
			//Serial.println(lastTransition_);
		}
		else{
		//Serial.println(mode);
		setVelocities(vSet[mode], wSet[mode]);
		delay(2);
		}
}

float TIGERBotMain::applyThreshold(float value,float limit){
	if (value > limit)
        value = limit;
    if (value < -limit)
        value = -limit;
return value;
}

/* *************************
 *    VERSIONING FUNCTIONS
 ***************************/
bool TIGERBotMain::setMainBoardFirmwareVersion(uint32_t version) {				/*60*/
  uint8_t i = EEPROM_writeAnything(FIRMWARE_ADDRESS, version);
  if(i > 0) { return true; } 
  else { return false;}
}

bool TIGERBotMain::setMainBoardHardwareVersion(uint32_t version) {				/*61*/
  uint8_t i = EEPROM_writeAnything(HARDWARE_ADDRESS, version);
  if(i > 0) {return true;} 
  else {return false;}
}

uint32_t TIGERBotMain::getMainBoardFirmwareVersion() {							/*62*/
  uint32_t version;
  uint8_t i = EEPROM_readAnything(FIRMWARE_ADDRESS, version);
  if(i > 0) {return version;} 
  else {return false;}
}

uint32_t TIGERBotMain::getMainBoardHardwareVersion() {							/*63*/
  uint32_t version;
  uint8_t i = EEPROM_readAnything(HARDWARE_ADDRESS, version);
  if(i > 0) {return version;} 
  else {return false;}
}

uint32_t TIGERBotMain::getMotorBoardFirmwareVersion() {							/*64*/
  I2C_->sendMessage(MSG_GET_FIRMWARE_VERSION, 0.0, 0.0);
  delay(I2CRequestTimeout_);
  if(I2C_->receiveMessage(&I2CBuffer_)) {
    return (uint32_t) I2CBuffer_.data_[0].fval;
  }
  return false;
}

uint32_t TIGERBotMain::getMotorBoardHardwareVersion() {							/*65*/
  I2C_->sendMessage(MSG_GET_HARDWARE_VERSION, 0.0, 0.0);
  delay(I2CRequestTimeout_);
  if(I2C_->receiveMessage(&I2CBuffer_)) {
    return (uint32_t) I2CBuffer_.data_[0].fval;
  }

  return false;
}

/* ═══════════════════════════════════════════════════════════════════════════
   V2021 ADDITIONS — MULTI-ROBOT FORMATION CONTROL
   ═══════════════════════════════════════════════════════════════════════════ */

/* MatrixXf TIGERBotMain::thresholdMatElements(MatrixXf& mat_in){
	
	MatrixXf mat_out;
	float threshold = 1e-5;
	//float upperThreshold = -1e-5;

    mat_out = ( mat_in.array().abs()<threshold).select(mat_in, 0.0f);
	
	return mat_out; 
} */
// bool TIGERBotMain::isRobotExpected(int robIDarray[],int robNum){
	
	// for(int i=0;i<robNum;i++){
		// if(ID_==robIDarray[i]){
			// return true;
			// break;
		// }
		// else
			// return false;
	// }
// }
 
void TIGERBotMain::sendConstantVelocity() {
	/* Broadcasts a constant rotation velocity from the leader to all followers via ESP-NOW. */
	Eigen::Matrix<float,2,1> dq(2,1);
	dq << 0,
		  360*(M_PI/180);
		  
	//mode 2
	radio_->sendAllVelocity(dq);
}


void TIGERBotMain::robIDs(int idarray[], int len) {
    /* TO DO: Replace with a method that fetches IDs directly from
     * WirelessInterfaceESP8266 instead of requiring manual injection. */
	roboids = new int[len];
	for(int i=0;i<len;i++){
		roboids[i]=idarray[i];
		//EigenID(i) = roboids[i];//doesn't work
	}	
	
	numOfRob = len;		// store the number of agents
	
	showIDs();
	//delete roboids; // clear the memory
}

int TIGERBotMain::getNumberOfRobots() {
	
	return numOfRob;
}

void TIGERBotMain::showIDs(){
	
	Serial.print("Number of IDs :" );
	Serial.println(numOfRob);
	
	// Shows the IDs in raw C++ array form
	Serial.print("IDs in use: {");
	for(int i=0;i<numOfRob;i++)
		Serial.printf("%d ",roboids[i]);
	Serial.println("}");
	//delete roboids; // clear the memory
	
}

void TIGERBotMain::runAlgorithm() {
	yield();
	if(doRunAlgorithm){
		Serial.printf("alg:%d",desiredAlgorithm);
		Serial.printf("runtime:%d",algorithmRunTime);
		switch(desiredAlgorithm) {
			case(MSG_RUN_moveToPositions):		
				{
					moveToPositions();
					resetAlgorithm();
					break;
				}
			case(MSG_RUN_dock):		
				{
					dock();
					resetAlgorithm();
					break;					
				}
/*			case(MSG_RUN_Directed_Distance_Angle_1):		
				{
					Directed_Distance_Angle_1();
				}
*/
			case(MSG_RUN_Directed_Distance_Area_3):		
				{
					Directed_Distance_Area_3();
					resetAlgorithm();
					break;
				}
			case(MSG_RUN_Directed_Distance_Area_4):		
				{
					Directed_Distance_Area_4();
					resetAlgorithm();					
					break;
				}

			case(MSG_RUN_Directed_Distance_Area_6):		
				{
					Directed_Distance_Area_6();
					resetAlgorithm();					
					break;
				}
/*
			case(MSG_RUN_TS_AngleConstraints):		
				{
					TS_AngleConstraints();
				}
			case(MSG_RUN_TS_AngleConstraints_Switch):		
				{
					TS_AngleConstraints_Switch();
				}
			case(MSG_RUN_TS_AreaConstraints):		
				{
					TS_AreaConstraints();
				}
			case(MSG_RUN_TS_AreaConstraints_Switch):		
				{
					TS_AreaConstraints_Switch();
				}
			case(MSG_RUN_TS_DirectedManeuvering):		
				{
					TS_DirectedManeuvering();
				}
			case(MSG_RUN_TS_FlockingControl_estimate_in_loop_v5):		
				{
					TS_FlockingControl_estimate_in_loop_v5();
				}
			case(MSG_RUN_TS_FlockingControl_estimate_in_loop_v6):		
				{
					TS_FlockingControl_estimate_in_loop_v6();
				}
			case(MSG_RUN_TS_FlockingControl_v3):		
				{
					TS_FlockingControl_v3();
				}
			case(MSG_RUN_TS_Orthogonal_Basis_2D):		
				{
					TS_Orthogonal_Basis_2D();
				}
*/
			case(MSG_RUN_TS_UndirectedManeuvering):		
				{
					TS_UndirectedManeuvering();
					resetAlgorithm();					
					break;
				}		
		}
		resetAlgorithm();
	}
}

void TIGERBotMain::resetAlgorithm() {
		/* Reset all algorithm dispatch parameters to their idle defaults. */
		desiredAlgorithm 	= 0;	
		doRunAlgorithm 		= false;
		algorithmRunTime 	= 0;
		stepTime			= 0;
		delayTime 			= 0;
		graph				= 0; // 0 for irrelevent or default directed
		
		receivedPoses		= 0;
		receivedInitialPoses= 0;
		
		poses.fill(0);
		initialPoses.fill(0);
		
		//delete pose1D;
		//delete initialPose1D;
		
		Serial.println("Algorithm Commands Reset.");
}

MatrixXf TIGERBotMain::getPoses() {
    /* Minimal receive loop that only parses MSG_SET_ALL_CURRENT_POSES,
     * bypassing all other message handlers. Returns a [3×N] pose matrix. */
	yield();
  
	uint8_t msgType;
	int desiredID; 
	String error;
  
	int n = numOfRob;
	pose1D = new float[n*3];
	//velocity1D = new float[n*2];

	/* Parse JSON message stored in radio_.msg */
	StaticJsonBuffer<2048> jsonBuffer; 	// 'jsonBuffer' allocates 2048 bytes on stack.	 
	
	int tries = 0;
	if(radio_->isConnected() && radio_->receiveMessage() > 0) { 
		yield();

		/* Parse JSON data into buffer */
		String msg = radio_->getMessage(); 										/*wI.pub.f.22*/
		JsonObject& root = jsonBuffer.parseObject(msg); 	//parse the root object		 
		Serial.println(msg);

		/* Parse message direction (to whom it was sent) */
		if (JSONGetNumber<int>(root, String("ID"), desiredID)){
			
			if (desiredID==ID_||desiredID==0){ // In Matlab constructMsg.m, the msgOut string has 'ID' inside them

				/* Parse message type */
				if(JSONGetNumber<uint8_t>(root, String("msgType"), msgType)){	/*11*/
					if (msgType == MSG_SET_ALL_CURRENT_POSES){
		
						for(int ii=0;ii<n;ii++){
							String strX  = "x" + String(roboids[ii]); 	 // produces a string "x1" or "x5" etc.
							String strY  = "y" + String(roboids[ii]); 	 // produces a string "y1" or "y3" etc.
							String strTh = "theta" + String(roboids[ii]); // produces a string "theta1" or "theta2" etc.

							float x = 0, y = 0, theta =0;
			
							if(JSONGetNumber<float>(root,strX,x) && JSONGetNumber<float>(root,strY,y)
								&& JSONGetNumber<float>(root,strTh,theta)) {
									
								receivedPoses = true;

								/* C++ style 1D array :(x1,y1,th1,x2,y2,th2,x3,....) */			
								pose1D[3*ii]= x; pose1D[3*ii+1]= y; pose1D[3*ii+2]= theta;													
							} 
							else {
								error = "MSG_SET_ALL_CURRENT_POSES: Failed to parse x or y or theta: " + msg;
								sendErrorMessage(error);
								Serial.println("Failed to parse x,y,theta");
							}
						}
						/* print the 1D Pose Array (for debugging) */
						// Serial.println("Received pose matrix: ");	
						// for(int k=0;k<(n*3);k++)
							// Serial.printf("%0.3f ",pose1D[k]);
					}						
				}
			} 
		}			
	}
	else{
		receivedPoses = false;
		poses.fill(0);
	}
	
	/* Map pose1D into Eigen style matrix */			
	Map<MatrixXf> poses(pose1D,3,n); // This works
	//Map<Matrix<float,3,n>> poses(pose1D); // Didn't work	
	
	//delete pose1D; // clear the memory
	return poses;	
}

/* ═══════════════════════════════════════════════════════════════════════════
   FORMATION CONTROL ALGORITHMS
   Ported from MATLAB to C++ by Tonmoy Sarker.
   ═══════════════════════════════════════════════════════════════════════════ */

int TIGERBotMain::moveToPositions() {
    /* Parks N robots at hardcoded initial positions using the automatic
     * parking controller and unicycle barrier certificate. Runs until
     * all robots converge or algorithmRunTime elapses (Mode 2). */
	yield();
	Serial.println("\nMoving to initial positions...");
	
	int n = numOfRob;	
	float safety = 0.08;
	float lambda = 0.08;

	// Use set positions
	MatrixXf initial_positions(3,n);
	MatrixXf initial_positions2(3,n);
/*	//4 agents..
	initial_positions << 0.4413,0.1271,0.2665,0.8285,//0.8366,
						 0.9781,0.5919,0.0787,0.1477,//0.7036,
						 0.052,0.0061,0.1052;0.1695;//0.1242; //+ [0.2;0.2;0];
						 */
	//3 agents..
	initial_positions << 0.5,0.7,0.4,
						 1.2,0.9,0.6,
						 0.0520,0.0061,0.1052;
						 
	initial_positions2 << 0.5,0.5,0.5,
						 1.3,1.1,0.9,
						 PI,PI,PI;
						 
		/* Map initialPose1D into Eigen style matrix */		//CRASHED	
		//Map<MatrixXf> target_pose(initialPose1D,3,n); 
		//cout<<"\nTarget_pose:"<<endl<<initial_positions<<endl;
		
	// Or randomly generated positions        
/* 	N = length(IDs);
	X = [90 1400]/1000;
	T = [0 2*pi];
	initial_positions = [(rand(N,1) * range(X) + min(X)),...
						(rand(N,1) * range(X) + min(X)),...
							rand(N,1) * range(T) + min(T)]' */
  
	MatrixXf q(3,n);q.fill(0);
	MatrixXf dq(2,n);dq.fill(0);
	
	// Parking
	float PositionError =0.01;
	float RotationError =0.05;	
	float v =0; float w =0;
	
	uint32_t stmp2 = millis();
	while(!(utility_ -> create_is_initialized(q,initial_positions,PositionError,RotationError))){
		
		yield();
		if((millis()-stmp2) > (algorithmRunTime)){
			Serial.printf("\nElapsed time : %d. Time out !!\n",millis()-stmp2);
			break;
		}
		
		q = getPoses();
		cout<<"\nq:\n"<<q<<endl;
			delete pose1D; // clear the memory
		
		if(receivedPoses){
			yield();
			dq =  utility_ -> create_automatic_parking_controller2(q,initial_positions,PositionError,RotationError);
			dq =  utility_ -> create_uni_barrier_certificate(dq,q,safety,lambda); 		
			cout<<"\ndq:\n"<<dq<<endl;				
yield();
		//mode 2
		if(ID_ == roboids[0])
			radio_->sendAllVelocity(dq);
		
		//mode 3
		//if(receivedPoses){
			for(int i=0;i<n;i++){
				if(ID_ == roboids[i]){
					v = dq(0,i); w = dq(1,i);
					setVelocities(v, w);
				}
			}
		//}	
		//else 
			//setVelocities(0,0);

		}
		delay(delayTime);
		//memoryCheck();
	}		
}

int TIGERBotMain::dock() {
    /* Parks all robots along the left edge of the arena in a vertical line
     * with fixed inter-robot spacing, facing inward (theta = PI). */
	yield();

	int n = numOfRob;
    float safety = 0.05;
    float lambda = 0.05;
         
    // Generate docking positions
	MatrixXf X = 0.15 * MatrixXf::Ones(1,n);

	float dy = -0.2;
	
	MatrixXf Y(1,n);
	Y(0)=1.3;
	for(int i=1; i<n;i++)
		Y(i)= Y(i-1)+ dy;
	
	MatrixXf theta = PI * MatrixXf::Ones(1,n);	
	
	MatrixXf pose(3,n);
	pose << X,
			Y,
			theta;
    
	// Parking
	float PositionError =0.01;//was 0.01
	float RotationError =0.05;	

    MatrixXf q(3,n);q.fill(0);
	MatrixXf dq(2,n);dq.fill(0);
	
	float v =0; float w =0;
	
	//uint32_t stmp2 = millis();
 	while(!(utility_ -> create_is_initialized(q,pose,PositionError,RotationError))){
		yield();
		// if((millis()-stmp2) > (algorithmRunTime)){
			// Serial.printf("algorithmRunTime:%d.Elapsed time : %d. Time out !!\n",algorithmRunTime,millis()-stmp2);
			// break;
		// }

		q = getPoses();
		cout<<"\nq:"<<endl<<q<<endl;
		delete pose1D; // clear the memory
		
		if(receivedPoses){			
		dq =  utility_ -> create_automatic_parking_controller2(q, pose,PositionError,RotationError);
		dq =  utility_ -> create_uni_barrier_certificate(dq, q,safety,lambda); 		
		cout<<"\ndq:"<<endl<<dq<<endl;

		//mode 2
		//radio_->sendAllVelocity(dq);
		if(ID_ == roboids[0])
			radio_->sendAllVelocity(dq);
		//mode 3
		//if(receivedPoses){
			for(int i=0;i<n;i++){
				if(ID_ == roboids[i]){
					v = dq(0,i); w = dq(1,i);
					setVelocities(v, w);
				}
			}
		//}	
		//else 
			//setVelocities(0,0);

		}
		delay(delayTime);
	}		
}

int TIGERBotMain::Directed_Distance_Area_3() {
    /* 3-agent undirected distance-rigidity formation control.
     * Drives robots to an equilateral triangle using edge tension energy.
     * TO DO: Parking initialization loop is commented out — place robots
     *        at initial_conditions manually before running. (Mode 3) */
	yield();
	
	int n = numOfRob; //cout<<"n:"<<n<<endl;
	
	// Declarations //	
	/* Initialize velocity vector for agents.  Each agent expects a 2 x 1
	   velocity vector containing the linear and angular velocity, respectively. */
	Eigen::Matrix<float,2,Dynamic> dx(2,n); 			dx.fill(0);
	Eigen::MatrixXf d(n,n); 							d.fill(0);
	Eigen::Matrix<float,Dynamic,4> ad(n-2,4); 			ad.fill(0);
	Eigen::Matrix<float,2,Dynamic> q_desire(2,n); 		q_desire.fill(0);
	
	Eigen::Matrix<float,3,Dynamic> initial_conditions(3,n); 
	
	Eigen::Matrix2i J;
	
	Eigen::Matrix<float,3,Dynamic> x(3,n); 			x.fill(0);
	Eigen::Matrix<float,2,Dynamic> q(2,n); 			q.fill(0);
	Eigen::Matrix<float,2,Dynamic> dxu(2,n); 		dxu.fill(0);
	
	Eigen::VectorXf vr(2*n-3); 						vr.fill(0);
	
	/* Graph settings*/
	Eigen::Matrix3i A; 
//if(graph = )
	// A << 0,0,0,   // 3 DIRECTED
		 // 1,0,0,
		 // 1,1,0;

	A << 0,1,1,   // 3 UNDIRECTED
		 1,0,1,
		 1,1,0;
	 
	float formationRadius = 0.21;					 
	q_desire << 0,1,2,
				0,sqrt(3),0;
	q_desire = 	formationRadius	* q_desire;	
	//cout << "\nq_desire:\n"<<q_desire <<endl;
	 // COLLINEAR
	 // initial_conditions = .2*[1 2 3]+.1;  
	 // initial_conditions = [initial_conditions; .75*ones(1,3); pi/2*ones(1,3)];

	// REFLECTED
	Matrix2f coeff1; Vector2f coeff2; Vector3f translation;
	coeff1 << 1,0,
			  0,-1;
	coeff2 << 0,
			  1;
	translation <<0.75,
				  1.4,
				   0;
	initial_conditions << coeff1*q_desire - 0.4*coeff2*MatrixXf::Ones(1,n),
						  (PI/2)* MatrixXf::Ones(1,n);
	for (int c=0; c<n ;c++)
		initial_conditions.col(c) = initial_conditions.col(c) + translation;
	//cout <<"\n ic:\n" <<initial_conditions <<endl;
	
	// Control Gains
	int Kr = 5;
	int Ka = 20;

	for(int ii=0;ii<n;ii++)
		for(int jj=0;jj<n;jj++)	
			d(ii,jj)=(q_desire.col(ii)-q_desire.col(jj)).norm();
	//cout<<"d:\n"<<d<<endl;
	
	/* Precompute area constraint table (ad) for triangle triplets in the graph */
	Eigen::Matrix3f temp_mat;
	int ord = 0;
	for (int ii = 0; ii<n; ii++){
      for (int jj = 0; jj<n; jj++){
         for (int kk = 0; kk<n; kk++){
            if (A(ii,jj) == 1 && A(ii,kk) == 1 && A(kk,jj) == 1){     //
                ord = ord+1;
				
				temp_mat<<1,1,1;
					      q_desire(1,ii),q_desire(1,jj),q_desire(1,kk),
						  q_desire(2,ii),q_desire(2,jj),q_desire(2,kk);
                
				float temp_area = 1/2*temp_mat.determinant(); //
                if (temp_area > 0) 
					ad.row(ord)<< ii+1,jj+1,kk+1,temp_area;  //
                if (temp_area < 0) 
					ad.row(ord)<<ii+1,kk+1,jj+1,-temp_area; //
            }
         }
      }
    }

	J << 0,1,
		-1,0;

	/* Grab tools for converting to single-integrator dynamics and ensuring safety */
	//Gains for the transformation from single-integrator to unicycle dynamics
	float linearVelocityGain = 1;
	float angularVelocityGain = PI/2;
	float formationControlGain = 4;

	float safety = 0.1;
	float lambda = 0.05;//projection_distance

	// Parking
	float PositionError =0.01;
	float RotationError =0.1;
	float v =0; float w =0;

	Serial.println("Moving to initial positions...");
/*
	while(!(utility_ -> create_is_initialized(x,initial_conditions,PositionError,RotationError))){
	
		x = getPoses();
		
		delete pose1D; // clear the memory
		
		if(receivedPoses){
			yield();
			dxu =  utility_ -> create_automatic_parking_controller2(x,initial_conditions,PositionError,RotationError);
			dxu =  utility_ -> create_uni_barrier_certificate(dxu,x,safety,lambda);			
			// dxu = create_si_barrier_certificate(dxu,x,safety);
		
			// mode 2
			// if(ID_ == roboids[0])
				// radio_->sendAllVelocity(dxu);
		
			// mode 3
			for(int i=0;i<n;i++){
				if(ID_ == roboids[i]){
					v = dxu(0,i); w = dxu(1,i);
					setVelocities(v, w);
				}
			}
		}
		delay(delayTime);
	}		
*/
	// Iterate for the previously specified number of iterations
	Serial.println("Moving to formation...");

	uint32_t timeStamp = millis();
	while((millis()-timeStamp) < algorithmRunTime){
		x = getPoses();

		delete pose1D; // clear the memory(to avoid stack overflow in Nodemcu)
	
		if(receivedPoses){						
			// This section contains the actual algorithm for formation control!
			q = x.topRows<2>();
TRACE();			
			Eigen::VectorXf u(2*n); 							u.fill(0);   
			Eigen::Matrix<float,Dynamic,Dynamic> e(n,n); 		e.fill(0);
			Eigen::Matrix<float,Dynamic,Dynamic> R(2*n-3,2*n); 	R.fill(0);
			Eigen::VectorXf e2n(2*n-3); 						e2n.fill(0);

			ord = 0; float temp =0;
			for(int i=0; i<n; i++){
				//for(int j=0; j<n; j++){// for directed
				for(int j=i+1; j<n; j++){// for undirected
					temp =((q.col(i)-q.col(j)).transpose())*(q.col(i)-q.col(j));
					e(i,j) = sqrt(temp)-d(i,j);
					if (A(i,j) == 1){
						e2n(ord) = e(i,j)*(e(i,j)+2*d(i,j));
						R.row(ord).middleCols<2>(2*i) = (q.col(i)-q.col(j)).transpose();
						R.row(ord).middleCols<2>(2*j) = -(q.col(i)-q.col(j)).transpose(); // for undirected						
						ord = ord+1;
					}
				}
			}			
TRACE();		
			vr = -Kr*(R.transpose()*e2n); // ( R'[2N*2N-3] x e2n[2N-3*1] )
			
			// TO DO.commented out in matlab ( Area constraint) ???
			// Eigen::VectorXf va(2*n); va.fill(0);
			// Eigen::Matrix<float,Dynamic,Dynamic> temp_nodes; temp_nodes.fill(0);
			// Eigen::Matrix<float,Dynamic,Dynamic> temp_a; temp_a.fill(0);
			// TODO
			// for (int i=0;i<(n-2);i++){
			//         temp_nodes = uint16(ad.row(i).leftCols(3));
			//         temp_a = 1/2*det([1 1 1;q(:,temp_nodes)]);
			//         va(2*temp_nodes(1)-1:2*temp_nodes(1)) = - Ka*(temp_a - ad(i,4))*J*(q(:,temp_nodes(2))-q(:,temp_nodes(3)));
			//     }

			u = vr; //+ va + vm;
			
			Map<MatrixXf> dx(u.data(),2,n);   //dx = reshape(u,2,[]);
TRACE();			
			// Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
			dx = utility_ -> create_si_barrier_certificate(dx,x,safety);
			dx = utility_ -> create_si_to_uni_mapping2(dx,x,linearVelocityGain,angularVelocityGain);	
			////dx = unicycle_barrier_certificate(dx, x);
			cout<<"dx:\n"<<dx<<endl;
		
			// Set velocities of agents 1:n
			//mode 2
			// if(ID_ == roboids[0])
				// radio_->sendAllVelocity(dx);
	TRACE();	
			//mode 3
			for(int i=0;i<n;i++){
				if(ID_ == roboids[i]){
					v = dx(0,i); w = dx(1,i);
					setVelocities(v, w);
				}
			}
		}
		delay(delayTime);
	}
}


int TIGERBotMain::Directed_Distance_Area_4() {
    /* 4-agent undirected distance-rigidity formation control.
     * Drives robots to a square formation using edge tension energy.
     * TO DO: Parking initialization loop is commented out — place robots
     *        at initial_conditions manually before running. (Mode 2) */
	yield();
	
	int n = numOfRob;
	
	// Declarations //	
	/* Initialize velocity vector for agents.  Each agent expects a 2 x 1
	   velocity vector containing the linear and angular velocity, respectively. */
	Eigen::Matrix<float,2,Dynamic> dx(2,n); 			dx.fill(0);
	Eigen::MatrixXf d(n,n); 							d.fill(0);
	Eigen::Matrix<float,Dynamic,4> ad(n-2,4); 			ad.fill(0);
	Eigen::Matrix<float,2,Dynamic> q_desire(2,n); 		q_desire.fill(0);
	
	Eigen::Matrix<float,3,Dynamic> initial_conditions(3,n); 
	
	Eigen::Matrix2i J;
	
	Eigen::Matrix<float,3,Dynamic> x(3,n); 			x.fill(0);
	Eigen::Matrix<float,2,Dynamic> q(2,n); 			q.fill(0);
	Eigen::Matrix<float,2,Dynamic> dxu(2,n); 		dxu.fill(0);
	
	Eigen::VectorXf vr(2*n-3); 						vr.fill(0);
	
	// Define timing variables
	Eigen::Matrix<float,10,1> desiredStepTime(10,1); 
	desiredStepTime << 0.14,0.15,0.16,0.18,0.19,0.21,0.22,0.24,0.25,0.27; // Retrieved from TIGERSquare.m Matlab
	float stepTime = desiredStepTime(n-1);  // step time (Specific to TIGERSquare, do not modify)
									/* Note : C/C++ array index starts from 0, 
											  while Matlab array index starts form 1 */
	int iterations=0;
	float T =0; 
		for(int i=1;;i++){
			if (T <200)		//T = 0:d:200; // define total run time
				T = T + stepTime;
			else{
				iterations= i;
				break;
			}
		}
	Serial.printf("\niterations:%d",iterations);
	/* Graph settings*/
	Eigen::Matrix4i A; 
	// A<< 0,0,0,0,
		// 1,0,0,0,
		// 1,1,0,0,   // add/remove connection here to make the framework flexible
		// 1,0,1,0;  // each row represents connection of each agent with the others (column-wise sequentially)

// Undirected
	A<< 0,1,1,1,
		1,0,1,0,
		1,1,0,1,   // add/remove connection here to make the framework flexible
		1,0,1,0;
		
		
	float formationRadius = 0.2;
	int desired_output =3; //(q_desire)
	
	//q_desire = utility_ -> getPolygonDims(formationRadius,n,desired_output);
	q_desire << 0.4914,0.4914,0.2086,0.2086,
				0.2086,0.4914,0.4914,0.2086;
	cout<<"q_desire:\n"<<q_desire<<endl;
	

	initial_conditions <<0.5387,0.5703,0.2555,0.2693,
						 0.2253,0.5527,0.5446,0.2092,
						 5.7896,1.5663,3.4028,3.2702;						 

	// Control Gains
	int Kr = 5;
	int Ka = 5;

	for(int ii=0;ii<n;ii++)
		for(int jj=0;jj<n;jj++)	
			d(ii,jj)=(q_desire.col(ii)-q_desire.col(jj)).norm();
	cout<<"d:\n"<<d<<endl;
	
	/* Precompute area constraint table (ad) for triangle triplets in the graph */
	Eigen::Matrix3f temp_mat;
	int ord = 0;
	for (int ii = 0; ii<n; ii++){
      for (int jj = 0; jj<n; jj++){
         for (int kk = 0; kk<n; kk++){
            if (A(ii,jj) == 1 && A(ii,kk) == 1 && A(kk,jj) == 1){     //
                ord = ord+1;
				
				temp_mat<<1,1,1;
					      q_desire(1,ii),q_desire(1,jj),q_desire(1,kk),
						  q_desire(2,ii),q_desire(2,jj),q_desire(2,kk);
                
				float temp_area = 1/2*temp_mat.determinant(); //
                if (temp_area > 0) 
					ad.row(ord)<< ii+1,jj+1,kk+1,temp_area;  //
                if (temp_area < 0) 
					ad.row(ord)<<ii+1,kk+1,jj+1,-temp_area; //
            }
         }
      }
    }

	J << 0,1,
		-1,0;

	/* TO DO
	// allocate variables to save.
	Matrix::<float,Dynamic,Dynamic> q_s(iterations+1,2*n);q_s.fill(0);
	Matrix::<float,Dynamic,Dynamic> dir_s(iterations+1,n);dir_s.fill(0);
	Matrix::<float,Dynamic,Dynamic> e_s(iterations+1,2*n-3);e_s.fill(0);
	Matrix::<float,Dynamic,Dynamic> u_s(iterations+1,2*n);u_s.fill(0);
	*/

	/* Grab tools for converting to single-integrator dynamics and ensuring safety */
	//Gains for the transformation from single-integrator to unicycle dynamics
	float linearVelocityGain = 1;
	float angularVelocityGain = PI/2;
	float formationControlGain = 4;

	float safety = 0.12;
	float lambda = 0.03;//projection_distance

	// Parking
	float PositionError =0.01;
	float RotationError =0.1;
	float v =0; float w =0;
/*
	Serial.println("Moving to initial positions...");
	while(!(utility_ -> create_is_initialized(x,initial_conditions,PositionError,RotationError))){
	
		x = getPoses();
		
		delete pose1D; // clear the memory
		
		if(receivedPoses){
			yield();
			dxu =  utility_ -> create_automatic_parking_controller2(x,initial_conditions,PositionError,RotationError);
			dxu =  utility_ -> create_uni_barrier_certificate(dxu,x,safety,lambda);			
			//dxu = create_si_barrier_certificate(dxu,x,safety);
		
			//mode 2
			// if(ID_ == roboids[0])
				// radio_->sendAllVelocity(dxu);
		
			//mode 3
			for(int i=0;i<n;i++){
				if(ID_ == roboids[i]){
					v = dxu(0,i); w = dxu(1,i);
					setVelocities(v, w);
				}
			}
		}
		delay(delayTime);
	}		
*/

	// Iterate for the previously specified number of iterations
	Serial.println("Moving to formation...");
	int t=0;
	uint32_t timeStamp = millis();
	while((millis()-timeStamp) < algorithmRunTime){

	//for (int t = 0;t<iterations;t++){
		x = getPoses();

		delete pose1D; // clear the memory(to avoid stack overflow in Nodemcu)
	
		if(receivedPoses){	
			t++;		
			// This section contains the actual algorithm for formation control!
			q = x.topRows<2>();
			////q_s(t+1,:) = reshape(q,1,[]);
			////dir_s(t+1,:) = x(3,:);
			//%q = reshape(q_vec,2,[]);
			
			Eigen::VectorXf u(2*n); 							u.fill(0);   
			Eigen::Matrix<float,Dynamic,Dynamic> e(n,n); 		e.fill(0);
			Eigen::Matrix<float,Dynamic,Dynamic> R(2*n-3,2*n); 	R.fill(0);
			Eigen::VectorXf e2n(2*n-3); 						e2n.fill(0);
			
			ord = 0; float temp =0;
			for(int i=0; i<n; i++){
				for(int j=i+1; j<n; j++){
				//for(int j=0; j<n; j++){// if und change to j = i+1:n
					temp =((q.col(i)-q.col(j)).transpose())*(q.col(i)-q.col(j));
					e(i,j) = sqrt(temp)-d(i,j);
					if (A(i,j) == 1){
						e2n(ord) = e(i,j)*(e(i,j)+2*d(i,j));
						R.row(ord).middleCols<2>(2*i) = (q.col(i)-q.col(j)).transpose();
						R.row(ord).middleCols<2>(2*j) = -(q.col(i)-q.col(j)).transpose(); // only for UNDIRECTED
												
						ord = ord+1;
					}
				}
			}			
		
			vr = -Kr*(R.transpose()*e2n); // ( R'[2N*2N-3] x e2n[2N-3*1] )
			
			// TO DO.commented out in matlab
			// Eigen::VectorXf va(2*n); va.fill(0);
			// Eigen::Matrix<float,Dynamic,Dynamic> temp_nodes; temp_nodes.fill(0);
			// Eigen::Matrix<float,Dynamic,Dynamic> temp_a; temp_a.fill(0);
			// TODO
			// for (int i=0;i<(n-2);i++){
			//         temp_nodes = uint16(ad.row(i).leftCols(3));
			//         temp_a = 1/2*det([1 1 1;q(:,temp_nodes)]);
			//         va(2*temp_nodes(1)-1:2*temp_nodes(1)) = - Ka*(temp_a - ad(i,4))*J*(q(:,temp_nodes(2))-q(:,temp_nodes(3)));
			//     }

			u = vr; //+ va + vm;0
			//// u_s(t+1,:) = u'; // save variable
			
			Map<MatrixXf> dx(u.data(),2,n);   //dx = reshape(u,2,[]);
			
			// Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
			dx = utility_ -> create_si_barrier_certificate(dx,x,safety);
			dx = utility_ -> create_si_to_uni_mapping2(dx,x,linearVelocityGain,angularVelocityGain);	
			////dx = unicycle_barrier_certificate(dx, x);
			cout<<"dx:\n"<<dx<<endl;
		
			// Set velocities of agents 1:n
			//mode 2
			if(ID_ == roboids[0])
				radio_->sendAllVelocity(dx);
		
			//mode 3
			// for(int i=0;i<n;i++){
				// if(ID_ == roboids[i]){
					// v = dx(0,i); w = dx(1,i);
					// setVelocities(v, w);
				// }
			// }
		}
		delay(delayTime);
	}
}


int TIGERBotMain::TS_UndirectedManeuvering() {
    /* N-agent undirected maneuvering: robots maintain a minimally-rigid
     * formation while tracking a circular reference trajectory.
     * Algorithm by Tairan Liu and Victor Fernandez-Kim.
     * Ported from MATLAB to C++ by Tonmoy Sarker.
     * TO DO: Parking initialization loop is commented out — place robots
     *        at initial positions manually before running. (Mode 3) */
	yield();
	
	int n = numOfRob;	
TRACE();
	// Define timing variables
	Eigen::Matrix<float,10,1> desiredStepTime(10,1); 
	desiredStepTime << 0.14,0.15,0.16,0.18,0.19,0.21,0.22,0.24,0.25,0.27; // Retrieved from TIGERSquare.m Matlab
	float stepTime = desiredStepTime(n-1);  // step time (Specific to TIGERSquare, do not modify)
									/* Note : C/C++ array index starts from 0, 
											  while Matlab array index starts form 1 */


	// Desired formation settings
	float radius = 0.3;
	float omega0 = 0.1;
	float alpha  = 0.03;
	float tol 	 = 1e-3;
	float eps  	 = 1e-5;
	float theta_d_dot_bound  = 0.15;
	MatrixXf Ka(2*n,2*n);   Ka.setIdentity(2*n,2*n);
	MatrixXf C(n,n);        C.setIdentity(n,n);

	// Limits
	float v_limit 	  = 0.035;
	float v_limit_per = 1;
	float w_limit 	  = PI/4;
	float w_limit_per = 1;

	Matrix<float,1,Dynamic> ang(1,n);			ang.fill(0);
	Matrix<float,2,Dynamic> pd(2,n);			pd.fill(0);
	float rd  = 0.225;

	//Declarations
	Matrix<float,3,Dynamic> initial_conditions(3,n); 
    Matrix<float,3,Dynamic> q(3,n); 					q.fill(0);
    Matrix<float,2,Dynamic> p(2,n); 					p.fill(0);
    Matrix<float,1,Dynamic> theta(1,n); 				theta.fill(0);
    Matrix<float,2,Dynamic> dq(2,n); 				    dq.fill(0);

	Matrix<float,Dynamic,Dynamic> e(n,n); 		    e.fill(0);
	Matrix<float,Dynamic,Dynamic> R(2*n-3,2*n); 	R.fill(0);
	VectorXf z(2*n-3);          z.fill(0);
	VectorXf theta_tilde(n); 	theta_tilde.fill(0);
	VectorXf v(n); 				v.fill(0);
	Vector2f vd1; 				vd1.fill(0);
	VectorXf vd(2*n);			vd.fill(0);
	Vector2f vd1_dot; 			vd1_dot.fill(0);
	VectorXf vd_dot(2*n);		vd_dot.fill(0);
	VectorXf u(2*n); 			u.fill(0);

	Matrix<float,Dynamic,Dynamic> R_dot(2*n-3,2*n); 	R_dot.fill(0);
	Matrix<float,Dynamic,Dynamic> B(2*n,2*n); 			B.fill(0);

	VectorXf z_dot(2*n-3);      z_dot.fill(0);
	VectorXf u_dot(2*n);        u_dot.fill(0);
	VectorXf theta_d_dot(n);    theta_d_dot.fill(0);
    VectorXf omega(n);          omega.fill(0);

	for (int ii=0; ii<n ;ii++){
		ang(ii)   = (ii+1)*(2*PI/n);
		pd(0,ii)  = rd*cos(ang(ii));
		pd(1,ii)  = rd*sin(ang(ii));
	}

	MatrixXf d(n,n); d.fill(0);
	for(int i=0; i<n; i++)
		for(int j=0; j<n; j++)
			d(i,j)=(pd.col(i)-pd.col(j)).norm();
	//cout<<"d:\n"<<d<<endl;
TRACE();
	//wrap(x); // necessary ?

	// scalem = diag([0.3 0.3]);
	 // q_init = scalem*[1.2 0.5 -0.5 -0.5  0.5;
					   // 0 0.6  0.6 -0.6 -0.6]+[-0.4;-0.2]+[1.4;1];
	 // initial_conditions = [q_init; zeros(1,N)];
	 /*
	Vector3f translation;	translation << 0.6,0.5,0;
	initial_conditions = utility_ -> getPolygonDims(radius,n,4);
	initial_conditions = initial_conditions + translation;
	*/
/*	
	if(n==3)
	initial_conditions <<
		1.3103,    0.9470,    0.7640,
		0.8363,    1.2081,    0.6525,
		1.0054,    6.0968,    3.0908;
	if(n==4)
	initial_conditions <<
		1.1715,    1.2031,    0.7825,    0.7480,
		0.6667,    1.0789,    1.1380,    0.6746,
		4.3883,    4.1128,    3.6780,    1.5625;
	if(n==6)
	initial_conditions <<
		1.0749,    1.2843,    1.1767,    0.9268,    0.7154,    0.7722,
		0.5875,    0.7772,    1.1218,    1.1777,    0.9650,    0.6794,
		1.7623,    5.4931,    0.7080,    0.9031,    2.0040,    3.9549;
	if(n==7)
	initial_conditions <<
		1.0525,    1.2752,    1.2393,    1.1182,    0.8596,    0.7091,    0.7789,
		0.5786,    0.7261,    0.9510,    1.1680,    1.1782,    0.9100,    0.6422,
		5.8106,    5.6239,    5.9523,    1.8572,    4.0484,    2.0083,    4.9991;
*/	
	MatrixXf Adj(n,n); Adj.fill(0);
	if (n==3){
		Adj << 0,1,1,     // 3 agents
			   1,0,1,
			   1,1,0;
	}
	if (n==4){  
		Adj << 0,1,1,1, 	//4 agents
			   1,0,1,0,
			   1,1,0,1,
			   1,0,1,0;
	}
	if (n==5){
		Adj << 0,1,1,1,1,   // 5 agents
			   1,0,1,0,0,
			   1,1,0,1,0,
			   1,0,1,0,1,
			   1,0,0,1,0;
	}
	if (n==6){
		Adj << 0,1,1,1,1,1, // 6 agents
			   1,0,1,0,0,0,
			   1,1,0,1,0,0,
			   1,0,1,0,1,0,
			   1,0,0,1,0,1,
			   1,0,0,0,1,0;
	}
	if (n==7) {  
		Adj << 0,1,1,1,1,1,1, // 7 agents
			   1,0,1,0,0,0,0,
			   1,1,0,1,0,0,0,
			   1,0,1,0,1,0,0,
			   1,0,0,1,0,1,0,
			   1,0,0,0,1,0,1,
			   1,0,0,0,0,1,0;
	}
	if (n==8) {  
		Adj << 0,1,1,1,1,1,1,1, //8 agents
			   1,0,1,0,0,0,0,0,
			   1,1,0,1,0,0,0,0,
			   1,0,1,0,1,0,0,0,
			   1,0,0,1,0,1,0,0,
			   1,0,0,0,1,0,1,0,
			   1,0,0,0,0,1,0,1,
			   1,0,0,0,0,0,1,0;
	}
	if (n==9) {  
		Adj << 0,1,1,1,1,1,1,1,1, //9 agents
			   1,0,1,0,0,0,0,0,0,
			   1,1,0,1,0,0,0,0,0,
			   1,0,1,0,1,0,0,0,0,
			   1,0,0,1,0,1,0,0,0,
			   1,0,0,0,1,0,1,0,0,
			   1,0,0,0,0,1,0,1,0,
			   1,0,0,0,0,0,1,0,1,
			   1,0,0,0,0,0,0,1,0;
	}
		// Adj << 0,1,0,0,1,		// ??
			   // 1,0,1,0,1,
			   // 0,1,0,1,1,
			   // 0,0,1,0,1,
			   // 1,1,1,1,0;

TRACE();
	/* Grab tools for converting to single-integrator dynamics and ensuring safety */
	float safety = 0.08;
	float lambda = 0.03;//projection_distance

	// Parking
	float PositionError =0.03;
	float RotationError =0.1;

	float v_final =0; float w_final =0; // conflicts declaration of v

/*
	Serial.println("Moving to initial positions...");
	while(!(utility_ -> create_is_initialized(q,initial_conditions,PositionError,RotationError))){
	
		q = getPoses();
		
		delete pose1D; // clear the memory
		
		if(receivedPoses){
			yield();
			dq =  utility_ -> create_automatic_parking_controller2(q,initial_conditions,PositionError,RotationError);
			dq =  utility_ -> create_uni_barrier_certificate(dq,q,safety,lambda);			
		
			//mode 2
			// if(ID_ == roboids[0])
				// radio_->sendAllVelocity(dq);
		
			//mode 3
			for(int i=0;i<n;i++){
				if(ID_ == roboids[i]){
					v_final = dq(0,i); w_final = dq(1,i);
					setVelocities(v_final, w_final);
				}
			}
		}
		delay(delayTime);
	}		
*/
	Serial.print("\nArrived at initial position\n");
	TRACE();
	
	int t=0;
	uint32_t timeStamp = millis();	
	while((millis()-timeStamp) < algorithmRunTime){
		
		yield();
	//iterations = 20000;
	// Iterate for the previously specified number of iterations
	//for (int t = 0; t<iterations; t++){  

		q = getPoses();
		//cout<<"q:\n"<<q<<endl;
		delete pose1D; // clear the memory(to avoid stack overflow in Nodemcu)
		
		
		if(receivedPoses){
			t++;
			cout<<"t:"<<t<<endl;
			p 	  = q.topRows<2>(); // 1st two rows (X & Y)
			theta = q.row(2);
TRACE();
			int ord = 0;
			for(int i=0; i<n; i++){
				for(int j= i+1; j<n; j++){// if und change to j = i+1:n
					e(i,j) = (p.col(i)-p.col(j)).norm()-d(i,j);
					if (Adj(i,j) == 1){
						z(ord)   = e(i,j)*(e(i,j)+2*d(i,j));
						R.row(ord).middleCols<2>(2*i) = (p.col(i)-p.col(j)).transpose();
						R.row(ord).middleCols<2>(2*j) = (p.col(j)-p.col(i)).transpose();
						ord = ord+1;
					}
				}
			}
TRACE();
			VectorXf On(n); 	On.fill(1);
			vd1 << -radius*omega0*sin(omega0*(stepTime*t)),
					radius*omega0*cos(omega0*(stepTime*t));
TRACE();
			//Kronecker Product
			for (int i=0; i<n; i++)
				vd.segment<2>(2*i) = On(i)* vd1; //vd = kron(ones(N,1),vd1);
TRACE();
			vd1_dot << -radius*pow(omega0,2)*cos(omega0*(stepTime*t)),
					   -radius*pow(omega0,2)*sin(omega0*(stepTime*t));
			//Kronecker Product
			for (int i=0; i<n; i++)
				vd_dot.segment<2>(2*i) = On(i)* vd1_dot; //vd_dot = kron(ones(N,1),vd1_dot);

			u = -Ka*(R.transpose()*z)+ vd;
			//cout<<"u:\n"<<u<<endl;
TRACE();			
			float ang_form;
			for (int i=0; i<n; i++){
				if( (u.segment<2>(2*i)).norm() > eps )
					ang_form = atan2(u(2*i+1),u(2*i));
				else
					ang_form = 0;

			   theta_tilde(i) = fmod((theta(i)-ang_form),2*PI);

			   if(theta_tilde(i) > PI)
					theta_tilde(i) = theta_tilde(i) - 2*PI;

				v(i) = (u.segment<2>(2*i)).norm()*cos(theta_tilde(i));

				// Saturation
				v(i) = applyThreshold(v(i), v_limit*v_limit_per);
			}
TRACE();
			for(int i=0; i<n; i++){
				float coeff1 = pow(cos(theta_tilde(i)),2);
				float coeff2 = (sin(2*theta_tilde(i)))/2;
				B.block<2,2>(2*i,2*i) << coeff1,-coeff2,
										 coeff2, coeff1;
			}
TRACE();
			ord = 0; Matrix2f Bi; Matrix2f Bj;
			for(int i=0; i<n; i++){
				for(int j=i+1; j<n; j++){// if und change to j = i+1:n
					if (Adj(i,j) == 1){
						float coeff1 = pow(cos(theta_tilde(i)),2);
						float coeff2 = (sin(2*theta_tilde(i)))/2;
						float coeff3 = pow(cos(theta_tilde(j)),2);
						float coeff4 = (sin(2*theta_tilde(j)))/2;
						Bi << coeff1,-coeff2,
							  coeff2, coeff1;
						Bj << coeff3,-coeff4,
							  coeff4, coeff3;
						R_dot.row(ord).middleCols<2>(2*i) =  (Bi*(u.segment<2>(2*i))-Bj*(u.segment<2>(2*j))).transpose();
						R_dot.row(ord).middleCols<2>(2*j) = -(Bi*(u.segment<2>(2*i))-Bj*(u.segment<2>(2*j))).transpose();
						ord = ord+1;
					}
				}
			}
TRACE();
			z_dot = 2*R*B*u;
			u_dot = -Ka*(R_dot.transpose()*z)-(Ka*R.transpose()*z_dot) + vd_dot;

			Matrix2f H;
			H << 0,-1,
				 1,0;
TRACE();
			for(int i=0;i<n; i++){
				if ( (u.segment<2>(2*i)).norm() > eps )
					theta_d_dot(i) = (u.segment<2>(2*i)).transpose()*H.transpose()*(u_dot.segment<2>(2*i));
					theta_d_dot(i) = theta_d_dot(i)/pow((u.segment<2>(2*i)).norm(),2);

				theta_d_dot(i) = applyThreshold(theta_d_dot(i), theta_d_dot_bound);
			}

			omega = -C*theta_tilde + theta_d_dot;
			for (int j=0; j<n ;j++)
				omega(j) = applyThreshold(omega(j), w_limit*w_limit_per);
TRACE();
			dq << v.transpose(),
				  omega.transpose();
				  
TRACE();
			dq = utility_ -> create_uni_barrier_certificate(dq,q,safety,lambda);
			cout<<"dq:\n"<<dq<<endl;
			
TRACE();		
			//mode 2
			//radio_->sendAllVelocity(dq);
			//if(ID_ == roboids[0])
				//radio_->sendAllVelocity(dq);
			//mode 3
			//if(receivedPoses){
				for(int i=0;i<n;i++){
					if(ID_ == roboids[i]){
						v_final = dq(0,i); w_final = dq(1,i);
						setVelocities(v_final, w_final);
					}
				}
			//}	
			//else 
				//setVelocities(0,0);	
TRACE();			
		}
	}
}

int TIGERBotMain::Directed_Distance_Area_6() {
    /* 6-agent undirected distance-rigidity formation control.
     * Drives robots to a hexagonal lattice using edge tension energy.
     * TO DO: Parking initialization loop is commented out — place robots
     *        at initial_conditions manually before running. (Mode 3) */
	yield();
	
	int n = numOfRob;
	
	// Declarations //	
	/* Initialize velocity vector for agents.  Each agent expects a 2 x 1
	   velocity vector containing the linear and angular velocity, respectively. */
	Eigen::Matrix<float,2,Dynamic> dx(2,n); 			dx.fill(0);
	Eigen::MatrixXf d(n,n); 							d.fill(0);
	Eigen::Matrix<float,Dynamic,4> ad(n-2,4); 			ad.fill(0);
	Eigen::Matrix<float,2,Dynamic> q_desire(2,n); 		q_desire.fill(0);
	
	Eigen::Matrix<float,3,Dynamic> initial_conditions(3,n); 
	
	Eigen::Matrix2i J;
	
	Eigen::Matrix<float,3,Dynamic> x(3,n); 			x.fill(0);
	Eigen::Matrix<float,2,Dynamic> q(2,n); 			q.fill(0);
	Eigen::Matrix<float,2,Dynamic> dxu(2,n); 		dxu.fill(0);
	
	Eigen::VectorXf vr(2*n-3); 						vr.fill(0);
	
	// Define timing variables
	Eigen::Matrix<float,10,1> desiredStepTime(10,1); 
	desiredStepTime << 0.14,0.15,0.16,0.18,0.19,0.21,0.22,0.24,0.25,0.27; // Retrieved from TIGERSquare.m Matlab
	float stepTime = desiredStepTime(n-1);  // step time (Specific to TIGERSquare, do not modify)
									/* Note : C/C++ array index starts from 0, 
											  while Matlab array index starts form 1 */
	
	/* Graph settings*/
	Eigen::Matrix<int,6,6> A(6,6);
	// A << 0,0,0,0,0,0,   // 6 DIRECTED
		 // 1,0,0,0,0,0,
		 // 1,1,0,0,0,0,
		 // 0,1,1,0,0,0,
		 // 0,0,1,1,0,0,
		 // 0,1,0,1,0,0;

	A << 0,1,1,0,0,0,   // 6 UNDIRECTED
		 1,0,1,1,0,1,
		 1,1,0,1,1,0,
		 0,1,1,0,1,1,
		 0,0,1,1,0,0,
		 0,1,0,1,0,0;
	
	float formationRadius = 0.2;
	int desired_output =3; //(q_desire)						 

	//cout << "\nq_desire:\n"<<q_desire <<endl;
	 // COLLINEAR
	 // initial_conditions = .2*[1 2 3]+.1;  
	 // initial_conditions = [initial_conditions; .75*ones(1,3); pi/2*ones(1,3)];

/*	q_desire << -2,-1,0,1,2,0,
				-sqrt(3),0,-sqrt(3),0,-sqrt(3),sqrt(3);
	q_desire = 	formationRadius	* q_desire;		*/

	q_desire << 0,1,2,3,4,2,
				0,sqrt(3),0,sqrt(3),0,2*sqrt(3);
	q_desire = 	formationRadius	* q_desire;	
	
	// q_desire = R*q_desire + +[0.55;0.7];
	
	/*
	COLLINEAR
	initial_conditions << 0.2*[1 2 3 4 5 6]+.1,  
						  0.75*ones(1,6), 
						  pi/2*ones(1,6)];

	REFLECTED - doesn't work, arena too small
	initial_conditions = [1*[1 0;0 -1]*q_desire-0.4*[0;1]*ones(1,N);
	    pi/2*ones(1,N)];
	initial_conditions = initial_conditions + [0;1.8;0];

	RECTANGLE
	initial_conditions = 0.35*[-1 1 0 0 -1 1; 0 0 0 -1 -1 -1]+[.8;.8];
	initial_conditions = [initial_conditions;pi/2*ones(1,N)];

	HEXAGON
	[~,~,initial_conditions,~] = getPolygonDims(0.35,6,0);
	initial_conditions(:,[3 6]) = initial_conditions(:,[6 3]);
	initial_conditions = [initial_conditions;zeros(1,N)];
	initial_conditions = initial_conditions + [0;0.1;0];					 
*/
	// Control Gains
	int Kr = 5; // was 20
	int Ka = 5;

	for(int ii=0;ii<n;ii++)
		for(int jj=0;jj<n;jj++)	
			d(ii,jj)=(q_desire.col(ii)-q_desire.col(jj)).norm();
	cout<<"d:\n"<<d<<endl;
	
	/* Precompute area constraint table (ad) for triangle triplets in the graph */
	Eigen::Matrix3f temp_mat;
	int ord = 0;
	for (int ii = 0; ii<n; ii++){
      for (int jj = 0; jj<n; jj++){
         for (int kk = 0; kk<n; kk++){
            if (A(ii,jj) == 1 && A(ii,kk) == 1 && A(kk,jj) == 1){     //
                ord = ord+1;
				
				temp_mat<<1,1,1;
					      q_desire(1,ii),q_desire(1,jj),q_desire(1,kk),
						  q_desire(2,ii),q_desire(2,jj),q_desire(2,kk);
                
				float temp_area = 1/2*temp_mat.determinant(); //
                if (temp_area > 0) 
					ad.row(ord)<< ii+1,jj+1,kk+1,temp_area;  //
                if (temp_area < 0) 
					ad.row(ord)<<ii+1,kk+1,jj+1,-temp_area; //
            }
         }
      }
    }

	J << 0,1,
		-1,0;

	/* Grab tools for converting to single-integrator dynamics and ensuring safety */
	//Gains for the transformation from single-integrator to unicycle dynamics
	float linearVelocityGain = 1;
	float angularVelocityGain = PI/2;
	float formationControlGain = 4;

	float safety = 0.12;
	float lambda = 0.05;//projection_distance

	// Parking
	float PositionError =0.01;
	float RotationError =0.1;
	float v =0; float w =0;

/*
	Serial.println("Moving to initial positions...");
	while(!(utility_ -> create_is_initialized(x,initial_conditions,PositionError,RotationError))){
	
		x = getPoses();
		
		delete pose1D; // clear the memory
		
		if(receivedPoses){
			yield();
			dxu =  utility_ -> create_automatic_parking_controller2(x,initial_conditions,PositionError,RotationError);
			dxu =  utility_ -> create_uni_barrier_certificate(dxu,x,safety,lambda);			
			//dxu = create_si_barrier_certificate(dxu,x,safety);
		
			//mode 2
			// if(ID_ == roboids[0])
				// radio_->sendAllVelocity(dxu);
		
			//mode 3
			for(int i=0;i<n;i++){
				if(ID_ == roboids[i]){
					v = dxu(0,i); w = dxu(1,i);
					setVelocities(v, w);
				}
			}
		}
		delay(delayTime);
	}		
*/

	// Iterate for the previously specified number of iterations
	Serial.println("Moving to formation...");
	int t=0;
	uint32_t timeStamp = millis();
	while((millis()-timeStamp) < algorithmRunTime){
		x = getPoses();

		delete pose1D; // clear the memory(to avoid stack overflow in Nodemcu)
	
		if(receivedPoses){	
			t++; // its a iteration counter. used in the Maneuvering algorithms
			
			// This section contains the actual algorithm for formation control!
			q = x.topRows<2>();
			
			Eigen::VectorXf u(2*n); 							u.fill(0);   
			Eigen::Matrix<float,Dynamic,Dynamic> e(n,n); 		e.fill(0);
			Eigen::Matrix<float,Dynamic,Dynamic> R(2*n-3,2*n); 	R.fill(0);
			Eigen::VectorXf e2n(2*n-3); 						e2n.fill(0);
			
			ord = 0; float temp =0;
			for(int i=0; i<n; i++){
				for(int j=i+1; j<n; j++){// if und change to j = i+1:n
					temp =((q.col(i)-q.col(j)).transpose())*(q.col(i)-q.col(j));
					e(i,j) = sqrt(temp)-d(i,j);
					if (A(i,j) == 1){
						e2n(ord) = e(i,j)*(e(i,j)+2*d(i,j));
						R.row(ord).middleCols<2>(2*i) = (q.col(i)-q.col(j)).transpose();
						R.row(ord).middleCols<2>(2*j) =-(q.col(i)-q.col(j)).transpose();						
						ord = ord+1;
					}
				}
			}			
		
			vr = -Kr*(R.transpose()*e2n); // ( R'[2N*2N-3] x e2n[2N-3*1] )
			
			// TO DO.commented out in matlab
			// Eigen::VectorXf va(2*n); va.fill(0);
			// Eigen::Matrix<float,Dynamic,Dynamic> temp_nodes; temp_nodes.fill(0);
			// Eigen::Matrix<float,Dynamic,Dynamic> temp_a; temp_a.fill(0);
			// TODO
			// for (int i=0;i<(n-2);i++){
			//         temp_nodes = uint16(ad.row(i).leftCols(3));
			//         temp_a = 1/2*det([1 1 1;q(:,temp_nodes)]);
			//         va(2*temp_nodes(1)-1:2*temp_nodes(1)) = - Ka*(temp_a - ad(i,4))*J*(q(:,temp_nodes(2))-q(:,temp_nodes(3)));
			//     }

			u = vr; //+ va + vm;0
			
			Map<MatrixXf> dx(u.data(),2,n);   //dx = reshape(u,2,[]);
			
			// Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
			dx = utility_ -> create_si_barrier_certificate(dx,x,safety);
			dx = utility_ -> create_si_to_uni_mapping2(dx,x,linearVelocityGain,angularVelocityGain);	
			cout<<"dx:\n"<<dx<<endl;
		
			// Set velocities of agents 1:n
			//mode 2
			// if(ID_ == roboids[0])
				// radio_->sendAllVelocity(dx);
		
			//mode 3
			for(int i=0;i<n;i++){
				if(ID_ == roboids[i]){
					v = dx(0,i); w = dx(1,i);
					setVelocities(v, w);
				}
			}
		}
		delay(delayTime);
	}
}
