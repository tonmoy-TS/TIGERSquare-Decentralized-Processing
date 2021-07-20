/*
 -------------------------------------------------------------------------------
 TIGERBot Motor Board class

 Initially created by Daniel Pickem (2014).
 Modified by Victor Fernandez-Kim (2018).
 -------------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "TIGERBot_Motor.h"

/* Initializing wheel and Motor related parameters. Units are in m */
const float TIGERBotMotor::rWheel_ = 0.0185;  // Radius of wheel			//pvt.v2
const float TIGERBotMotor::cWheel_ = 0.1162;  // Circumference of wheel		//pvt.v3
const float TIGERBotMotor::rTrack_ = 0.045;   // Radius of wheel base		//pvt.v4
const float TIGERBotMotor::cTrack_ = 0.2827;  //Circumference of wheel base	//pvt.v5

// Constructors
TIGERBotMotor::TIGERBotMotor() {}

// Destructor
TIGERBotMotor::~TIGERBotMotor() {}

//------------------------------------------------------------------------------
// Initialization function
//------------------------------------------------------------------------------
void TIGERBotMotor::initialize(uint8_t address) {							/*01*/
  /* Declare the motor pins as outputs */
  DDRB  = 0xFF;   /* Set all pins on port B to output */
  DDRD  = 0xFF;   /* Set all pins on port D to output */
  PORTB = 0x00;   /* Set all pins on port B to low */
  PORTD = 0x00;   /* Set all pins on port D to low */

  /* Set up I2C slave with address 2 */
  i2c_ = I2CInterface();								//
  if(address == 2) { i2c_.initialize(address);} 
  else { i2c_.initialize();}

  /* Register onRequest and onReceive events in the setup function of
   * the main file. See the example files for this library */

  /* Initialize last step times */
  lastStepLeft_       = micros();		    //v17
  lastStepRight_      = micros();		    //v18

  /* Pull reset pin for ESP8266 high to avoid a reset
     VFK: commented out
	 */
  //sbi(MAINBOARD_RESET_PORT, MAINBOARD_RESET_PIN);

  /* Set velocity thresholds */
  
  /* Full speed settings */
  setVelocitiesMax(.035, 45); // (m/s) and (deg/s)				/*08*/
  setRPSMax(.3);               // (RPS)							/*10*/
	
  // setVelocitiesMax(0.1, 45); // (m/s) and (deg/s)
  // setRPSMax(.87);            // (RPS)

  /* Conservative settings */
  // setVelocitiesMax(0.1, 360);
  // setRPSMax(4);

  /* Set initial steps per revolution to 50 */
  setStepsPerRevolution(STEPS_PER_REVOLUTION);					/*11*///defined

  /* Set firmware and hardware version */
  //setFirmwareVersion(FIRMWARE_VERSION);
  
  /* Limit the steps in between 1 and 8 */
	curStepLeft_ = constrain(curStepLeft_,1,8);			/*pvt.v15*/
	curStepRight_= constrain(curStepRight_,1,8);		/*pvt.v16*/
}

//------------------------------------------------------------------------------
// Velocity control functions
//------------------------------------------------------------------------------
void TIGERBotMotor::step() {									/*02*/
  stepLeft();									/*pvt.f1*/
  stepRight();									/*pvt.f3*/

  /* Enable sleep if necessary */
  if(sleepNow_) {								/*pvt.v20*/
    enableDeepSleep(sleepDuration_);							/*22*//*pvt.v19*/
    sleepNow_ = false;
  }
}

/* Set functions */
void TIGERBotMotor::setVelocities(float v, float w) {			/*07*/
  /* Input: v in [m / sec]
   *        w in [deg / sec]
   * NOTE: Limits velocities so that max RPS are obeyed.
   */

  /* Store requested velocities */
  v_ = v;										/*pvt.v9*/
  w_ = w;										/*pvt.v10*/

  /* Saturate velocities based on vMax_ and wMax_ */
  saturateVelocities();							/*pvt.f10*/

  /* Compute required rotations per second */
  float rpsLeftLinear  = v_ / cWheel_;
  float rpsRightLinear = v_ / cWheel_;
  float rpsLeftRotation  = -(cTrack_ / cWheel_) * (w_ / 360.0);
  float rpsRightRotation =  (cTrack_ / cWheel_) * (w_ / 360.0);
  rpsLeft_  = rpsLeftLinear + rpsLeftRotation;					/*pvt.v7*/
  rpsRight_ = rpsRightLinear + rpsRightRotation;				/*pvt.v8*/

  /* Set RPS values */
  setRPS(rpsLeft_, rpsRight_);									/*09*/
	// Serial.print("velocities: ");
	// Serial.print(rpsLeft_);
	// Serial.print(", ");
	// Serial.println(rpsRight_);
	
  /* Add measurement data every time velocities are update */
  collectData();												/*27*/
}

void TIGERBotMotor::setVelocitiesMax(float vMax, float wMax){	/*08*/
  /* NOTE: v ... [m/sec]
   *       w ... [deg/sec]
   */
  vMax_ = vMax;									/*pvt.v11*/
  wMax_ = wMax;									/*pvt.v12*/
}

void TIGERBotMotor::setRPS(float rpsLeft, float rpsRight) {		/*09*/
  rpsLeft_  = rpsLeft;							/*pvt.v7*/
  rpsRight_ = rpsRight;							/*pvt.v8*/

  /* Vector scale rps values if they exceed maximum allowed rps */
  float maxRpsCurrent = max(abs(rpsLeft_), abs(rpsRight_));
  if(maxRpsCurrent > rpsMax_) {
    float scale = rpsMax_ / maxRpsCurrent;
    rpsLeft_  *= scale;
    rpsRight_ *= scale;
  }
  
  /* Set delay variables in microseconds */
  if(abs(rpsLeft_) < 1E-4) {
    delayLeft_  = 1E6; /* 1 second delay per step if speed is close to 0 */	//pvt.v13
    rpsLeft_    = 0.0;
  } else {
    delayLeft_  = (1000000.0 / (fabs(rpsLeft_) * stepsPerRevolution_));
  }
  if(abs(rpsRight_) < 1E-4) {
    delayRight_ = 1E6;  /* 1 second delay per step if speed is close to 0 *///pvt.v14
    rpsRight_   = 0.0;
  } else {
    delayRight_ = (1000000.0 / (fabs(rpsRight_) * stepsPerRevolution_));
  }
}

void TIGERBotMotor::setRPSMax(float rpsMax) {					/*10*/
  rpsMax_ = rpsMax;								/*pvt.v1*/
}

void TIGERBotMotor::setStepsPerRevolution(uint16_t steps) {		/*11*/
  stepsPerRevolution_ = steps;					/*pvt.v6*/
}

void TIGERBotMotor::saturateVelocities() {						/*pvt.f10*/
  float scaleV;
  float scaleW;

  if(abs(v_) > abs(vMax_)) {
    scaleV = abs(vMax_ / v_);
    v_ *= scaleV;
    w_ *= scaleV;
  }

  if(abs(w_) > abs(wMax_)) {
    scaleW = abs(wMax_ / w_);
    v_ *= scaleW;
    w_ *= scaleW;
  }
}

//------------------------------------------------------------------------------
// I2C communication functions
//------------------------------------------------------------------------------
void TIGERBotMotor::requestEvent() {							/*04*/
  /* A request event requires a previous receive event to have occured
   * that sets the message type.
   */
  switch(I2CBuffer_.msgType_) {
    case(MSG_GET_VELOCITIES):
      i2c_.sendMessage(MSG_GET_VELOCITIES, v_, w_);
      break;
    case(MSG_GET_VELOCITIES_MAX):
      i2c_.sendMessage(MSG_GET_VELOCITIES_MAX, vMax_, wMax_);
      break;
    case(MSG_GET_RPS):
      i2c_.sendMessage(MSG_GET_RPS, rpsLeft_, rpsRight_);
      break;
    case(MSG_GET_RPS_MAX ):
      i2c_.sendMessage(MSG_GET_RPS_MAX, rpsMax_, rpsMax_);
      break;
    case(MSG_GET_AVG_RPS):
      i2c_.sendMessage(MSG_GET_AVG_RPS, rpsLeftAvg_.getAverage(),
                                        rpsRightAvg_.getAverage());
      break;
    case(MSG_GET_AVG_TEMPERATURES):
      i2c_.sendMessage(MSG_GET_AVG_TEMPERATURES, tempLeftAvg_.getAverage(),
                                                 tempRightAvg_.getAverage());
      break;
    case(MSG_GET_AVG_CURRENTS):
      i2c_.sendMessage(MSG_GET_AVG_CURRENTS, currentLeftAvg_.getAverage(),
                                             currentRightAvg_.getAverage());
      break;
    //case(MSG_GET_FIRMWARE_VERSION):
      //i2c_.sendMessage(MSG_GET_FIRMWARE_VERSION, getFirmwareVersion(), 0.0);
      //break;
    //case(MSG_GET_HARDWARE_VERSION):
      //i2c_.sendMessage(MSG_GET_FIRMWARE_VERSION, getHardwareVersion(), 0.0);
      //break;
    case(MSG_ECHO):
      i2c_.sendMessage(MSG_ECHO, I2CBuffer_.data_[0].fval,
                                 I2CBuffer_.data_[1].fval);
      break;
    default:
      break;
  }

  /* Clear buffer after received message */
  I2CBuffer_.clear(); //

  /* Visual output */
  //toggleLeds();										/*12*/
}

void TIGERBotMotor::receiveEvent() {							/*05*/
  /* Receive 1 uint8_t value and 2 or 3 float values
   * representing the following
   * 1 ... message type [uint8_t]
   * 2 ... data field 1 [float]
   * 3 ... data field 2 [float]
   * 4 ... data field 3 [float] (might not be necessary)
   */

  /* Receive and process message */
  if(i2c_.receiveMessage(&I2CBuffer_)) {
    //I2CBuffer_.print();
    processI2CMessage(&I2CBuffer_);						/*06*/

    /* Visual feedback */
    //toggleLeds();										/*12*/
  }
}

void TIGERBotMotor::processI2CMessage(I2CMessage* msg) {		/*06*/
  switch(msg->msgType_) {
    case(MSG_SET_VELOCITIES):
      /* Set linear velocity v and rotational velocity w */
      setVelocities(msg->data_[0].fval, msg->data_[1].fval);
      break;
    case(MSG_SET_RPS):
      /* Set motor rps values for left and right motor */
      setRPS(msg->data_[0].fval, msg->data_[1].fval);
      break;
    case(MSG_SET_VELOCITIES_MAX):
      setVelocitiesMax(msg->data_[0].fval, msg->data_[1].fval);
      break;
    case(MSG_SET_RPS_MAX):
      setRPSMax(msg->data_[0].fval);
      break;
    case(MSG_SET_STEPS_PER_REV):
      setStepsPerRevolution(msg->data_[0].fval);
      break;
    case(MSG_DEEP_SLEEP):
      /* NOTE: The microcontroller is put to sleep in the step() function,
       *       since the chip just freezes if sleep is enabled in the I2C
       *       callback routine.
       */
      sleepDuration_ = msg->data_[0].fval;
      sleepNow_      = true;

      /* Wake up (motor and main board) after sleepTime sec */
      if(sleepDuration_ < 0) {
        /* Sleep for 10 seconds if no sleep time is provided */
        sleepDuration_ = 10;
      }
      break;
    default:
      break;
  }
}

//------------------------------------------------------------------------------
// LED functions
//------------------------------------------------------------------------------
void TIGERBotMotor::toggleLeds() {								/*11*/
  toggleLedLeft();
  toggleLedRight();
}

void TIGERBotMotor::toggleLedLeft() {							/*13.a*/
  if(ledStateLeft_) {
    cbi(LED_LEFT_PORT, LED_LEFT_PIN);
  } else {
    sbi(LED_LEFT_PORT, LED_LEFT_PIN);
  }
  ledStateLeft_ = !ledStateLeft_;
}

void TIGERBotMotor::toggleLedRight() {							/*13.b*/
  if(ledStateRight_) {
    cbi(LED_RIGHT_PORT, LED_RIGHT_PIN);
  } else {
    sbi(LED_RIGHT_PORT, LED_RIGHT_PIN);
  }
  ledStateRight_ = !ledStateRight_;
}

void TIGERBotMotor::ledsOn() {									/*14*/
  ledOnLeft();
  ledOnRight();
}

void TIGERBotMotor::ledsOff() {									/*15*/
  ledOffLeft();
  ledOffRight();
}

void TIGERBotMotor::ledOnLeft(){								/*16*/
  sbi(LED_LEFT_PORT, LED_LEFT_PIN);
  ledStateLeft_ = true;
}

void TIGERBotMotor::ledOnRight(){								/*17*/
  sbi(LED_RIGHT_PORT, LED_RIGHT_PIN);
  ledStateRight_ = true;
}

void TIGERBotMotor::ledOffLeft(){								/*18*/
  cbi(LED_LEFT_PORT, LED_LEFT_PIN);
  ledStateLeft_ = false;
}

void TIGERBotMotor::ledOffRight(){								/*19*/
  cbi(LED_RIGHT_PORT, LED_RIGHT_PIN);
  ledStateRight_ = false;
}

//------------------------------------------------------------------------------
// Motor control functions
//------------------------------------------------------------------------------
void TIGERBotMotor::nextStepLeft() {							/*pvt.f5*/
  if(rpsLeft_ < 0) {
    /* forward */
    curStepLeft_--;								/*pvt.v15*/

    /* Step count has to be in [1 .. 8] */
    if(curStepLeft_ < 1) { curStepLeft_ = 8; }
  } else {
    /* backward */
    curStepLeft_++;								/*pvt.v15*/

    /* Step count has to be in [1 .. 8] */
    if(curStepLeft_ > 8) { curStepLeft_ = 1; }
  }
  /* Visual Feedback for motion */
  toggleLeds();											/*12*/
}

void TIGERBotMotor::nextStepRight() {							/*pvt.f6*/
  if(rpsRight_ > 0) {
    /* forward */
    curStepRight_--;

    /* Step count has to be in [1 .. 8] */
    if(curStepRight_ < 1) { curStepRight_ = 8; }
  } 
  else {
    /* backward */
    curStepRight_++;

    /* Step count has to be in [1 .. 8] */
    if(curStepRight_ > 8) { curStepRight_ = 1; }
  }
  /* Visual Feedback for motion */
  toggleLeds();										/*12*/
}

void TIGERBotMotor::stepLeft() {								/*pvt.f1*/

  if(rpsLeft_ == 0) {
    stopMotorLeft();							/*pvt.f8*/
  } 
  else {
		// Serial.print("L: "); Serial.print(delayLeft_); Serial.print(", ");
		// Serial.print(curStepLeft_); Serial.print(", "); Serial.println(rpsLeft_);
	
	/* Check if more than delayLeft_ time has passed since last step */
    if( (micros() - lastStepLeft_) >= delayLeft_ ){
      /* Update lastStepLeft_ variable */
      lastStepLeft_ = micros();

      /* Execute step
       * NOTE: This switch-statement executes the step sequence required
       * 	   for bipolar stepper motors with 8 steps per step sequence.
       */

	  switch(curStepLeft_) { //CW step sequence				/*pvt.v15*/
        case 1: //Orange
          sbi(M11_PORT, M11_PIN);
          cbi(M12_PORT, M12_PIN);
          cbi(M13_PORT, M13_PIN);
          cbi(M14_PORT, M14_PIN);
          break;
        case 2: //Orange Yellow
          sbi(M11_PORT, M11_PIN);
          cbi(M12_PORT, M12_PIN);
          sbi(M13_PORT, M13_PIN);
          cbi(M14_PORT, M14_PIN);
          break;
        case 3: //Yellow
          cbi(M11_PORT, M11_PIN);
          cbi(M12_PORT, M12_PIN);
          sbi(M13_PORT, M13_PIN);
          cbi(M14_PORT, M14_PIN);
          break;
        case 4: //Yellow Pink
          cbi(M11_PORT, M11_PIN);
          sbi(M12_PORT, M12_PIN);
          sbi(M13_PORT, M13_PIN);
          cbi(M14_PORT, M14_PIN);
          break;
        case 5: //Pink
          cbi(M11_PORT, M11_PIN);
          sbi(M12_PORT, M12_PIN);
          cbi(M13_PORT, M13_PIN);
          cbi(M14_PORT, M14_PIN);
          break;
        case 6: //Pink Blue
          cbi(M11_PORT, M11_PIN);
          sbi(M12_PORT, M12_PIN);
          cbi(M13_PORT, M13_PIN);
          sbi(M14_PORT, M14_PIN);
          break;
        case 7: //Blue
          cbi(M11_PORT, M11_PIN);
          cbi(M12_PORT, M12_PIN);
          cbi(M13_PORT, M13_PIN);
          sbi(M14_PORT, M14_PIN);
          break;
        case 8: //Orange Blue
          sbi(M11_PORT, M11_PIN);
          cbi(M12_PORT, M12_PIN);
          cbi(M13_PORT, M13_PIN);
          sbi(M14_PORT, M14_PIN);
          break;
        default:
          cbi(M11_PORT, M11_PIN);
          cbi(M12_PORT, M12_PIN);
          cbi(M13_PORT, M13_PIN);
          cbi(M14_PORT, M14_PIN);
          break;
      }

      /* Update current step */
      nextStepLeft();							/*pvt.f5*/
    }
  }
}

void TIGERBotMotor::stepRight() {							/*pvt.f3*/

  if(rpsRight_ == 0) {
    stopMotorRight();							/*pvt.f9*/

  } else {
		// Serial.print("R: "); Serial.print(delayRight_); Serial.print(", ");
		// Serial.print(curStepRight_); Serial.print(", "); Serial.println(rpsRight_);
		/* Check if more than delayRight_ time has passed since last step */
    if( (micros() - lastStepRight_) >= delayRight_ ) {
      /* Update lastStepLeft_ variable */
      lastStepRight_ = micros();				/*pvt.v18*/

      /* Execute step
       * NOTE: This switch-statement executes the step sequence required
       * 	   for bipolar stepper motors with 8 steps per step sequence.
       */
	  switch(curStepRight_) {								/*pvt.v16*/
        case 1:
          sbi(M21_PORT, M21_PIN);
          cbi(M22_PORT, M22_PIN);
          cbi(M23_PORT, M23_PIN);
          cbi(M24_PORT, M24_PIN);
          break;
        case 2:
          sbi(M21_PORT, M21_PIN);
          cbi(M22_PORT, M22_PIN);
          sbi(M23_PORT, M23_PIN);
          cbi(M24_PORT, M24_PIN);
          break;
        case 3:
          cbi(M21_PORT, M21_PIN);
          cbi(M22_PORT, M22_PIN);
          sbi(M23_PORT, M23_PIN);
          cbi(M24_PORT, M24_PIN);
          break;
        case 4:
          cbi(M21_PORT, M21_PIN);
          sbi(M22_PORT, M22_PIN);
          sbi(M23_PORT, M23_PIN);
          cbi(M24_PORT, M24_PIN);
          break;
        case 5:
          cbi(M21_PORT, M21_PIN);
          sbi(M22_PORT, M22_PIN);
          cbi(M23_PORT, M23_PIN);
          cbi(M24_PORT, M24_PIN);
          break;
        case 6:
          cbi(M21_PORT, M21_PIN);
          sbi(M22_PORT, M22_PIN);
          cbi(M23_PORT, M23_PIN);
          sbi(M24_PORT, M24_PIN);
          break;
        case 7:
          cbi(M21_PORT, M21_PIN);
          cbi(M22_PORT, M22_PIN);
          cbi(M23_PORT, M23_PIN);
          sbi(M24_PORT, M24_PIN);
          break;
        case 8:
          sbi(M21_PORT, M21_PIN);
          cbi(M22_PORT, M22_PIN);
          cbi(M23_PORT, M23_PIN);
          sbi(M24_PORT, M24_PIN);
          break;
        default:
          cbi(M21_PORT, M21_PIN);
          cbi(M22_PORT, M22_PIN);
          cbi(M23_PORT, M23_PIN);
          cbi(M24_PORT, M24_PIN);
          break;
      }

      /* Update current step */
      nextStepRight();								/*pvt.f6*/
    }
  }
}

bool TIGERBotMotor::isStopped() {					/*pvt.f7*/
  float threshold = 1E-4;

  if(abs(rpsLeft_) < threshold && abs(rpsRight_) < threshold) {		/*pvt.v7,v8*/
    return true;
  } else {
    return false;
  }
}

void TIGERBotMotor::stopMotors() {							/*03*/
  stopMotorLeft();									/*pvt.f8*/
  stopMotorRight();									/*pvt.f9*/
}

void TIGERBotMotor::stopMotorLeft() {					/*pvt.f8*/
  //Serial.println("Motor left stopped");
  cbi(M11_PORT, M11_PIN);
  cbi(M12_PORT, M12_PIN);
  cbi(M13_PORT, M13_PIN);
  cbi(M14_PORT, M14_PIN);

  rpsLeft_   = 0.0;
  delayLeft_ = 1E6; /* 1 second delay per step if speed is close to 0 */
}

void TIGERBotMotor::stopMotorRight() {					/*pvt.f9*/
  //Serial.println("Motor right stopped");
  cbi(M21_PORT, M21_PIN);
  cbi(M22_PORT, M22_PIN);
  cbi(M23_PORT, M23_PIN);
  cbi(M24_PORT, M24_PIN);

  rpsRight_   = 0.0;
  delayRight_ = 1E6; /* 1 second delay per step if speed is close to 0 */
}

//------------------------------------------------------------------------------
// Sleep functions
//------------------------------------------------------------------------------
void TIGERBotMotor::enableDeepSleep() {						/*21*/
  /* NOTE: This function activates deep sleep without any
   *       wakeup interrupts. This function is meant to be used
   *       as a last resort when battery voltage drops too far.
   * NOTE: Enabling deep sleep mode reduces power consumption of the
   *       motor board to ~5 mA
   * ATTENTION: ONLY A POWER CYCLE OF THE ROBOT WILL WAKE THE MOTOR
   *            BOARD UP AGAIN.
   */
  /* Set a power saving mode from 5 different available modes:
   * SLEEP_MODE_IDLE      - the least power savings
   * SLEEP_MODE_ADC
   * SLEEP_MODE_PWR_SAVE
   * SLEEP_MODE_STANDBY
   * SLEEP_MODE_PWR_DOWN  - the most power savings
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();									//

  /* Power off LED */
  ledsOff();								/*15*/

  /* Power down motors */
  stopMotors();								/*03*/

  /* Put the device to sleep. The program would continue execution from
   * here, if an interrupt triggered a wake up.
   */
  sleep_mode();										//
}

void TIGERBotMotor::enableDeepSleep(uint32_t sec) {			/*22*/
  /* Use watchdog timer to sleep multiples of 8 sec at a time
   * Note: See http://forum.arduino.cc/index.php?topic=173850.0
   */
  /* Compute sleep times */
  uint16_t i;
  uint16_t cycles8Sec = round(sec / 8);
  uint16_t cycles1Sec = sec % 8;

  /* Activate sleep in multiples of 8 seconds */
  for (i = 0; i < cycles8Sec; i++) {
    sleep8Sec();
  }

  /* Activate sleep in multiples of 1 second */
  for (i = 0; i < cycles1Sec; i++) {
    sleep1Sec();
  }

  /* Wake up main board (through a reset) after motor board wakes up */
  //resetMainBoard();
}

void TIGERBotMotor::sleep1Sec() {							/*23*/
  sleepNSec(0b000110);
}

void TIGERBotMotor::sleep8Sec() {							/*24*/
  sleepNSec(0b100001);
}

void TIGERBotMotor::sleepNSec(const byte interval) {		/*25*/
  // sleep bit patterns:
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001

  /* Configure watch dog timer
   * NOTE: This timer can measure the longest duration (8 sec)
   */
  MCUSR   = 0;                      // reset various flags
  WDTCSR |= 0b00011000;             // see docs, set WDCE, WDE
  WDTCSR  = 0b01000000 | interval;  // set WDIE, and appropriate delay

  /* Reset watchdog */
  wdt_reset();

  /* Power off LED */
  ledsOff();									/*15*/

  /* Power down motors */
  stopMotors();

  /* Set sleep parameters */
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);

  /* Enable sleep mode and wait for watchdog interrupt */
  sleep_mode();
}

/*
void TIGERBotMotor::resetMainBoard() {						/*26*/
  /* Wake up the main board through a HIGH - LOW - HIGH pulse on pin PD3
   *
   * NOTE: Pin PD3 is only connected to the RESET line of the ESP8266 on the
   *       newest robots.
   
  uint32_t pulseWidth = 500;
  sbi(MAINBOARD_RESET_PORT, MAINBOARD_RESET_PIN);
  delay(pulseWidth);
  cbi(MAINBOARD_RESET_PORT, MAINBOARD_RESET_PIN);
  delay(pulseWidth);
  sbi(MAINBOARD_RESET_PORT, MAINBOARD_RESET_PIN);
}
*/

/**********************************************************************/

void TIGERBotMotor::collectData() {							/*27*/
  /* This function measures inputs on the following ADC channels
   * A0 ... current left
   * A1 ... current right
   * A2 ... temperature left
   * A3 ... temperature right
   *
   * Note: Currents measured according to
   * www.diodes.com/_files/datasheets/ZXCT1009.pdf
   */
  /* Update average motor velocities */

  /* (PAUL): TODO: Add NaN check to all values
  */

  rpsLeftAvg_.addData(rpsLeft_);
  rpsRightAvg_.addData(rpsRight_);

  /* Update average motor temperatures */
  /* 1. Measure ADC voltages */
  uint32_t tempL_raw = analogRead(A2);
  uint32_t tempR_raw = analogRead(A3);

  /* 2. Map raw voltage data to 0 - 5 V*/
  float tempLV = tempL_raw * 3.3; tempLV /= 1024.0;
  float tempRV = tempR_raw * 3.3; tempRV /= 1024.0;

  /* 3. Use linear approximation according to datasheet */
  float tempL = (tempLV - 0.5) * 100;
  float tempR = (tempRV - 0.5) * 100;

  /* 4. Add data to averaging filter */
  tempLeftAvg_.addData(tempL);
  tempRightAvg_.addData(tempR);

  /* Update average motor currents */
  /* 1. Measure ADC voltages */
  uint32_t currL_raw = analogRead(A0);
  uint32_t currR_raw = analogRead(A1);

  /* 2. Map raw voltage data to 0 - 5 V*/
  float currLV = currL_raw * 3.3; currLV /= 1024.0;
  float currRV = currR_raw * 3.3; currRV /= 1024.0;

  /* 3. Use linear approximation according to datasheet */
  float cL = currLV/4.7;
  float cR = currRV/4.7;
  
  /* 4. Add data to averaging filter
   * Note: The multiplier of 10000 stems from Rout being 10 k (see schematic)
   */
  currentLeftAvg_.addData(cL * 10000);
  currentRightAvg_.addData(cR * 10000);
}
																	/*pvt.f10*/
float TIGERBotMotor::map(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/* EEPROM Functions
bool TIGERBotMotor::setFirmwareVersion(uint32_t version) {			//28//
  uint8_t i = EEPROM_writeAnything(FIRMWARE_ADDRESS, version);

  if(i > 0) {
    return true;
  } else {
    return false;
  }
}

bool TIGERBotMotor::setHardwareVersion(uint32_t version) {			//29//
  uint8_t i = EEPROM_writeAnything(HARDWARE_ADDRESS, version);

  if(i > 0) {
    return true;
  } else {
    return false;
  }
}

uint32_t TIGERBotMotor::getFirmwareVersion() {						//30//
  uint32_t version;
  uint8_t i = EEPROM_readAnything(FIRMWARE_ADDRESS, version);

  if(i > 0) {
    return version;
  } else {
    return 0;
  }
}

uint32_t TIGERBotMotor::getHardwareVersion() {						//31//
  uint32_t version;
  uint8_t i = EEPROM_readAnything(HARDWARE_ADDRESS, version);

  if(i > 0) {
    return version;
  } else {
    return 0;
  }
}
*/
