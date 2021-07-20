/**
 * @file    TIGERBot_Utility.h
 * @brief   Utility class interface for the TIGERBot fleet.
 *
 * Declares TIGERBotUtility, providing barrier certificates,
 * kinematic transformations, initialization checking, automatic
 * parking control, and polygon initial-condition generation
 * for multi-robot experiments on the TIGERSquare platform.
 *
 * Original Matlab version created by the Robotarium team (Georgia Tech).
 * TIGERSquare version created and modified by Victor Fernandez-Kim (2018).
 * Ported and extended for ESP8266 by Tonmoy Sarker (2021).
 */

#ifndef _TIGERBOT_UTILITY_h_
#define _TIGERBOT_UTILITY_h_

//------------------------------------------------------------------
// Defines
//------------------------------------------------------------------

//------------------------------------------------------------------
// Includes
//------------------------------------------------------------------
/* Include basic Arduino libraries */
#include <ArduinoTrace.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#include <iostream>
//#include<ctime>				//srand()
#include<cstdlib>

#define _USE_MATH_DEFINES
#include <cmath>  			// M_PI and trigonometric functions

using namespace std;

//#include <ArduinoSTL.h>
#include "Eigen313.h"
using namespace Eigen;

#include "eiquadprog.hpp"
#define PI 3.1416

class TIGERBotUtility {
	
  //----------------------------------------------------------------
  // Public Member Functions
  //----------------------------------------------------------------
  public:
    /* Constructors */
    TIGERBotUtility (//WirelessInterfaceESP8266*  radio,
						//I2CInterface*              i2c,
						);
    /* Destructor */
    ~TIGERBotUtility();

/* ── Controller and utility functions ──────────────────────────────────────── */

//bool create_is_initialized(const Ref<const MatrixXf>x, const Ref<const MatrixXf>ic);
bool create_is_initialized(const MatrixXf& x, const MatrixXf& ic, float pos_err,float rot_err);
MatrixXf create_si_barrier_certificate(const MatrixXf& dxi, const MatrixXf& x,float safety);
MatrixXf create_uni_barrier_certificate(const MatrixXf& dxu, const MatrixXf& x,float safety_radius, float proj_dist);
MatrixXf create_si_to_uni_mapping_si2uniDyn(const MatrixXf& dxi,const MatrixXf& states);
MatrixXf create_si_to_uni_mapping_uni2siStates(const MatrixXf& states);
MatrixXf create_si_to_uni_mapping2(const MatrixXf& dxi, const MatrixXf& states,float lin_vel_gain, float ang_vel_lim);
MatrixXf create_si_to_uni_mapping3(const MatrixXf& dxi, const MatrixXf& states);
MatrixXf create_uni_to_si_mapping_uni2siDyn(const MatrixXf& dxu, const MatrixXf& states);
MatrixXf create_uni_to_si_mapping_si2uniStates(const MatrixXf& uni_states, const MatrixXf& si_states);
MatrixXf create_automatic_parking_controller2(const MatrixXf& states, const MatrixXf& poses,float pos_err,float rot_err);
MatrixXf getPolygonDims(float radius, int n, int desired_output);

//MatrixXf eigenqpIneq(MatrixXf& Q, MatrixXf& c, MatrixXf& A, MatrixXf& b); /* TO DO: EigenQP-based alternative */

/* TO DO: parameter setter declarations — not yet implemented
    float setSafetyRadius(float );
	float setBarrierGain();
	float setPositionError();
	float setRotationError();
	float setProjectionDistance();
	float setVelocityMagnitudeLimit();
	float setLinearVelocityGain();
	float setAngularVelocityGain();
	*/
    //--------------------------------------------------------------
    // Public Member Variables
    //--------------------------------------------------------------
   
   //WirelessInterfaceESP8266* radio_;
	private:
	/* TO DO: static parameter storage — not yet implemented
	float static pos_err;
	float static rot_err;
	float static safety_radius;
	float static gamma;
	float static proj_dist;
	float static vel_mag_lim;
	float static lin_vel_gain;	//LinearVelocityGain
    float static ang_vel_lim;	// AngularVelocityLimit (default PI; use PI/2 for DistArea4)
	*/
	//--------------------------------------------------------------------------
	// Private Member Functions
    //--------------------------------------------------------------------------
	private:
	int factorial(int num);
	int nchoosek(int n, int k);
	float wrap(float p);	
	
    //--------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------
	
};
#endif