/*
 -------------------------------------------------------------------------------
 I2C Interface for capable of running as both master (on the TIGERBot main
 board) and slave (on the TIGERBot motor board)
 
 METHODS:

 NOTES: On the main board this class can be used in both slave and master mode
        as well.

        - I2C master (for regular operation)
        - I2C slave (for debugging)   
 
 EXAMPLES:
 
 Initially created by Daniel Pickem 6/15/15.
 Modified by Victor Fernandez-Kim 6/13/18.
 -------------------------------------------------------------------------------
 */
#ifndef _I2C_INTERFACE_h_
#define _I2C_INTERFACE_h_

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>

/* Include I2C message definitions */
#include "include/I2CMessage.h"

class I2CInterface{
	public:
    // Constructors
    I2CInterface();

    // Destructor
    ~I2CInterface();

//--------------------------------------------------------------------------
// Public Member Functions
//--------------------------------------------------------------------------
#ifdef ESP8266
/*01*/	void initialize(uint8_t sdaPin, uint8_t sclPin);
#else
/*02*/	void initialize(uint8_t address = 0);
#endif

    /* I2C operation as master in default mode or slave mode for debugging */
/*03*/	void sendMessage(uint8_t msgType, float d1, float d2);
/*04*/	void sendMessage(uint8_t msgType, float d1, float d2, float d3);
/*05*/	void sendMessage(uint8_t msgType, float *data, 
						 uint8_t len, uint8_t address = 2);
/*06*/	void sendMessage(uint8_t msg, uint8_t address = 2);
/*07*/	bool receiveMessage(I2CMessage* msgOut, uint8_t address = 2);

/*08*/	bool isMaster() { return isMaster_;}

//--------------------------------------------------------------------------
// Private Member Variables
//--------------------------------------------------------------------------
	private:
	bool isMaster_;
};
#endif
