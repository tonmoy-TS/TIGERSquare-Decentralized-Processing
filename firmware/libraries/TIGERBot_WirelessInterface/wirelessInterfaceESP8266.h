/**
 * @file    wirelessInterfaceESP8266.h
 * @brief   Interface for the ESP8266 wireless class.
 *
 * Declares WirelessInterfaceESP8266, providing UDP messaging, ESP-NOW
 * peer-to-peer communication, access-point management, and MAC-address-based
 * robot identification for the TIGERBot fleet.
 *
 * Originally created by Daniel Pickem (2015).
 * Extended by Victor Fernandez-Kim (2018).
 * ESP8266 port and formation algorithm C++ translation by Tonmoy Sarker (2021).
 */

#ifndef _WIRELESS_INTERFACE_ESP8266_h_
#define _WIRELESS_INTERFACE_ESP8266_h_

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
/* Include ESP8266 headers for Wifi and UDP */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <IPAddress.h>
#include <espnow.h>

/* WiFi authentication data */
#include "wifiConfig.h"

//#include "sendRcv.h"

/* Include message definitions */
//#include "TIGERBot_Messages/TIGERBot_Messages.h"


/* Include low-level ESP8266 functions */
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


/* ── C++ standard library and linear algebra ──*/
//#include <bits/stdc++.h>

#include <iostream>
//#include <algorithm> 		// for *min_element(arr, arr + n)
#include<ctime>				//srand()
#include<cstdlib>

#define _USE_MATH_DEFINES
#include <cmath>  			// for using M_PI, trigonometric functions (sin/cos/tan)

using namespace std;

//#include <BasicLinearAlgebra.h>
//using namespace BLA;

//#include <ArduinoSTL.h>
#include "Eigen313.h"
using namespace Eigen;

#define PI 3.1416

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define BUFFER_SIZE     256

class WirelessInterfaceESP8266 {
	
  //--------------------------------------------------------------------------
  // Public Member Functions
  //--------------------------------------------------------------------------
  public:
    /* Constructors */
    WirelessInterfaceESP8266();

    /* Destructor */
    ~WirelessInterfaceESP8266();

    /* Setup function */
    bool initialize(); /* Set ESP8266 in AP+STA mode, connect to router, begin UDP */
    bool switchAP(String SSID, String password);

    /* Wireless communication functions */
    void    sendMessage(String data);
    int32_t receiveMessage();
    
	  /* Wifi Connection Status functions */
    bool isConnected(); /* returns true when WiFi.status() == WL_CONNECTED */

    /* Set functions */
    void setPortIncoming(uint16_t port);
    void setPortOutgoing(uint16_t port);
    void setHostIP(String hostIP);
    bool setWifiChannel(uint8_t channel); /* channels 1–12 valid for 2.4 GHz */
    void setServerSSID(String ssid);
    void setServerPassword(String password);
    void reconnectToMainHost();

	  bool setWifiAPMode();
	  bool setWifiSTAMode();

    /* Get functions — inline accessors returning cached state */

    uint16_t  getPortIncoming() 	{ return portIncoming_; };
    uint16_t  getPortOutgoing() 	{ return portOutgoing_; };
	  uint16_t  getInterRobotPortIncoming() { return InterRobotPortIncoming_; };
	  uint16_t  getInterRobotPortOutgoing()  { return InterRobotPortOutgoing_; };

    IPAddress getHostIP()        { return hostIP_; };
    String    getHostIPString()  { return hostIPStr_; };
    IPAddress getLocalIP()       { return WiFi.localIP(); };
    int       getWiFiStatus()    { return WiFi.status(); };
    bool      getHostIPStatus()  { return receivedHostIP_; };


    String    getMACaddress();
    int       getMACID(); /* maps MAC address to robot index 1–10 */
    String    getMessage()      { return msg_; }
	  String 	  getMACaddressfromMACID();


	  int       getLeaderID()			{return leaderID_;}
	  IPAddress getLeaderAPIP() 		{ return leaderAPIP_; };
	  String    getLeaderAPIPString() { return leaderAPIPStr_; };	
	
	  IPAddress getLeaderLocalIP() 		{ return leaderLocalIP_; };
	  String    getLeaderLocalIPString()  { return leaderLocalIPStr_; };	

	  void scanForRobots();
	  void assignLeaderParameters();
	  bool connectToLeader(String SSID, String password);	
	
	  //void runSemiCentralizedMode(); /* TO DO: not yet implemented */
	  void getIDs(int IDarray[], int len);		
	  bool isConnectedToLeader();
	  int numberOfRobots(int num);
	  void setLeaderFollowerConnections(int robIDarray[],int robNum);

  /* ── ESP-NOW inter-robot communication ─────────────────────────────────────── */
	  uint8_t* getMAC_hexa(int mac_id);
	  void initESPNow();
	  void setESPNowSenderReceiver(esp_now_recv_cb_t recv, esp_now_send_cb_t send);
	  void sendAllVelocity(const MatrixXf& dq);
	  void sendData(float data);
		
  //--------------------------------------------------------------------------
  // Public Member Variables
  //--------------------------------------------------------------------------
  public:
    WiFiClient* wifiClient_;
    WiFiUDP udp_;
	  int myID;
	
	  /* Static IP configuration for each robot in AP mode (10 robots) */
    IPAddress AP_IP_Robot1, AP_gateway_Robot1, AP_subnet_Robot1;
    IPAddress AP_IP_Robot2, AP_gateway_Robot2, AP_subnet_Robot2;
    IPAddress AP_IP_Robot3, AP_gateway_Robot3, AP_subnet_Robot3;
    IPAddress AP_IP_Robot4, AP_gateway_Robot4, AP_subnet_Robot4;
    IPAddress AP_IP_Robot5, AP_gateway_Robot5, AP_subnet_Robot5;
    IPAddress AP_IP_Robot6, AP_gateway_Robot6, AP_subnet_Robot6;
    IPAddress AP_IP_Robot7, AP_gateway_Robot7, AP_subnet_Robot7;
    IPAddress AP_IP_Robot8, AP_gateway_Robot8, AP_subnet_Robot8;
    IPAddress AP_IP_Robot9, AP_gateway_Robot9, AP_subnet_Robot9;
    IPAddress AP_IP_Robot10,AP_gateway_Robot10,AP_subnet_Robot10;

  /* ── Fleet coordination state ─────────────────────────────────────────────── */
    int     networksFound;
    int     N;
    int     scannedIDs[10];
    String  scannedMACaddress[10];
    int     leaderID_;
    bool    enableWifiScan;
    int     *ids;
    int     numOfRobots;
    //RowVectorXi iDs; /* TO DO: Eigen-based ID vector (alternative to int*) */

    
    String    leaderMACAddress_ ;	
    String    leaderAPSSID_;
    String    leaderAPPassword_;
    
    IPAddress leaderAPIP_;
    String 		leaderAPIPStr_;
    IPAddress leaderLocalIP_;
    String 		leaderLocalIPStr_;
    
	
	
  //--------------------------------------------------------------------------
  // Private Member Variables
  //--------------------------------------------------------------------------
  private:
    IPAddress hostIP_; 
    String 		hostIPStr_; 	
    uint16_t 	portIncoming_; 	
    uint16_t 	portOutgoing_; 	
	  uint16_t	InterRobotPortIncoming_;
	  uint16_t	InterRobotPortOutgoing_;
    String 		serverSSID_; 	
    String 		serverPassword_;
    uint8_t 	wifiChannel_;	
    char 		  receiveBuffer_[BUFFER_SIZE];	
    char 		  sendBuffer_[BUFFER_SIZE];		
    bool 		  receivedHostIP_;
    String 		msg_;
	  //bool		runWifiAPMode_;
	  int			  MACID_;
	  String    foundSSID_;

  //--------------------------------------------------------------------------
  // Private Member Functions
  //--------------------------------------------------------------------------
  private:
    /* Functions handling UDP package translation */
    bool      sendUdpPacket(String msg);
    int32_t   receiveUdpPacket();

    void      processHostIP(String IP_msg);
    IPAddress parseIPString(String ip);
    void      processWirelessParameters(String hostIP,
                                        int incomingPort,
                                        int outgoingPort);
	

	
};
#endif
