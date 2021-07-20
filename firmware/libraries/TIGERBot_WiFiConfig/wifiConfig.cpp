/**
 * @file    wifiConfig.cpp
 * @brief   Wifi authentication data for TIGERBot main board ESP8266.
 *
 * Declares the wifi network SSID and password for the TIGERBot fleet, as well as the broadcast address
 * for UDP messaging. Also declares the SSID and password for each robot's access point (AP) mode.
 *
 * Originally created by Daniel Pickem (2015).
 * Extended by Victor Fernandez-Kim (2018).
 * Modified by Tonmoy Sarker (2021).
 */

#include "wifiConfig.h"

// Lab router settings (See "Router Info.doc")
extern const char* wifiConfig::wifiSSID      = " "; //Enter SSID of your router here
extern const char* wifiConfig::wifiPassword  = " "; //Enter password of your router here

// standard udp broadcast address.
extern const char* wifiConfig::wifiBroadcast = "192.168.**.***"; //Enter broadcast address of your router here

/* The following part is only if you want to turn on the Wifi-AP modes for robots*/
/* Assign Soft-AP for individual robots */

extern const char* wifiConfig::Robot1SSID      = "TIGERBOT-1";
extern const char* wifiConfig::Robot1Password  = "TIGERBOT-1";

extern const char* wifiConfig::Robot2SSID      = "TIGERBOT-2";
extern const char* wifiConfig::Robot2Password  = "TIGERBOT-2";

extern const char* wifiConfig::Robot3SSID      = "TIGERBOT-3";
extern const char* wifiConfig::Robot3Password  = "TIGERBOT-3";

extern const char* wifiConfig::Robot4SSID      = "TIGERBOT-4";
extern const char* wifiConfig::Robot4Password  = "TIGERBOT-4";

extern const char* wifiConfig::Robot5SSID      = "TIGERBOT-5";
extern const char* wifiConfig::Robot5Password  = "TIGERBOT-5";

extern const char* wifiConfig::Robot6SSID      = "TIGERBOT-6";
extern const char* wifiConfig::Robot6Password  = "TIGERBOT-6";

extern const char* wifiConfig::Robot7SSID      = "TIGERBOT-7";
extern const char* wifiConfig::Robot7Password  = "TIGERBOT-7";

extern const char* wifiConfig::Robot8SSID      = "TIGERBOT-8";
extern const char* wifiConfig::Robot8Password  = "TIGERBOT-8";

extern const char* wifiConfig::Robot9SSID      = "TIGERBOT-9";
extern const char* wifiConfig::Robot9Password  = "TIGERBOT-9";

extern const char* wifiConfig::Robot10SSID      = "TIGERBOT-10";
extern const char* wifiConfig::Robot10Password  = "TIGERBOT-10";
