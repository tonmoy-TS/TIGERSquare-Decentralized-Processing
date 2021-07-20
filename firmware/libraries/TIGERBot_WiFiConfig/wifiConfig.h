/**
 * @file    wifiConfig.h
 * @brief   Wifi authentication data for TIGERBot main board ESP8266.
 *
 * Declares the wifi network SSID and password for the TIGERBot fleet, as well as the broadcast address
 * for UDP messaging. Also declares the SSID and password for each robot's access point (AP) mode.
 * 
 * Originally created by Daniel Pickem (2015).
 * Extended by Victor Fernandez-Kim (2018).
 * Modified by Tonmoy Sarker (2021).
 */

#ifndef _WIFI_CONFIG_
#define _WIFI_CONFIG_

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
namespace wifiConfig {
	extern const char* wifiSSID;
	extern const char* wifiPassword;
	extern const char* wifiBroadcast;
	
/* SSID and Password for setting up Wifi AP mode for each robot*/	
	extern const char* Robot1SSID;
	extern const char* Robot1Password;
	extern const char* Robot2SSID;
	extern const char* Robot2Password;
	extern const char* Robot3SSID;
	extern const char* Robot3Password;
	extern const char* Robot4SSID;
	extern const char* Robot4Password;
	extern const char* Robot5SSID;
	extern const char* Robot5Password;
	extern const char* Robot6SSID;
	extern const char* Robot6Password;
	extern const char* Robot7SSID;
	extern const char* Robot7Password;
	extern const char* Robot8SSID;
	extern const char* Robot8Password;
	extern const char* Robot9SSID;
	extern const char* Robot9Password;
	extern const char* Robot10SSID;
	extern const char* Robot10Password;

}
#endif
