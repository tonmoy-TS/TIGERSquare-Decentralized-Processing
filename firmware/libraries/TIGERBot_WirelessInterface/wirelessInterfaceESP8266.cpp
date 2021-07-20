/**
 * @file    wirelessInterfaceESP8266.cpp
 * @brief   Wireless interface implementation for the ESP8266 WiFi module.
 *
 * Provides UDP messaging, ESP-NOW peer-to-peer communication, access-point
 * management, and MAC-address-based robot identification for the TIGERBot fleet.
 *
 * Originally created by Daniel Pickem (2015).
 * Extended by Victor Fernandez-Kim (2018).
 * ESP8266 port and formation algorithm C++ translation by Tonmoy Sarker (2021).
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "wirelessInterfaceESP8266.h"
#include "ESP8266WiFi.h"
	
/* Constructors */
WirelessInterfaceESP8266::WirelessInterfaceESP8266() {
  /* Set default access point parameters */
  portOutgoing_ = 4999; /* host-bound UDP port */
  portIncoming_ = 4998; /* robot-bound UDP port */
  InterRobotPortIncoming_ = 2000;
  InterRobotPortOutgoing_ = 2001;
};

/* Destructor */
WirelessInterfaceESP8266::~WirelessInterfaceESP8266(){};

//------------------------------------------------------------------------------
// Public Member Functions
//------------------------------------------------------------------------------


bool WirelessInterfaceESP8266::initialize() {
  /* Reset Host IP availability flag */
  receivedHostIP_ = false;
  //runWifiAPMode_ = true; /* TO DO: uncomment to test AP-only mode */

  /* AP+STA mode enables concurrent ESP-NOW (peer) and UDP (host) communication */
  WiFi.mode(WIFI_AP_STA);
  Serial.println("NodeMCU WiFi mode: AP + STA");

  /* Connect to access point (Wi-Fi router) defined in wifiConfig.cpp */
  //WiFi.begin(wifiConfig::wifiSSID, wifiConfig::wifiPassword);

  //setWifiAPMode();
  setWifiSTAMode();

  initESPNow();
};

bool WirelessInterfaceESP8266::setWifiAPMode() {
 
/* Define static IP, gateway, and subnet for each robot (10 robots total) */
IPAddress AP_IP_Robot1(192, 168, 4, 10), AP_gateway_Robot1(192, 168, 0, 10), AP_subnet_Robot1(255, 255, 255, 10);
IPAddress AP_IP_Robot2(192, 168, 4, 20), AP_gateway_Robot2(192, 168, 0, 20), AP_subnet_Robot2(255, 255, 255, 20);
IPAddress AP_IP_Robot3(192, 168, 4, 30), AP_gateway_Robot3(192, 168, 0, 30), AP_subnet_Robot3(255, 255, 255, 30);
IPAddress AP_IP_Robot4(192, 168, 4, 40), AP_gateway_Robot4(192, 168, 0, 40), AP_subnet_Robot4(255, 255, 255, 40);
IPAddress AP_IP_Robot5(192, 168, 4, 50), AP_gateway_Robot5(192, 168, 0, 50), AP_subnet_Robot5(255, 255, 255, 50);
IPAddress AP_IP_Robot6(192, 168, 4, 60), AP_gateway_Robot6(192, 168, 0, 60), AP_subnet_Robot6(255, 255, 255, 60);
IPAddress AP_IP_Robot7(192, 168, 4, 70), AP_gateway_Robot7(192, 168, 0, 70), AP_subnet_Robot7(255, 255, 255, 70);
IPAddress AP_IP_Robot8(192, 168, 4, 80), AP_gateway_Robot8(192, 168, 0, 80), AP_subnet_Robot8(255, 255, 255, 80);
IPAddress AP_IP_Robot9(192, 168, 4, 90), AP_gateway_Robot9(192, 168, 0, 90), AP_subnet_Robot9(255, 255, 255, 90);
IPAddress AP_IP_Robot10(192,168, 4, 100),AP_gateway_Robot10(192,168, 0, 100),AP_subnet_Robot10(255, 255, 255,100);

  int MACID_ = WirelessInterfaceESP8266::getMACID();
	
  /* Set ESP8266 in Soft-Access Point(AP) mode */
	if(MACID_==1){ 
				WiFi.softAPConfig(AP_IP_Robot1,AP_gateway_Robot1,AP_subnet_Robot1);
				WiFi.softAP(wifiConfig::Robot1SSID, wifiConfig::Robot1Password);
				}
    if(MACID_==2){ 
				WiFi.softAPConfig(AP_IP_Robot2,AP_gateway_Robot2,AP_subnet_Robot2);
				WiFi.softAP(wifiConfig::Robot2SSID, wifiConfig::Robot2Password);
				}
    if(MACID_==3){ 
				WiFi.softAPConfig(AP_IP_Robot3,AP_gateway_Robot3,AP_subnet_Robot3);
				WiFi.softAP(wifiConfig::Robot3SSID, wifiConfig::Robot3Password);
				}
	if(MACID_==4){ 
				WiFi.softAPConfig(AP_IP_Robot4,AP_gateway_Robot4,AP_subnet_Robot4);
				WiFi.softAP(wifiConfig::Robot4SSID, wifiConfig::Robot4Password);
				}
	if(MACID_==5){ 
				WiFi.softAPConfig(AP_IP_Robot5,AP_gateway_Robot5,AP_subnet_Robot5);
				WiFi.softAP(wifiConfig::Robot5SSID, wifiConfig::Robot5Password);
				}
	if(MACID_==6){ 
				WiFi.softAPConfig(AP_IP_Robot6,AP_gateway_Robot6,AP_subnet_Robot6);
				WiFi.softAP(wifiConfig::Robot6SSID, wifiConfig::Robot6Password);
				}
	if(MACID_==7){ 
				WiFi.softAPConfig(AP_IP_Robot7,AP_gateway_Robot7,AP_subnet_Robot7);
				WiFi.softAP(wifiConfig::Robot7SSID, wifiConfig::Robot7Password);
				}
	if(MACID_==8){ 
				WiFi.softAPConfig(AP_IP_Robot8,AP_gateway_Robot8,AP_subnet_Robot8);
				WiFi.softAP(wifiConfig::Robot8SSID, wifiConfig::Robot8Password);
				}    
	if(MACID_==9){ 
				WiFi.softAPConfig(AP_IP_Robot9,AP_gateway_Robot9,AP_subnet_Robot9);
				WiFi.softAP(wifiConfig::Robot9SSID, wifiConfig::Robot9Password);
				}
	if(MACID_==10){
				WiFi.softAPConfig(AP_IP_Robot10,AP_gateway_Robot10,AP_subnet_Robot10);
				WiFi.softAP(wifiConfig::Robot10SSID,wifiConfig::Robot10Password);
				}						
 
	Serial.printf("Wifi AP mode initialized. SSID:TIGERBOT-%d.Password:TIGERBOT-%d.IP:",MACID_,MACID_);
	Serial.print(WiFi.softAPIP());

}


bool WirelessInterfaceESP8266::setWifiSTAMode() {
   
    IPAddress staticIPRobot1(192, 168, 4, 100); 
    IPAddress staticIPRobot2(192, 168, 4, 102);
    IPAddress staticIPRobot3(192, 168, 4, 103);
    IPAddress staticIPRobot4(192, 168, 4, 104);
    IPAddress staticIPRobot5(192, 168, 4, 105);
    IPAddress staticIPRobot6(192, 168, 4, 106);
    IPAddress staticIPRobot7(192, 168, 4, 107);
    IPAddress staticIPRobot8(192, 168, 4, 108);
    IPAddress staticIPRobot9(192, 168, 4, 109);
    IPAddress staticIPRobot10(192, 168,4, 110);
    IPAddress gateway(192, 168, 0, 30);
    IPAddress subnet(255, 255, 255, 2); 
  
    /* Retrieve the MAC Address*/
	int MACID_ = WirelessInterfaceESP8266::getMACID();
	
    /* Connect to Access point (Wifi Router Network) defined in wifiConfig.cpp */
    WiFi.begin(wifiConfig::wifiSSID, wifiConfig::wifiPassword);

    /* Set ESP8266 Wifi configuration for station mode */
	if(MACID_==1){WiFi.config(staticIPRobot1,gateway,subnet);}
  if(MACID_==2){WiFi.config(staticIPRobot2,gateway,subnet);}
  if(MACID_==3){WiFi.config(staticIPRobot3,gateway,subnet);}
	if(MACID_==4){WiFi.config(staticIPRobot4,gateway,subnet);}
	if(MACID_==5){WiFi.config(staticIPRobot5,gateway,subnet);}
	if(MACID_==6){WiFi.config(staticIPRobot6,gateway,subnet);}
	if(MACID_==7){WiFi.config(staticIPRobot7,gateway,subnet);}
	if(MACID_==8){WiFi.config(staticIPRobot8,gateway,subnet);}    
	if(MACID_==9){WiFi.config(staticIPRobot9,gateway,subnet);}
	if(MACID_==10){WiFi.config(staticIPRobot10,gateway,subnet);}
	

  Serial.print("\nWifi Station mode initialized. Connecting to: ");Serial.print(wifiConfig::wifiSSID);
  Serial.print(". Password: "); Serial.println(wifiConfig::wifiPassword);
  /* If WiFi is not connected, try multiple times */
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    tries++;
	Serial.print(".");
  }
  
    /* Set autoconnect and reconnect flag */
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  if(WiFi.status() == WL_CONNECTED) {
    /* Start listening for UDP packages */
    if(udp_.begin(portIncoming_)) {
      Serial.print("\nConnected. Network: "); Serial.print(wifiConfig::wifiSSID);
      Serial.print(". Local IP: "); Serial.println(WiFi.localIP());
      Serial.print("Listening on UDP port: "); Serial.println(portIncoming_);
	  Serial.print("WiFi Channel: "); Serial.println(WiFi.channel());
    }

    return true;
  }
}

bool WirelessInterfaceESP8266::switchAP(String SSID, String password) {
  /* Disconnect from current AP if any */
  Serial.println("\nDisconnecting from previous Access Point...");
  WiFi.disconnect();
  
  Serial.print("Connecting to WiFi SSID: "); Serial.println(SSID);
  Serial.print("Connecting using PW: "); Serial.println(password);

  /* Cast strings to char arrays */
  char SSIDChar[sizeof(char) * (SSID.length() + 1)];
  SSID.toCharArray(SSIDChar, sizeof(char) * (SSID.length() + 1));
  char pwChar[sizeof(char) * (password.length() + 1)];
  password.toCharArray(pwChar, sizeof(char) * (password.length() + 1));

  /* Reset ESP8266 in Station Mode */
  WiFi.mode(WIFI_STA);

  /* Connect to new access point with the channel specified */
  WiFi.begin(SSIDChar, pwChar, wifiChannel_);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    tries++;
  }

  /* Set autoconnect and reconnect flag */
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  /* Restart UDP service */
  if(WiFi.status() == WL_CONNECTED) {
    /* Start listening for UDP packages */
    if(udp_.begin(portIncoming_)) {
      Serial.print("Connected AP: "); Serial.println(SSID);		// Gets the IP address of the connected router
      Serial.print("Listening on UDP port: "); Serial.println(portIncoming_);
      Serial.printf("BSSID: %s\n", WiFi.BSSIDstr().c_str());	// Gets the MAC address of the connected router
    }

    return true;
  }
}

/* Wireless communication functions */
void WirelessInterfaceESP8266::sendMessage(String data) {
  /* Send message to host via UDP */
  sendUdpPacket(data);
}

int32_t WirelessInterfaceESP8266::receiveMessage(){
  return receiveUdpPacket();
};

/* -----------------------------*/
/*        Set functions         */
/* -----------------------------*/
void WirelessInterfaceESP8266::setPortIncoming(uint16_t port) {
  /* Assign new port */
  portIncoming_ = port;

  /* Stop listening to current port */
  udp_.stop();
  delay(250);

  if(udp_.begin(portIncoming_)) {
    Serial.print("Listening on UDP port: ");
    Serial.println(portIncoming_);
  } else {
    Serial.print("Failed to open UDP connection on port: ");
    Serial.println(portIncoming_);
  }
}

void WirelessInterfaceESP8266::setPortOutgoing(uint16_t port) {
  portOutgoing_ = port;

  Serial.print("Sending to UDP port: ");
  Serial.println(portOutgoing_);
}


void WirelessInterfaceESP8266::setHostIP(String hostIP) {
  /* Parse host IP */
  hostIPStr_ = hostIP;			
  receivedHostIP_ = true; /* set after receiving valid host IP */

  Serial.print("\nSending to host IP: ");
  Serial.println(hostIPStr_);
}

bool WirelessInterfaceESP8266::setWifiChannel(uint8_t channel) {
  if(channel > 0 && channel < 13) { /* valid 2.4 GHz Wi-Fi channels */
    wifiChannel_ = channel;
    wifi_set_channel(channel);
    Serial.print("Switched to channel "); Serial.println(channel);
    return true;
  } else {
    return false;
  }
}
/* function to set the new SSID */
void WirelessInterfaceESP8266::setServerSSID(String ssid) {
  serverSSID_ = ssid;
}
/* function to set the new password */
void WirelessInterfaceESP8266::setServerPassword(String password) {
  serverPassword_ = password;
}

/* disconnects from the gateway AP, and connects to the main server */
void WirelessInterfaceESP8266::reconnectToMainHost(){
  switchAP(serverSSID_,serverPassword_);
}


/* -----------------------------*/
/*     Status functions         */
/* -----------------------------*/
bool WirelessInterfaceESP8266::isConnected() {
  if( WiFi.status() == WL_CONNECTED ) {
    return true;
  } else {
    return false;
  }
}

/* -----------------------------*/
/*        Get functions         */
/* -----------------------------*/
String WirelessInterfaceESP8266::getMACaddress(){
  /* Access the MAC address of the robot */
  byte mac[6];

  /* e.g. "18:fe:34:d4:d8:fe" */
  WiFi.macAddress(mac); //NodeMCU MAC address obtaining function

  char macChar[50] = {0};
  sprintf(macChar,"%02x:%02x:%02x:%02x:%02x:%02x",
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(macChar);
}

/* Function to provide the NodeMCU ID based on the MAC table */
int WirelessInterfaceESP8266::getMACID(){
	String myMAC = WirelessInterfaceESP8266::getMACaddress();
	if (myMAC == "68:c6:3a:9f:b3:1c"){return 1;}
	if (myMAC == "60:01:94:51:b6:c5"){return 2;}
	if (myMAC == "ec:fa:bc:08:0a:ef"){return 3;}
	if (myMAC == "68:c6:3a:9f:b3:d7"){return 4;}
	if (myMAC == "b4:e6:2d:28:ae:66"){return 5;}
	if (myMAC == "b4:e6:2d:28:ae:08"){return 6;}
	if (myMAC == "b4:e6:2d:28:ad:6e"){return 7;}
	if (myMAC == "b4:e6:2d:28:b1:aa"){return 8;}
	if (myMAC == "b4:e6:2d:28:b5:39"){return 9;}
	if (myMAC == "b4:e6:2d:28:ae:74"){return 10;} 
	else
		return false;
}

/* Returns the MAC address string corresponding to this robot's own MAC ID. */
String WirelessInterfaceESP8266::getMACaddressfromMACID() {
	
	int MACID_ = WirelessInterfaceESP8266::getMACID();

	if(MACID_ == 1) {return "68:c6:3a:9f:b3:1c";}
	if(MACID_ == 2) {return "60:01:94:51:b6:c5";}
	if(MACID_ == 3) {return "ec:fa:bc:08:0a:ef";}
	if(MACID_ == 4) {return "68:c6:3a:9f:b3:d7";}
	if(MACID_ == 5) {return "b4:e6:2d:28:ae:66";}
	if(MACID_ == 6) {return "b4:e6:2d:28:ae:08";}
	if(MACID_ == 7) {return "b4:e6:2d:28:ad:6e";}
	if(MACID_ == 8) {return "b4:e6:2d:28:b1:aa";}
	if(MACID_ == 9) {return "b4:e6:2d:28:b5:39";}
	if(MACID_ == 10){return "b4:e6:2d:28:ae:74";} 
}

uint8_t *WirelessInterfaceESP8266::getMAC_hexa(int mac_id) {
	/* Returns the 6-byte MAC address array for the given robot ID */
	static uint8_t mac_robot1[] = {0x68, 0xC6, 0x3A, 0x9F, 0xB3, 0x1C}; // ROB 1
	static uint8_t mac_robot2[] = {0x60, 0x01, 0x94, 0x51, 0xB6, 0xC5}; // ROB 2
	static uint8_t mac_robot3[] = {0xEC, 0xFA, 0xBC, 0x08, 0x0A, 0xEF}; // ROB 3
	static uint8_t mac_robot4[] = {0x68, 0xC6, 0x3A, 0x9F, 0xB3, 0xD7}; // ROB 4
	static uint8_t mac_robot5[] = {0xB4, 0xE6, 0x2D, 0x28, 0xAE, 0x66}; // ROB 5
	static uint8_t mac_robot6[] = {0xB4, 0xE6, 0x2D, 0x28, 0xAE, 0x08}; // ROB 6
	static uint8_t mac_robot7[] = {0xB4, 0xE6, 0x2D, 0x28, 0xAD, 0x6E}; // ROB 7
	static uint8_t mac_robot8[] = {0xB4, 0xE6, 0x2D, 0x28, 0xB1, 0xAA}; // ROB 8
	static uint8_t mac_robot9[] = {0xB4, 0xE6, 0x2D, 0x28, 0xB5, 0x39}; // ROB 9
	static uint8_t mac_robot10[]= {0xB4, 0xE6, 0x2D, 0x28, 0xAE, 0x74}; // ROB 10
	
	if(mac_id == 1) {return mac_robot1;}
	if(mac_id == 2) {return mac_robot2;}
	if(mac_id == 3) {return mac_robot3;}
	if(mac_id == 4) {return mac_robot4;}
	if(mac_id == 5) {return mac_robot5;}
	if(mac_id == 6) {return mac_robot6;}
	if(mac_id == 7) {return mac_robot7;}
	if(mac_id == 8) {return mac_robot8;}
	if(mac_id == 9) {return mac_robot9;}
	if(mac_id == 10){return mac_robot10;} 
}

//--------------------------------------------------------------------------//
// 						Private Member Functions							//
//--------------------------------------------------------------------------//

bool WirelessInterfaceESP8266::sendUdpPacket(String msg) {
  /* Allow background tasks of WiFi stack to execute */
  yield();

  /* Reset sendBuffer */
  memset(sendBuffer_, 0, sizeof(uint8_t) * BUFFER_SIZE);

  /* Convert message to char array */
  msg.toCharArray(sendBuffer_, msg.length() + 1);

  /* Send UDP packet and verify sending */
  if (receivedHostIP_) {

    udp_.beginPacket(parseIPString(hostIPStr_), portOutgoing_);
  } 
  else {

	//Serial.println(msg);
    udp_.beginPacket(wifiConfig::wifiBroadcast, portOutgoing_);
    //udp_.beginPacket(wifiConfig::wifiBroadcast, InterRobotPortOutgoing_);
    /* "192.168.1.255" is the standard UDP broadcast address for the local subnet */
  }

  uint8_t noBytesSent = udp_.write(sendBuffer_, msg.length() + 1);

  if(udp_.endPacket()) {
    return true;
  } else {
    return false;
  }
}

int32_t WirelessInterfaceESP8266::receiveUdpPacket() {
  /* Allow background tasks of WiFi stack to execute */
  yield();

  /* Parse UDP package */
  int noBytes = udp_.parsePacket();

  if ( noBytes ) {
    /* Read UDP package payload into char buffer */
    udp_.read(receiveBuffer_, noBytes); // read the packet into the buffer

    /* Create a string message from buffer */
    String msg;

    /* The received message is expected to contain a CSV string */
    for (int i = 0; i <= noBytes; i++){
      msg += String(receiveBuffer_[i]);
    }

    /* Store message in class variable */
    msg_ = msg;
	Serial.print("
 Received udp message: ");
	//Serial.print(msg_);
    return noBytes;
  }

  return -1;
}

void WirelessInterfaceESP8266::processHostIP(String hostIP) {
  /* Assign IP to class variables */
  hostIP_     = parseIPString(hostIP);		
  hostIPStr_  = String(hostIP);				

  receivedHostIP_ = true;
  Serial.print("Host IP is assigned to "); 
  Serial.println(hostIPStr_);
}

IPAddress WirelessInterfaceESP8266::parseIPString(String IP) {
  /* Parse serverIP, e.g. "192.168.1.22" */
  uint8_t ip[4];
  uint8_t del[5];

  del[0] = -1;
  del[1] = IP.indexOf('.');
  del[2] = IP.indexOf('.', del[1] + 1);
  del[3] = IP.indexOf('.', del[2] + 1);
  del[4] = IP.length();

  ip[0] = IP.substring(0, del[1]).toInt();
  //Serial.println(ip[0]);

  for (int i = 1; i < 4; i++) {
    ip[i] = IP.substring(del[i]+1, del[i+1]).toInt();
    //Serial.println(ip[i]);
  }

  return IPAddress(ip);
}


/* ═══ Fleet coordination extensions ═══════════════════════════════════════════ */


void WirelessInterfaceESP8266::getIDs(int IDarray[], int len) {
		ids = new int[len];
	for(int i=0;i<len;i++){
		ids[i]=IDarray[i];
	}	
	//delete ids; /* TO DO: deallocate before reassigning to avoid leak */	
}

int WirelessInterfaceESP8266::numberOfRobots(int num){

	numOfRobots= num;
	//Serial.printf("\nNumber of robots : %d",numOfRobots);
	
	return numOfRobots;
}

void WirelessInterfaceESP8266::setLeaderFollowerConnections(int robIDarray[],int robNum){
	
	int followerNum = robNum -1 ;
	
	/* For follower robots */
	if(getMACID()!=robIDarray[0]){		
		WiFi.softAPdisconnect(true); //Disable soft-AP mode
		assignLeaderParameters();
		//setEspNowCommunication(); /* TO DO: ESP-NOW peer setup not yet called here */
		
		if(WiFi.SSID()!=leaderAPSSID_){		
			connectToLeader(leaderAPSSID_,leaderAPPassword_);
			Serial.printf("\nConnected to Leader : %s",leaderAPSSID_.c_str());	
			setHostIP(leaderAPIPStr_);
		}		
	}
	
	/* For leader robot */	
	// Continues the loop until it finds all the followers connected
	else{
		int tries = 0;
		while (followerNum != WiFi.softAPgetStationNum()) { 			
			tries++;
			Serial.print(" > ");
			delay(3000);
		}
		
		// delay(1000);
		// Serial.println("All connected");
		// if(followerNum == WiFi.softAPgetStationNum()){
		//	delay(5000);
		//	Serial.printf("\n%d agents connected to leader.",WiFi.softAPgetStationNum());
		// }
		yield();
	}
}
 
/* Scans for active TIGERBot robots in range using asynchronous WiFi scan.
 * Populates scannedIDs[] and scannedMACaddress[] with found robots, including self. */
void WirelessInterfaceESP8266::scanForRobots() {
	
	/* Remember to turn off all other robots outside the testbed to make sure 
	scanning doesn't pick up undesired robots */
	int networksFound = WiFi.scanNetworks();	
	Serial.println("\nSearching for Active Robots...");

	for (int i=0,j=0; i < networksFound; i++){    
		//Serial.printf("%d: Network-Name:%s \n",i + 1, WiFi.SSID(i).c_str()); 
 
		N=j;
		String foundSSID_ =  WiFi.SSID(i);
	
		if (foundSSID_ == "TIGERBOT-1"){scannedIDs[j] = 1; scannedMACaddress[j]="68:c6:3a:9f:b3:1c"; j++;}
		if (foundSSID_ == "TIGERBOT-2"){scannedIDs[j] = 2; scannedMACaddress[j]="60:01:94:51:b6:c5"; j++;}
		if (foundSSID_ == "TIGERBOT-3"){scannedIDs[j] = 3; scannedMACaddress[j]="ec:fa:bc:08:0a:ef"; j++;}
		if (foundSSID_ == "TIGERBOT-4"){scannedIDs[j] = 4; scannedMACaddress[j]="68:c6:3a:9f:b3:d7"; j++;}
		if (foundSSID_ == "TIGERBOT-5"){scannedIDs[j] = 5; scannedMACaddress[j]="b4:e6:2d:28:ae:66"; j++;}
		if (foundSSID_ == "TIGERBOT-6"){scannedIDs[j] = 6; scannedMACaddress[j]="b4:e6:2d:28:ae:08"; j++;}
		if (foundSSID_ == "TIGERBOT-7"){scannedIDs[j] = 7; scannedMACaddress[j]="b4:e6:2d:28:ad:6e"; j++;}
		if (foundSSID_ == "TIGERBOT-8"){scannedIDs[j] = 8; scannedMACaddress[j]="b4:e6:2d:28:b1:aa"; j++;}
		if (foundSSID_ == "TIGERBOT-9"){scannedIDs[j] = 9; scannedMACaddress[j]="b4:e6:2d:28:b5:39"; j++;}
		if (foundSSID_ == "TIGERBOT-10"){scannedIDs[j]=10; scannedMACaddress[j]="b4:e6:2d:28:ae:74"; j++;}
		//else Serial.printf("**j:%d  N:%d ",j,N); 
		N=j;       
	}
    scannedIDs[N]= WirelessInterfaceESP8266::getMACID(); //include the robot itself
    scannedMACaddress[N] = WirelessInterfaceESP8266::getMACaddress();
    N=N+1;
    
    Serial.print("\nNumber of Active robots : "); Serial.print(N);
    Serial.print("\nActive Robots IDs: ");
    for(int k=0;k<N;k++){
      /* sort IDs in ascending order */
      for(int l=k+1;l<N;l++){
		if(scannedIDs[k]>scannedIDs[l]){
          int tempNumber = scannedIDs[k];
          String tempStr = scannedMACaddress[k];
          scannedIDs[k] = scannedIDs[l];
          scannedMACaddress[k]=scannedMACaddress[l];
          scannedIDs[l]= tempNumber;
          scannedMACaddress[l]= tempStr;
        }
      }
      Serial.printf(" %d ",scannedIDs[k]);
      //Serial.printf(" %s ",scannedMACaddress[k].c_str());
    }     
}

void WirelessInterfaceESP8266::assignLeaderParameters() {
	
	enableWifiScan = false; /* TO DO: move to initialization */
	/* If IDs retrieved through wifi scanning */
	if (enableWifiScan==true){	
		leaderID_ = scannedIDs[0];
		String leaderMACAddress_ = scannedMACaddress[0];
	}
	/* If IDs passed through main sketch */
	else {
		leaderID_ = ids[0];
	}

	//Serial.printf("\nThe leader is assigned to Robot ID : %d",leaderID_);

	/* get the SSID & Password for the leader */
	if(leaderID_ == 1){ leaderAPSSID_= wifiConfig::Robot1SSID; leaderAPPassword_= wifiConfig::Robot1Password;
						leaderAPIPStr_="192.168.4.10";
						}
	if(leaderID_ == 2){ leaderAPSSID_= wifiConfig::Robot2SSID; leaderAPPassword_= wifiConfig::Robot2Password;
						leaderAPIPStr_="192.168.4.20";
						}			
	if(leaderID_ == 3){ leaderAPSSID_= wifiConfig::Robot3SSID; leaderAPPassword_= wifiConfig::Robot3Password;
						leaderAPIPStr_="192.168.4.30";
						}
	if(leaderID_ == 4){ leaderAPSSID_= wifiConfig::Robot4SSID; leaderAPPassword_= wifiConfig::Robot4Password;
						leaderAPIPStr_="192.168.4.40";
						}
	if(leaderID_ == 5){ leaderAPSSID_= wifiConfig::Robot5SSID; leaderAPPassword_= wifiConfig::Robot5Password;
						leaderAPIPStr_="192.168.4.50";
						}
	if(leaderID_ == 6){ leaderAPSSID_= wifiConfig::Robot6SSID; leaderAPPassword_= wifiConfig::Robot6Password;
						leaderAPIPStr_="192.168.4.60";
						}
	if(leaderID_ == 7){ leaderAPSSID_= wifiConfig::Robot7SSID; leaderAPPassword_= wifiConfig::Robot7Password;
						leaderAPIPStr_="192.168.4.70";
						}
	if(leaderID_ == 8){ leaderAPSSID_= wifiConfig::Robot8SSID; leaderAPPassword_= wifiConfig::Robot8Password;
						leaderAPIPStr_="192.168.4.80";
						}
	if(leaderID_ == 9){ leaderAPSSID_= wifiConfig::Robot9SSID; leaderAPPassword_= wifiConfig::Robot9Password;
						leaderAPIPStr_="192.168.4.90";
						}
	if(leaderID_ == 10){leaderAPSSID_= wifiConfig::Robot10SSID;leaderAPPassword_= wifiConfig::Robot10Password;
						leaderAPIPStr_="192.168.4.100";
						}
	else 
		yield();
	
}

bool WirelessInterfaceESP8266::connectToLeader(String SSID, String password) {

  /* Connects a follower to the leader's soft-AP. Functionally equivalent to
   * switchAP() but listens on InterRobotPortIncoming_ instead of portIncoming_. */

  /* Disconnect from current AP if any */
  Serial.println("\nDisconnecting from previous Access Point...");
  WiFi.disconnect();
  
  Serial.print("Connecting to Leader: "); Serial.println(SSID);
  Serial.print(" using PW: "); Serial.println(password);

  /* Cast strings to char arrays */
  char SSIDChar[sizeof(char) * (SSID.length() + 1)];
  SSID.toCharArray(SSIDChar, sizeof(char) * (SSID.length() + 1));
  char pwChar[sizeof(char) * (password.length() + 1)];
  password.toCharArray(pwChar, sizeof(char) * (password.length() + 1));

  /* Reset ESP8266 in Station Mode */
  WiFi.mode(WIFI_STA); 

  /* Connect to new access point with the channel specified */
  WiFi.begin(SSIDChar, pwChar, wifiChannel_);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    tries++;
  }

  /* Set autoconnect and reconnect flag */
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  /* Restart UDP service */
  if(WiFi.status() == WL_CONNECTED) {
    /* Start listening for UDP packages */
    if(udp_.begin(InterRobotPortIncoming_)) {
      Serial.print("Connected AP: "); Serial.println(SSID);
	  Serial.printf("BSSID (MAC ID): %s\n", WiFi.BSSIDstr().c_str());	// Gets the MAC address of the connected router	  
      Serial.print("Listening to UDP port: ");
      Serial.println(portIncoming_);
    }
    return true;
  }
}


void WirelessInterfaceESP8266::initESPNow() {
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }	
}

void WirelessInterfaceESP8266::setESPNowSenderReceiver(esp_now_recv_cb_t recv, esp_now_send_cb_t send) {
  /* Leader robot: register as controller and add all follower robots as peers */
  if(getMACID() == ids[0]) {
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(send);

	for(int i=1;i<numOfRobots;i++){
		uint8_t* mac = getMAC_hexa(ids[i]);
		esp_now_add_peer(mac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
	}
  }

  /* Follower robots: register as slave receiver */
  else {
    WiFi.disconnect();
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(recv);
  }
}

void WirelessInterfaceESP8266::sendAllVelocity(const MatrixXf& dq) {
	yield();
	int n = dq.cols();
	float vel[2*n];

	Eigen::Map<MatrixXf>(&vel[0], 2, n) = dq; /* map Eigen matrix to C array (column-major) */
	delay(2);
	// Send message via ESP-NOW
	esp_now_send(0, (uint8_t *) &vel, sizeof(vel));
}

void WirelessInterfaceESP8266::sendData(float data) {
	Serial.printf("sending data:%f ", data);
	// Send message via ESP-NOW
	esp_now_send(0, (uint8_t *) &data, sizeof(data));
}

/* TO DO: network_setup() — placeholder for configuring WIFI_PS_NONE and
 * registering peers via esp_now_peer_info_t. Not yet implemented.

 void WirelessInterfaceESP8266::network_setup(){
  esp_wifi_set_ps(WIFI_PS_NONE)
  esp_now_peer_info_t peer_info;
  peer_info.channel = WIFI_CHANNEL;
  memcpy(peer_info.peer_addr, broadcast_mac, 6);
  peer_info.ifidx = ESP_IF_WIFI_STA;
  peer_info.encrypt = false;
  esp_err_t status = esp_now_add_peer(&peer_info);

} */
