/**
 * @file    firmware_main.ino
 * @brief   TIGERBot main firmware for the ESP8266-based multi-robot platform.
 *
 * Authors:
 * Victor Fernandez-Kim (2018)
 * Tonmoy Sarker (2021)
 *
 * Implements four cooperative control architectures:
 *   1  Centralized         — host computes all velocities, distributes via UDP
 *   2  Semi-Centralized    — leader robot runs algorithm, broadcasts via ESP-NOW  (default)
 *   3  Semi-Decentralized  — each robot runs algorithm after host trigger
 *   4  Decentralized       — reserved for future use
 *
 * Usage:
 *   1. Set `controlMethod` and populate `IDs[]` to match the active robot fleet.
 *   2. Flash all robots; the leader is identified at runtime by `IDs[0]`.
 *
 * @platform  ESP8266 (NodeMCU / Wemos D1 Mini)
 *
 */

/* Selects the correct I2C interface variant at compile time */
#define ESP8266

/* ── Debug ────*/
#include <ArduinoTrace.h>

/* ── Standard Arduino ─── */
#include <EEPROM.h>
#include <Wire.h>

/* ── ESP8266 Wi-Fi ───── */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

/* ── Serialization ───── */
#include <ArduinoJson.h>

/* ── Over-the-Air updates ──── */
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>

/* ── Sensor drivers ──── */
#include <TIGERBot_LSM9DS1.h>

/* ── Linear algebra ──── */
#include "Eigen313.h"
using namespace Eigen;
using namespace std;

/* ── TIGERSquare libraries ── */
#include "TIGERBot_Messages.h"
#include "TIGERBot_Main_ESP8266.h"
#include "TIGERBot_Utility.h"
#include "I2CInterface.h"
#include "wirelessInterfaceESP8266.h"
#include "wifiConfig.h"

/* ═══════════════════════════════════════════════════════════════════════════
   PLATFORM OBJECTS
   ═══════════════════════════════════════════════════════════════════════════ */

WirelessInterfaceESP8266 wifi;
I2CInterface             i2c;
Adafruit_INA219          ina219;
LSM9DS1                  imu;
ControllerTarget         controller;
EstimatorBase            estimator;
TIGERBotUtility          utility;

TIGERBotMain mainboard(&wifi, &i2c, &ina219, &imu, &utility, &controller, &estimator);

/* ═══════════════════════════════════════════════════════════════════════════
   CONFIGURATION
   ═══════════════════════════════════════════════════════════════════════════ */

/** IDs of the robots participating in this experiment. */
int IDs[] = {2, 4, 7, 10};
int len    = sizeof(IDs) / sizeof(IDs[0]);

/**
 * Control architecture selection:
 *   1 — Centralized
 *   2 — Semi-Centralized   (default)
 *   3 — Semi-Decentralized
 *   4 — Decentralized
 */
int controlMethod = 2;

/* ═══════════════════════════════════════════════════════════════════════════
   ESP-NOW CALLBACKS  (Mode 2)
   ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Handles incoming ESP-NOW velocity packets.
 *
 * Packet layout: [v0, w0, v1, w1, ..., vN-1, wN-1]  (packed IEEE-754 floats)
 * Each robot identifies its own slot by matching its MAC ID against the IDs array.
 */
class WiFiConnector {
public:
    static void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t dataLen) {
        int   count = dataLen / 4;
        float velocities[count];
        memcpy(&velocities, incomingData, sizeof(velocities));

        Serial.print("\nReceived velocities: ");
        for (int i = 0; i < count; i++) {
            Serial.printf("%f ", velocities[i]);
        }

        float v = 0.0f, w = 0.0f;
        for (int i = 0; i < count; i++) {
            if (wifi.getMACID() == IDs[i]) {
                v = velocities[2 * i];
                w = velocities[2 * i + 1];
                mainboard.setVelocities(v, w);
            }
        }
        mainboard.setVelocities(v, w);
    }

    static void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
        Serial.println(sendStatus == 0 ? "Delivery success" : "Delivery fail");
    }
};

/* ═══════════════════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════════════════ */

void setup() {
    Serial.begin(115200);
    Serial.println("Mainboard initialization starting...");

    mainboard.initialize();

    ArduinoOTA.begin();
    Serial.println("Ready for OTA firmware updates");

    Serial.print("Firmware version: "); Serial.println(mainboard.getMainBoardFirmwareVersion());
    Serial.print("Hardware version: "); Serial.println(mainboard.getMainBoardHardwareVersion());

    mainboard.robIDs(IDs, len);
    wifi.numberOfRobots(len);
    wifi.getIDs(IDs, len);

    wifi.setESPNowSenderReceiver(WiFiConnector::OnDataRecv, WiFiConnector::OnDataSent);
}

/* ═══════════════════════════════════════════════════════════════════════════
   MAIN LOOP
   ═══════════════════════════════════════════════════════════════════════════ */

void loop() {
    yield();
    if (controlMethod == 1) { runCentralizedMode();       }
    if (controlMethod == 2) { runSemiCentralizedMode();   }
    if (controlMethod == 3) { runSemiDecentralizedMode(); }
    if (controlMethod == 4) { runDecentralizedMode();     }
    else                      yield();

    ArduinoOTA.handle();
}

/* ═══════════════════════════════════════════════════════════════════════════
   CONTROL MODE IMPLEMENTATIONS
   ═══════════════════════════════════════════════════════════════════════════ */

void runCentralizedMode() {
    yield();
    mainboard.updateWireless();
    yield();
    if (controlMethod == 3) {
        mainboard.runAlgorithm();
    }
    yield();
    mainboard.updateController();
    mainboard.updateMeasurements();
}

void runSemiCentralizedMode() {
    yield();
    if (wifi.getMACID() == IDs[0]) {
        mainboard.runAlgorithm();
    } else {
        yield();
    }
    mainboard.updateWireless();
    yield();
    mainboard.updateController();
    mainboard.updateMeasurements();
}

void runSemiDecentralizedMode() {
    yield();
    mainboard.updateWireless();
    mainboard.runAlgorithm();
    yield();
    mainboard.updateController();
    mainboard.updateMeasurements();
}

void runDecentralizedMode() {
    // TO DO: Requires localization Sensorboard.
}
