# TIGERSquare — Decentralized Processing

ESP8266 firmware and Arduino libraries enabling decentralized control algorithm processing for the TIGERSquare multi-robot testbed at the LSU iCORE Lab.

This repository contains the firmware contributions described in:

**T. Sarker, "Decentralized Processing for Distance-Based Formation Control," M.S. Thesis, Louisiana State University, August 2021.**
[[PDF]](docs/DECENTRALIZED%20PROCESSING%20FOR%20DISTANCE%20BASED%20FORMATION%20CONTROL_Tonmoy%20Sarker_2021.pdf)

---

## Repository Structure

```
firmware/
├── libraries/
│   ├── TIGERBot_Utility/           # Control utility library (barrier certs, kinematics, parking)
│   ├── TIGERBot_WirelessInterface/ # UDP + ESP-NOW communication
│   ├── TIGERBot_Main/              # Main board coordination
│   ├── TIGERBot_Motor/             # Stepper motor control
│   ├── TIGERBot_IMU/               # IMU interface
│   ├── TIGERBot_I2CInterface/      # I2C bus interface
│   ├── TIGERBot_Messages/          # Message type definitions
│   └── TIGERBot_WiFiConfig/        # Wi-Fi configuration
└── TIGERBot_firmware/
    └── firmware_main/
        └── firmware_main.ino       # Main robot firmware sketch
```

## Dependencies

Install these libraries separately into your Arduino `libraries/` folder before building:

- [Eigen](https://eigen.tuxfamily.org/) — matrix operations
- [eiquadprog](https://github.com/stack-of-tasks/eiquadprog) — QP solver for barrier certificates
- [SimpleEspNowConnection](https://github.com/mtroller/SimpleEspNowConnection) — ESP-NOW helper

## Hardware

- **Microcontroller**: ESP8266 NodeMCU (ESP-12E)
- **Motor driver**: ATmega 168P (stepper)
- **Board target**: NodeMCU 1.0 (ESP-12E Module) in Arduino IDE

---

*More documentation and credits will be added soon.*
