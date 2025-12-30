# 🏥 MediBot: Web-Teleoperated & Autonomous Hospital Logistics Robot

**A cost-effective, IoT-enabled mobile robot designed for autonomous medical supply delivery and remote teleoperation in hospital environments.**

[![Project Status](https://img.shields.io/badge/Status-Prototype-orange.svg)](#)
[![Tech](https://img.shields.io/badge/Platform-ESP32-blue.svg)](#)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20/%20C++-green.svg)](#)

MediBot addresses the "last 50 meters" delivery challenge in healthcare facilities[cite: 46]. [cite_start]By utilizing an ESP32-based architecture, the system provides a scalable and affordable alternative to expensive commercial Automated Guided Vehicles (AGVs), with a total component cost under $50[cite: 32, 163].


---

## 🚀 Key Features

* [cite_start]**🤖 Dual-Mode Navigation:** Combines IR-based line-following for structured paths with ultrasonic obstacle avoidance for safety in dynamic corridors[cite: 27, 33].
* [cite_start]**🌐 Web-Based Teleoperation:** Features an asynchronous WebSocket interface hosted directly on the ESP32, allowing low-latency (<100ms) remote manual control via smartphone or laptop[cite: 173, 217].
* [cite_start]**📦 Smart Delivery Lid:** An automated proximity-based lid system using a servo motor that opens only when an object is detected within 15cm[cite: 138, 268].
* [cite_start]**🛡️ Collision Mitigation:** Real-time distance monitoring triggers an automatic motor shutdown if an obstacle is detected within a 15cm safety buffer[cite: 137, 185].
* [cite_start]**📶 Fail-Safe Connectivity:** Functions as a standalone Wi-Fi Access Point ("LogisticsRobot"), ensuring the system remains operational even without local hospital network infrastructure[cite: 102, 172].

---

## 🛠️ Technical Specifications

### Hardware Architecture
| Component | Function |
| :--- | :--- |
| **ESP32 Dev Module** | [cite_start]Central processing & WebSocket hosting (240 MHz Dual-Core) [cite: 101, 104] |
| **L298N Motor Driver** | [cite_start]Controls 4-wheel differential drive movement [cite: 109] |
| **HC-SR04 Ultrasonic** | [cite_start]Dual-purpose: Obstacle avoidance and smart lid trigger [cite: 136] |
| **TCRT5000 IR Array** | [cite_start]High-reflectivity detection for 2-3cm path following [cite: 128, 131] |
| **SG90 Servo** | [cite_start]Actuates the secure supply compartment lid [cite: 145] |
| **7-12V Power System** | [cite_start]Stabilized power delivery for high-torque navigation (~3.1A peak) [cite: 276, 287] |


### Software Stack
* [cite_start]**Language:** C++ / Arduino Framework[cite: 170].
* [cite_start]**Protocols:** WebSockets for real-time, non-blocking manual overrides[cite: 199].
* [cite_start]**Logic:** Implements software debouncing and asynchronous event handling to manage concurrent sensor polling and motor control[cite: 197, 218].

---

## 📊 Performance Metrics

[cite_start]In controlled environment testing, MediBot achieved the following reliability ratings over 10 trials[cite: 247, 281]:

* [cite_start]**Line-Following Accuracy:** 82% overall (90% straight-line stability)[cite: 282].
* [cite_start]**Obstacle Avoidance:** 85% success rate navigating obstacles at 5–50cm[cite: 283].
* [cite_start]**Smart Lid Operation:** 95% reliability in proximity-based triggering[cite: 284].
* [cite_start]**Latency:** Web-interface response time maintained under 100ms[cite: 217].

---

## 🔧 Setup & Installation

1.  [cite_start]**Hardware Wiring:** Connect components to the ESP32 GPIOs as defined in the documentation (e.g., Motors to 27, 26, 33, 32; IR to 18, 19)[cite: 105, 209].
2.  **Flash Firmware:** Upload the provided C++ code using the Arduino IDE.
3.  [cite_start]**Connect:** Join the Wi-Fi network `LogisticsRobot` from any browser-enabled device[cite: 102].
4.  [cite_start]**Control:** Access `192.168.4.1` to toggle between **Autonomous Mode** and **Manual Teleoperation**[cite: 281].

---
