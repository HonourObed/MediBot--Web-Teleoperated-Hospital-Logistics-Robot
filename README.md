# 🏥 MediBot: Web-Teleoperated & Autonomous Hospital Logistics Robot

**A cost-effective, IoT-enabled mobile robot designed for autonomous medical supply delivery and remote teleoperation in hospital environments.**

[![Project Status](https://img.shields.io/badge/Status-Prototype-orange.svg)](#)
[![Tech](https://img.shields.io/badge/Platform-ESP32-blue.svg)](#)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20/%20C++-green.svg)](#)

MediBot addresses the "last 50 meters" delivery challenge in healthcare facilities. By utilizing an ESP32-based architecture, the system provides a scalable and affordable alternative to expensive commercial Automated Guided Vehicles (AGVs).



---

## 🚀 Key Features

* **🤖 Dual-Mode Navigation:** Combines IR-based line-following for structured paths with ultrasonic obstacle avoidance for safety in dynamic corridors.
* **🌐 Web-Based Teleoperation:** Features an asynchronous WebSocket interface hosted directly on the ESP32, allowing low-latency remote manual control via smartphone or laptop.
* **📦 Smart Delivery Lid:** An automated proximity-based lid system using a servo motor that opens only when an object is detected within 15cm.
* **🛡️ Collision Mitigation:** Real-time distance monitoring triggers an automatic motor shutdown if an obstacle is detected within a 15cm safety buffer.
* **📶 Fail-Safe Connectivity:** Functions as a standalone Wi-Fi Access Point ("LogisticsRobot"), ensuring the system remains operational even without local hospital network infrastructure.

---

## 🛠️ Technical Specifications

### Hardware Architecture
| Component | Function |
| :--- | :--- |
| **ESP32 Dev Module** | Central processing & WebSocket hosting (240 MHz Dual-Core) |
| **L298N Motor Driver** | Controls 4-wheel differential drive movement |
| **HC-SR04 Ultrasonic** | Dual-purpose: Obstacle avoidance and smart lid trigger |
| **TCRT5000 IR Array** | High-reflectivity detection for 2-3cm path following |
| **SG90 Servo** | Actuates the secure supply compartment lid |
| **7-12V Power System** | Stabilized power delivery for high-torque navigation |



### Software Stack
* **Language:** C++ (Arduino Framework).
* **Communication:** WebSockets for real-time, non-blocking manual overrides.
* **Logic:** Implements software debouncing and asynchronous event handling to manage concurrent sensor polling and motor control.

---

## 📊 Performance Summary

In controlled environment testing, MediBot demonstrated high reliability across its core functions:

* **Navigation:** Achieved 90% accuracy for straight-line stability and 100% reliability in stopping commands.
* **Safety:** Successfully navigated obstacles at distances between 5–50cm with 85% success.
* **Interaction:** The smart lid reached 95% reliability in proximity-based triggering.
* **Latency:** Web-interface response time maintained under 100ms for seamless remote operation.

---

## 🔧 Setup & Installation

1.  **Hardware Wiring:** Connect components to the ESP32 GPIOs as defined in the system documentation.
2.  **Flash Firmware:** Upload the C++ code using the Arduino IDE or PlatformIO.
3.  **Connect:** Join the Wi-Fi network `LogisticsRobot` from any browser-enabled device.
4.  **Control:** Access `192.168.4.1` to toggle between **Autonomous Mode** and **Manual Teleoperation**.

---

## 👥 Contributors
* **Your Name/Team** - Department of Mechatronics Engineering.
