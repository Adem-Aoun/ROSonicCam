---
layout: default
title: ROSonicCam
description: ROS 2-compatible ultrasonic sensor and camera control node for drones and robots.
---

<style>
h1, h2, h3 {
  color: #2c3e50;
}
p {
  font-size: 16px;
  line-height: 1.6;
}
.feature-badge {
  display: inline-block;
  background-color: #007acc;
  color: white;
  padding: 3px 8px;
  border-radius: 5px;
  font-size: 13px;
  margin: 2px;
}
</style>

# ROSonicCam

**ROSonicCam** is a lightweight, ROS 2-compatible embedded sensor node designed for directional obstacle detection and camera actuation in drones and mobile robots. Built on the **ESP32 microcontroller**, it combines **ultrasonic sensing**, **servo-based camera control**, and **adaptive Kalman filtering**, all managed under a **FreeRTOS**-driven architecture.

> 🔧 The current release is implemented with the **Arduino framework** for fast prototyping and ease of development.  
> 🚀 A future release will be based on **ESP-IDF**, bringing improved efficiency, memory management, and low-level hardware integration.

---

## 🌟 Key Features

- 🔌 **micro-ROS** integration with ROS 2 (Humble, Foxy, etc.)
- 📡 **Five-directional** HC-SR04 ultrasonic sensing (front, back, left, right, downward)
- 🧠 **Adaptive Kalman filtering** for real-time noise suppression
- ⏱️ **Non-blocking FreeRTOS multitasking** (20 Hz)
- 🎥 **Servo-controlled camera positioning** (PWM, ROS 2 service)
- 🩺 **System diagnostics** published via `/diagnostics`
- 🤖 Designed for UAVs, rovers, and edge robotics

---

## 📷 Hardware Overview

| Component        | Description                         |
|------------------|-------------------------------------|
| **MCU**          | ESP32-WROOM                         |
| **Sensors**      | 5× HC-SR04 ultrasonic modules       |
| **Actuator**     | Servo motor (PWM, GPIO 26)          |
| **Transport**    | micro-ROS over Serial or UDP        |
| **Agent**        | ROS 2 agent running on host machine |

---

## 🔗 Repository

👉 [Browse the code on GitHub](https://github.com/Adem-Aoun/ROSonicCam)

---

## 🚀 ROS 2 Interfaces

| Interface                      | Type                                         | Description                        |
|-------------------------------|----------------------------------------------|------------------------------------|
| `/ultrasonic_sensor/*/raw`    | `sensor_msgs/msg/Range`                      | Unfiltered HC-SR04 readings        |
| `/ultrasonic_sensor/*/filtered` | `sensor_msgs/msg/Range`                    | Kalman-filtered distance estimates |
| `/diagnostics`                | `diagnostic_msgs/msg/DiagnosticStatus`       | Real-time health reporting         |
| `/servo_cam_service`          | `servocam_interfaces/srv/Servocam`           | Set servo angle (0°–180°)          |

---

## 📘 Quick Start

1. 🔧 **Flash the firmware (Arduino-based)**:
   ```bash
   pio run -t upload
    ````

2. 🔌 **Start the micro-ROS Agent** on your PC:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```

3. 📡 **View filtered distance data**:

   ```bash
   ros2 topic echo /ultrasonic_sensor/forward/filtered
   ```

4. 🎥 **Control camera via servo**:

   ```bash
   ros2 service call /servo_cam_service servocam_interfaces/srv/Servocam "{angle_deg: 90.0}"
   ```

---

## 🏷️ Tags / Keywords

`ros2` · `esp32` · `arduino` · `freertos` · `ultrasonic` · `kalman-filter` · `servo-control`
`robotics` · `drones` · `obstacle-avoidance` · `micro-ros` · `embedded-systems`

---

## 📅 Roadmap: ESP-IDF & RP2040 Support

We are actively developing an **ESP-IDF-based version** of ROSonicCam, which will offer:

* ⚡ Optimized memory usage and task scheduling
* 🛠️ Low-level access to ESP32 peripherals and interrupts
* 📉 Reduced latency and better servo response
* 🧠 Cleaner abstraction for multi-platform support (ESP-IDF, RP2040,..)

📌 **Star** this repo to follow updates and releases!

---

## 📄 License

Licensed under the [MIT License](https://opensource.org/licenses/MIT)

