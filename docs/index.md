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
  padding: 3px 10px;
  border-radius: 4px;
  font-size: 13px;
  margin: 2px 4px 2px 0;
}
table {
  font-size: 14px;
}
</style>

# 🤖 ROSonicCam

**ROSonicCam** is a ROS 2-compatible embedded sensing node designed for autonomous drones and robots. It enables **multi-directional ultrasonic sensing**, **servo-based camera control**, and **adaptive Kalman filtering**, all orchestrated under a **real-time FreeRTOS** architecture.

> 🛠️ Built on the **Arduino framework** for rapid prototyping  
> 🚀 Upcoming: **ESP-IDF** & **RP2040** versions for enhanced performance and deeper hardware integration

---

## 🌟 Core Features

<div class="feature-badge">ros2</div>
<div class="feature-badge">esp32</div>
<div class="feature-badge">ultrasonic</div>
<div class="feature-badge">servo-control</div>
<div class="feature-badge">kalman-filter</div>
<div class="feature-badge">freertos</div>
<div class="feature-badge">micro-ros</div>
<div class="feature-badge">drones</div>
<div class="feature-badge">robotics</div>

- 🛁 Five-directional ultrasonic obstacle detection (HC-SR04)
- 🧠 Adaptive Kalman filter for reliable, noise-resistant measurements
- ⏱️ FreeRTOS-based multi-threaded architecture at 20 Hz
- 📵 Servo motor (PWM) for live camera orientation
- 🔌 micro-ROS communication over Serial or UDP
- 🩺 Live diagnostics via `/diagnostics` topic

---

## 🔧 Hardware Setup

| Component        | Details                             |
|------------------|-------------------------------------|
| **MCU**          | ESP32-WROOM                         |
| **Sensors**      | 5 × HC-SR04 ultrasonic modules      |
| **Actuator**     | PWM servo motor (GPIO 26)           |
| **Communication**| micro-ROS client → ROS 2 agent      |
| **Host**         | ROS 2 (Foxy/Humble) companion device|

---

## 🔗 Repository

🔍 [View source code on GitHub →](https://github.com/Adem-Aoun/ROSonicCam)

---

## 🚀 ROS 2 Interfaces

| Topic/Service                      | Type                                      | Purpose                          |
|-----------------------------------|-------------------------------------------|----------------------------------|
| `/ultrasonic_sensor/*/raw`        | `sensor_msgs/msg/Range`                   | Raw sensor distance              |
| `/ultrasonic_sensor/*/filtered`   | `sensor_msgs/msg/Range`                   | Filtered (Kalman) distance       |
| `/diagnostics`                    | `diagnostic_msgs/msg/DiagnosticStatus`    | Sensor status and hardware health|
| `/servo_cam_service`              | `servocam_interfaces/srv/Servocam`        | Set servo angle via ROS 2        |

---

## ⚙️ Quick Start

### 1. Flash Firmware to ESP32
```bash
pio run -t upload
```

### 2. Start micro-ROS Agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### 3. Monitor Sensor Data
```bash
ros2 topic echo /ultrasonic_sensor/forward/filtered
```

### 4. Control Servo (Camera Angle)
```bash
ros2 service call /servo_cam_service servocam_interfaces/srv/Servocam "{angle_deg: 90.0}"
```

---

## 🧱 Roadmap

### 🔜 ESP-IDF Port
- ⚡ Lower latency and tighter real-time guarantees
- 📲 OTA updates & deep power control
- 🛠️ Native interrupt/DMA use for sensor reads
- 💬 Cleaner ROS 2 abstraction

### 🦖 Raspberry Pi Pico (RP2040) Support
- Support for bare-metal + Pico-SDK builds
- micro-ROS client port to RP2040
- Servo control and ultrasonic read via PIO

---

## 🌿 Tags

`ros2` · `esp32` · `arduino` · `freertos` · `ultrasonic` · `kalman-filter` · `servo-control`  
`robotics` · `drones` · `micro-ros` · `embedded-systems` · `rp2040` · `esp-idf`

---

## 📄 License

Licensed under the [MIT License](https://opensource.org/licenses/MIT)  
© 2024 [Adem Aoun](https://github.com/Adem-Aoun)


## 📄 License

Licensed under the [MIT License](https://opensource.org/licenses/MIT)

