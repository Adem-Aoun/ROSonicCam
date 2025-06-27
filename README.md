# SkySonar

[![Build](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/Adem-Aoun/SkySonar/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**ESP32-based multi-directional ultrasonic sensor node for micro‑ROS**

SkySonar is a research-grade embedded system delivering robust range sensing for drones and robotics. It integrates five HC‑SR04 sensors, adaptive Kalman filtering, real-time FreeRTOS scheduling, and micro-ROS connectivity—encapsulated in a concise PlatformIO/Arduino project. This README doubles as reference documentation suitable for academic inclusion.

---

## 📋 Table of Contents

1. [Introduction](#introduction)
2. [System Architecture](#system-architecture)
3. [Electronics Design](#electronics-design)
4. [Software Architecture](#software-architecture)

   * [Includes & Dependencies](#includes--dependencies)
   * [FreeRTOS Scheduling](#freertos-scheduling)
   * [Inter-task Synchronization](#inter-task-synchronization)
   * [micro-ROS Integration](#micro-ros-integration)
5. [Kalman Filter Algorithm](#kalman-filter-algorithm)
6. [Hardware Setup](#hardware-setup)
7. [PlatformIO Build & Deployment](#platformio-build--deployment)
8. [Operation & Validation](#operation--validation)
9. [API Reference](#api-reference)
10. [Troubleshooting](#troubleshooting)
11. [License](#license)

---

## 1. Introduction

SkySonar provides continuous 360° distance awareness via five HC‑SR04 sensors. It uses an adaptive Kalman filter to smooth measurement noise and FreeRTOS to guarantee deterministic timing. Data is published over ROS 2 topics in real time, facilitating seamless integration into research autonomy stacks.

## 2. System Architecture

* **Sensors**: HC‑SR04 ultrasonic modules, one pointing in each cardinal direction and downward.
* **Controller**: ESP32 Wroom (Upesy) running Arduino framework under PlatformIO.
* **RTOS**: FreeRTOS coordinates three main tasks for sensing, publishing, and service handling.
* **Middleware**: micro-ROS client publishes `sensor_msgs/Range` and `diagnostic_msgs/DiagnosticStatus`, offers a `Servocam` service.

Diagram:

```
+--------------+      +-----------+      +---------+
| HC-SR04 (5x) |-->Voltage Divider-->ESP32-->FreeRTOS-->micro-ROS-->ROS2 Agent
+--------------+      +-----------+      +---------+
```

## 3. Electronics Design

### Voltage Divider

To interface 5 V echo outputs with 3.3 V ESP32 GPIOs, each echo pin uses a resistor divider (R1, R2):

```
V_out = V_in × R2 / (R1 + R2)
```

Choosing R1 = 1.8 kΩ and R2 = 3.3 kΩ yields:

```
V_out ≈ 5 V × (3.3 kΩ / 5.1 kΩ) ≈ 3.2 V
```

Component selection:

* R1 ∈ \[1–10 kΩ]
* R2 ∈ \[1–10 kΩ]

Ensure tight tolerance (±1%) for consistent thresholds.

## 4. Software Architecture

### Includes & Dependencies

```cpp
#include <Arduino.h>
#include <NewPing.h>                // Ultrasonic sensor driver
#include <micro_ros_platformio.h>   // micro-ROS Arduino transport
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/ledc.h>             // Servo PWM
#include <servocam_interfaces/srv/servocam.h>
#include <math.h>
```

Key features:

* Fixed-size buffers for frame IDs and messages
* Single `malloc` for diagnostics compatibility
* Build flags in `platformio.ini` enable smooth compilation

### FreeRTOS Scheduling

Three concurrent tasks:

| Task               | Priority | Period  | Function                                          |
| ------------------ | -------- | ------- | ------------------------------------------------- |
| sensorReadingTask  | 3        | 50 ms   | Non-blocking ping, Kalman update, store readings  |
| publishingTask     | 2        | 50 ms   | Publish raw & filtered `sensor_msgs/Range` topics |
| micro-ROS executor | 1        | \~10 ms | Spin executor for services & diagnostics callback |

Each uses:

```c
vTaskDelay(pdMS_TO_TICKS(period_ms));
```

to yield CPU time.

### Inter-task Synchronization

A binary semaphore `data_mutex` protects shared arrays:

```c
// Producer (sensorReadingTask)
xSemaphoreTake(data_mutex, portMAX_DELAY);
raw_readings[i] = raw;
filtered_readings[i] = filtered;
xSemaphoreGive(data_mutex);

// Consumer (publishingTask)
xSemaphoreTake(data_mutex, portMAX_DELAY);
memcpy(local, raw_readings);
xSemaphoreGive(data_mutex);
```

### micro-ROS Integration

* **Initialization**: Serial transport via `set_microros_serial_transports(Serial)`
* **Node & Support**:

  ```c
  ```

allocator = rcl\_get\_default\_allocator();
rclc\_support\_init(\&support, 0, NULL, \&allocator);
rclc\_node\_init\_default(\&node, "ultrasonic\_sensor\_node", "", \&support);

````
- **Publishers**:
  ```c
rclc_publisher_init(&pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), topic, &qos);
````

* **Service**:

  ```c
  ```

rclc\_service\_init\_default(\&servo\_srv, \&node, ROSIDL\_GET\_SRV\_TYPE\_SUPPORT(servocam\_interfaces, srv, Servocam), "servo\_cam\_service");

```

## 5. Kalman Filter Algorithm

Each sensor uses a 1D adaptive Kalman filter:

**Prediction**
```

x\_prior = x\_prev
P\_prior = P\_prev + Q\_prev

```

**Update**
```

K = P\_prior / (P\_prior + R\_prev)
x = x\_prior + K \* (z - x\_prior)
P = (1 - K) \* P\_prior

```

**Adaptive Noise**
```

Innovation = z - x\_prior
R = (1 - α) \* R\_prev + α \* Innovation^2,   α = 0.01
Variance = MeanSquared(meas\_window)
Q = max(Q\_min, Variance × 0.1)

````

This self-tuning filter adapts to changing noise characteristics without manual recalibration.

## 6. Hardware Setup

Follow the [Hardware Requirements](#hardware-setup) section in `README.md` for pin wiring and power.

## 7. PlatformIO Build & Deployment

See `platformio.ini` for dependencies and flags. Use:
```bash
pio run
pio run --target upload
pio device monitor
````

## 8. Operation & Validation

1. Launch micro-ROS Agent:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```
2. Inspect topics:

   ```bash
   ros2 topic list | grep ultrasonic_sensor
   ros2 topic echo /ultrasonic_sensor/downward/filtered
   ros2 topic echo /diagnostics
   ```
3. Invoke servo service:

   ```bash
   ros2 service call /servo_cam_service servocam_interfaces/srv/Servocam "{ angle_deg: 90.0 }"
   ```

## 9. API Reference

* **Topics**:

  * `/ultrasonic_sensor/<name>/raw` (`sensor_msgs/Range`)
  * `/ultrasonic_sensor/<name>/filtered` (`sensor_msgs/Range`)
  * `/diagnostics` (`diagnostic_msgs/DiagnosticStatus`)

* **Service**:

  * `/servo_cam_service` (`servocam_interfaces/Servocam`)

## 10. Troubleshooting

* **Diagnostics missing**: verify IDL in `extra_packages/diagnostic_msgs`
* **Timeouts**: confirm voltage divider and echo wiring
* **Agent disconnect**: match serial params and agent transport

## 11. License

MIT ©
