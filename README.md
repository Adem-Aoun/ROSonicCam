# SkySonar

**ESP32-based multi-directional ultrasonic sensor node for micro‑ROS**

SkySonar is a high-performance PlatformIO/Arduino project for embedded drones and robotics, providing reliable range sensing via five HC‑SR04 sensors. Leveraging native FreeRTOS, an adaptive Kalman filter, and micro‑ROS integration, SkySonar delivers low-latency 20 Hz distance measurements—both raw and filtered—plus real-time health diagnostics and a future-proof servo control service.

---

## 🚀 Key Features

* **Multi‑Sensor Coverage**
  Five directional HC‑SR04 sensors (`downward`, `forward`, `left`, `right`, `back`) for full situational awareness.

* **Adaptive Kalman Filtering**
  Implements a 1D Kalman filter per sensor with:

  * Online estimation of measurement noise (R) via exponential moving variance
  * Dynamic process noise (Q) scaling based on measurement window variance
  * Robust initialization and gain computation

* **Deterministic FreeRTOS Tasks**

  * **sensorReadingTask**: Non‑blocking `NewPing` reads and Kalman update at **20 Hz**
  * **publishingTask**: ROS 2 publishing of raw & filtered data at **20 Hz**
  * micro‑ROS executor: service and diagnostics spin within `loop()` (10 ms slice)

* **Thread Safety**
  Shared `raw_readings` and `filtered_readings` arrays protected by a FreeRTOS **binary semaphore** for atomic access.

* **micro‑ROS Integration**

  * Publishes `sensor_msgs/Range` on `/ultrasonic_sensor/<name>/{raw,filtered}` with accurate ROS 2 timestamps
  * Health status via `diagnostic_msgs/DiagnosticStatus` on `/diagnostics`
  * **servo\_cam\_service** (custom `Servocam.srv`) for future camera gimbal control via LEDC PWM

* **Memory & CPU Efficiency**

  * Fixed-size frame ID and message buffers; only one `malloc` remains (ROS 2 compatibility)
  * No blocking delays in sensor loops; all timing via `millis()` and task delays

---

## 🛠 Hardware Requirements

| Component        | Connection                         |
| ---------------- | ---------------------------------- |
| ESP32 (Upesy)    | USB / 5 V & GND                    |
| HC‑SR04 Triggers | GPIO 4, 17, 18, 32, 21 → VCC (5 V) |
| HC‑SR04 Echoes   | GPIO 16, 14, 22, 35, 25 → GND      |
| Status LED       | GPIO 2 → LED → Resistor → GND      |
| Servo PWM (opt.) | GPIO 26 (LEDC Timer 0, Channel 0)  |

> Ensure all grounds (ESP32, sensors, servo) are common.

### Voltage Divider for Echo Pins

The HC‑SR04 echo output swings up to 5 V, but the ESP32 GPIO pins are only 3.3 V tolerant. To protect the ESP32, each echo line must use a simple resistor divider:

```
V_out = V_in × R2 / (R1 + R2)
```

For example:

* R1 (between sensor echo and ESP32 pin): 1.8 kΩ
* R2 (between ESP32 pin and ground):     3.3 kΩ

This yields:

```
V_out = 5 V × (3.3 kΩ / (1.8 kΩ + 3.3 kΩ)) ≈ 3.2 V
```

Use resistor values in the 1 kΩ–10 kΩ range to limit current without degrading signal edges. Ensure each echo pin has its own divider network.

---

## ⚙️ Software Setup

### 1. Clone & Open Project

```bash
cd ~/Documents/PlatformIO/Projects/
git clone https://github.com/yourusername/sky_sonar.git
cd sky_sonar
```

### 2. Install PlatformIO

* **VSCode**: Install the PlatformIO IDE extension
* **CLI**: `pip install platformio`

### 3. Configure `platformio.ini`

```ini
[env:upesy_wroom]
platform    = espressif32
board       = upesy_wroom
framework   = arduino
monitor_speed = 115200

lib_deps =
  https://github.com/micro-ROS/micro_ros_platformio
  teckel12/NewPing@^1.9.7

build_flags =
  -I include
  -Wno-unused-variable
  -Wno-unused-parameter
```

### 4. Extra Packages (IDL Generation)

Place your custom `Servocam.srv` in `extra_packages/servocam_interfaces/srv/Servocam.srv` and core ROS 2 IDLs (e.g. `diagnostic_msgs/msg/DiagnosticStatus.msg`) in `extra_packages/diagnostic_msgs` so PlatformIO auto‑generates message headers.

### 5. Build & Flash

```bash
# Build
pio run
# Upload to ESP32
pio run --target upload
# Monitor serial output
pio device monitor
```

---

## ▶️ Running & Validation

1. **Start micro‑ROS Agent** on PC:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v
   ```
2. **Verify Topics**:

   ```bash
   ros2 topic list | grep ultrasonic_sensor
   ros2 topic echo /ultrasonic_sensor/downward/filtered
   ros2 topic echo /diagnostics
   ```
3. **Call `servo_cam_service`**:

   ```bash
   ros2 service call /servo_cam_service servocam_interfaces/srv/Servocam "{ angle_deg: 90.0 }"
   ```

---

## 🔍 Code Walkthrough

### 1. Kalman Filter Mathematics

SkySonar implements a scalar Kalman filter for each sensor with the following steps:

**Prediction (Time Update)**

```
P = P + Q
```

* `P` is the estimate error covariance.
* `Q` is the process noise covariance, set adaptively based on the variance of the last 10 measurements.

**Update (Measurement Update)**

```
K = P / (P + R)
x = x + K * (z - x)
P = (1 - K) * P
```

* `z` is the new raw sensor measurement.
* `R` is the estimated measurement noise covariance, updated by:

```
R = (1 - alpha) * R + alpha * (z - x_prior)^2,   alpha = 0.01
```

This adaptive filter balances measurement trust and prediction stability in noisy environments.

### 2. FreeRTOS Task Structure

Tasks are scheduled with fixed priorities to guarantee real-time behavior:

| Task                 | Priority | Role                                 | Rate    |
| -------------------- | -------- | ------------------------------------ | ------- |
| sensorReadingTask    | 3        | Ultrasonic read + Kalman update      | 20 Hz   |
| publishingTask       | 2        | ROS2 message packing & publishing    | 20 Hz   |
| micro-ROS executor\* | 1        | Service callbacks & diagnostics spin | 100 Hz† |

> † Executor is polled every 10 ms inside `loop()`.

Each task uses `vTaskDelay()` for periodic execution without blocking other tasks.

### 3. Semaphore & Thread Safety

A binary semaphore `data_mutex` ensures atomic access to shared arrays:

```c
// Producer (sensorReadingTask)
xSemaphoreTake(data_mutex, portMAX_DELAY);
// write raw_readings[i] and filtered_readings[i]
xSemaphoreGive(data_mutex);

// Consumer (publishingTask)
xSemaphoreTake(data_mutex, portMAX_DELAY);
// copy into local variables for publish
xSemaphoreGive(data_mutex);
```

This prevents concurrent read/write collisions.

### 4. micro-ROS QoS & Service Handling

* **Sensor topics**: Best-Effort reliability, Volatile durability (minimize latency).
* **Diagnostics topic**: Reliable reliability, Transient Local durability (health updates always available).
* **servo\_cam\_service**: Handled by micro-ROS executor slice for sub-10 ms response.

All publishers and the service are initialized before any FreeRTOS tasks start to avoid startup races.

## 🛠 Troubleshooting

* **No `/diagnostics` topic**: Ensure IDL present in `extra_packages/diagnostic_msgs` and rebuild.
* **Sensor timeouts**: Check wiring, confirm echo pins configured as inputs, verify 5 V pulse levels.
* **micro‑ROS errors**: Match agent transport flags (`SERIAL` vs `UDP`), correct Baud and port.

---

## 📄 License

MIT © 
# SkySonar
