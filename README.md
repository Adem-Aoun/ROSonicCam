```markdown
# SkySonar

[![Build](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/Adem-Aoun/SkySonar/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**ESP32-based multi-directional ultrasonic sensor node for micro-ROS**

SkySonar provides deterministic range sensing with adaptive filtering and ROS 2 integration. It features:
- Five HC-SR04 sensors arranged omnidirectionally
- Real-time Kalman filtering with adaptive noise estimation
- FreeRTOS task prioritization for timing guarantees
- Servo control via ROS 2 services
- Diagnostic monitoring and hardware validation

---

## 📋 Table of Contents
1. [Introduction](#introduction)
2. [System Architecture](#system-architecture)
3. [Electronics Design](#electronics-design)
4. [Software Architecture](#software-architecture)
5. [Kalman Filter Algorithm](#kalman-filter-algorithm)
6. [Hardware Setup](#hardware-setup)
7. [PlatformIO Build & Deployment](#platformio-build--deployment)
8. [Operation & Validation](#operation--validation)
9. [API Reference](#api-reference)
10. [Troubleshooting](#troubleshooting)
11. [License](#license)

---

## 1. Introduction <a name="introduction"></a>
SkySonar delivers high-fidelity range measurements for robotics applications through:
- **Sensor Fusion**: Five HC-SR04 units arranged downward, forward, left, right, and backward
- **Noise Reduction**: Adaptive Kalman filtering with runtime-tuned Q/R values
- **Real-Time OS**: FreeRTOS task prioritization for sensor polling (high priority) and publishing (medium priority)
- **ROS 2 Integration**: Raw and filtered `/ultrasonic_sensor/<direction>` topics + `/diagnostics` status reporting

---

## 2. System Architecture <a name="system-architecture"></a>
```mermaid
graph LR
    subgraph Sensor Array
        D[Downward\nHC-SR04]:::sensor
        F[Forward\nHC-SR04]:::sensor
        L[Left\nHC-SR04]:::sensor
        R[Right\nHC-SR04]:::sensor
        B[Back\nHC-SR04]:::sensor
    end
    
    subgraph Signal Conditioning
        VD[Voltage Divider\n1.8kΩ/3.3kΩ]:::circuit
    end
    
    subgraph Processing Unit
        ESP[ESP32 Wroom]:::mcu
        RTOS[FreeRTOS]:::os
            RTOS --> T1[Sensor Task]:::task
            RTOS --> T2[Publish Task]:::task
    end
    
    subgraph ROS Ecosystem
        ROS2[ROS 2 Foxy/Humble]:::ros
            ROS2 --> TP1[/raw\nRange Msg/]:::topic
            ROS2 --> TP2[/filtered\nRange Msg/]:::topic
            ROS2 --> DP[/diagnostics\nStatus/]:::topic
            ROS2 --> SV[servo_cam_service]:::service
    end
    
    Sensor Array --> VD
    VD --> ESP
    ESP -->|micro-ROS| ROS2
    
    classDef sensor fill:#9f9,stroke:#333;
    classDef circuit fill:#f96,stroke:#333;
    classDef mcu fill:#69f,stroke:#333;
    classDef os fill:#aaf,stroke:#333;
    classDef task fill:#ff9,stroke:#333;
    classDef ros fill:#f9f,stroke:#333;
    classDef topic fill:#bfb,stroke:#333;
    classDef service fill:#fbb,stroke:#333;
```

---

## 3. Electronics Design <a name="electronics-design"></a>
### Voltage Divider Circuit
⚠️ **Important Note**: GPIOs used (16,14,22,35,25) require signal conditioning unless your ESP32 variant explicitly supports 5V-tolerant inputs.

```mermaid
circuit LR
    VCC[5V Echo] --> R1[1.8kΩ]
    R1 --> R2[3.3kΩ]
    R2 --> GND
    VOUT --> ESP_GPIO
    classDef resistor fill:#ff9,stroke:#333;
    class R1,R2 resistor;
```

**Component Specifications**:
- R1: 1.8kΩ ±1% tolerance
- R2: 3.3kΩ ±1% tolerance
- Max current: 5V / 5.1kΩ ≈ 0.98mA

---

## 4. Software Architecture <a name="software-architecture"></a>
### Task Scheduling
```mermaid
gantt
    title FreeRTOS Task Prioritization
    dateFormat  ms
    axisFormat %L
    
    section High Priority
    Sensor Polling     :a1, 0, 10
    Kalman Update      :a2, after a1, 10
    Data Storage       :a3, after a2, 5
    
    section Medium Priority
    Data Serialization :b1, 0, 15
    ROS Publishing     :b2, after b1, 10
    
    section Main Loop
    Service Handling   :c1, 10, 8
    Executor Spin      :c2, after c1, 12
```

### Key Implementation Details
```c
// Shared memory protection
SemaphoreHandle_t data_mutex = xSemaphoreCreateMutex();

// Kalman filter structure
struct KalmanFilter {
  float x;           // State estimate
  float P;           // Estimate covariance
  float Q;           // Process noise
  float R;           // Measurement noise
  float innovation;  // Residual
};

// ROS 2 topic initialization
rclc_publisher_init(
  &filtered_pub, 
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
  "ultrasonic_sensor/filtered",
  &sensor_qos
);
```

---

## 5. Kalman Filter Algorithm <a name="kalman-filter-algorithm"></a>
```mermaid
flowchart TD
    Start[New Measurement] --> Predict
    Predict["x_prior = x_prev\nP_prior = P_prev + Q"] --> Update
    Update["Innovation = z - x_prior\nK = P_prior / (P_prior + R)\nx = x_prior + K*Innovation\nP = (1-K)*P_prior"] --> Adapt
    Adapt["R = (1-α)R_prev + α*Innovation²\nQ = max(Q_min, variance*0.1)"] --> End[Output]
```

---

## 6. Hardware Setup <a name="hardware-setup"></a>
**Pin Mapping**:
| Sensor     | TRIG Pin | ECHO Pin |
|------------|----------|----------|
| Downward   | GPIO4    | GPIO16   |
| Forward    | GPIO17   | GPIO14   |
| Left       | GPIO18   | GPIO22   |
| Right      | GPIO32   | GPIO35   |
| Back       | GPIO21   | GPIO25   |
| **Servo**  | GPIO26   |          |

> ⚙️ Configuration Tip: TRIG/ECHO pins are defined in `TRIG_PINS[]` and `ECHO_PINS[]` arrays for easy reassignment.

**Power Requirements**:
- 5V/2A power supply
- Decoupling capacitor: 100μF across 5V/GND

---

## 7. PlatformIO Build & Deployment <a name="platformio-build--deployment"></a>
```ini
[env:upesy_wroom]
platform = espressif32
board = upesy_wroom
framework = arduino
lib_deps = 
    hideakitai/NewPing@^1.9.4
    micro-ROS/micro_ros_platformio@^0.3.0
```

---

## 8. Operation & Validation <a name="operation--validation"></a>
**Diagnostic Checks**:
```bash
$ ros2 topic echo /diagnostics
level: 0
name: "ultrasonic_sensor_system"
message: "All sensors operational"
hardware_id: "ESP32"
```

**Service Call Example**:
```bash
$ ros2 service call /servo_cam_service servocam_interfaces/srv/Servocam "{angle_deg: 45}"
requester: making request: servocam_interfaces.srv.Servocam_Request(angle_deg=45.0)

response:
servocam_interfaces.srv.Servocam_Response(success=True, message="Servo position set successfully")
```

---

## 9. API Reference <a name="api-reference"></a>
**Filtered Range Message**:
```yaml
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: "ultrasonic_downward"
radiation_type: 0  # ULTRASOUND
field_of_view: 0.26
min_range: 0.02
max_range: 4.0
range: 1.85
```

---

## 10. Troubleshooting <a name="troubleshooting"></a>
| Symptom               | Solution                          |
|-----------------------|-----------------------------------|
| No ROS 2 topics       | Verify micro-ROS agent connection |
| Servo not responding  | Check PWM configuration (duty cycle delta >10) |
| Intermittent readings | Inspect voltage divider circuits  |

---

## 11. License <a name="license"></a>
MIT © 2023 