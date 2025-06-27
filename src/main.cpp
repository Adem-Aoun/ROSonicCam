#include <Arduino.h>
#include <NewPing.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/ledc.h>
#include <servocam_interfaces/srv/servocam.h>
#include <math.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example requires Arduino framework with serial transport.
#endif

#define NUM_SENSORS 5
const char* SENSOR_NAMES[] = {"downward", "forward", "left", "right", "back"};
const int TRIG_PINS[]   = {4,  17, 18, 32, 21};
const int ECHO_PINS[]   = {16, 14, 22, 35, 25};
#define MAX_RANGE_CM 450

#define FRAME_ID_BUFFER_SIZE 30
#define MESSAGE_BUFFER_SIZE 100
#define DIAGNOSTICS_BUFFER_SIZE 64

NewPing sensors[NUM_SENSORS] = {
  NewPing(TRIG_PINS[0], ECHO_PINS[0], MAX_RANGE_CM),
  NewPing(TRIG_PINS[1], ECHO_PINS[1], MAX_RANGE_CM),
  NewPing(TRIG_PINS[2], ECHO_PINS[2], MAX_RANGE_CM),
  NewPing(TRIG_PINS[3], ECHO_PINS[3], MAX_RANGE_CM),
  NewPing(TRIG_PINS[4], ECHO_PINS[4], MAX_RANGE_CM)
};

#define SERVO_PIN 26
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_CHANNEL LEDC_CHANNEL_0
#define SERVO_RESOLUTION LEDC_TIMER_12_BIT
#define SERVO_FREQ 50
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

#define MIN_RANGE 0.02f
#define MAX_RANGE 4.00f
#define FOV 0.26f

#define WINDOW_SIZE 10 
#define ALPHA 0.01f    

#define SENSOR_CHECK_INTERVAL 5000
#define SENSOR_READ_INTERVAL 50
#define PUBLISH_INTERVAL 50
#define DIAGNOSTICS_INTERVAL 1000

struct KalmanFilter {
  float x;           
  float P;           
  float Q;           
  float R;            
  float K;            
  bool initialized;
  
  float R_estimate;    
  float innovation;   
  float measurement_window[WINDOW_SIZE]; 
  int window_index;    
};

char frame_id_buffers[NUM_SENSORS * 2][FRAME_ID_BUFFER_SIZE];
char servo_response_buffer[MESSAGE_BUFFER_SIZE];
char diagnostics_message_buffer[DIAGNOSTICS_BUFFER_SIZE];

float raw_readings[NUM_SENSORS];
float filtered_readings[NUM_SENSORS];
KalmanFilter kf[NUM_SENSORS]; 
SemaphoreHandle_t data_mutex; 
bool hardware_ok = true;      
bool servo_configured = false; 

unsigned long last_sensor_check = 0;
unsigned long last_diagnostics = 0;

rcl_publisher_t filtered_publishers[NUM_SENSORS];
rcl_publisher_t raw_publishers[NUM_SENSORS];
rcl_publisher_t diagnostics_publisher;
sensor_msgs__msg__Range filtered_msgs[NUM_SENSORS];
sensor_msgs__msg__Range raw_msgs[NUM_SENSORS];
diagnostic_msgs__msg__DiagnosticStatus diagnostics_msg;
rcl_service_t servo_service;
servocam_interfaces__srv__Servocam_Response servo_res;
servocam_interfaces__srv__Servocam_Request servo_req;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; }

void error_loop() {
  while(1) {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }
}

void initKalmanFilter(KalmanFilter* kf) {
  kf->x = 0.0f;
  kf->P = 1.0f;
  kf->Q = 0.01f; 
  kf->R = 0.1f;  
  kf->initialized = false;
  kf->R_estimate = kf->R;
  kf->window_index = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    kf->measurement_window[i] = 0.0f;
  }
}

float computeVariance(KalmanFilter* kf) {
  float mean = 0.0f;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    mean += kf->measurement_window[i];
  }
  mean /= WINDOW_SIZE;
  
  float variance = 0.0f;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    variance += pow(kf->measurement_window[i] - mean, 2);
  }
  return variance / WINDOW_SIZE;
}

float kalmanFilter(KalmanFilter* kf, float measurement) {
  if (!kf->initialized) {
    kf->x = measurement;
    kf->initialized = true;
    kf->measurement_window[0] = measurement;
    kf->window_index = 1;
    return measurement;
  }
  
  kf->P += kf->Q;
  kf->innovation = measurement - kf->x;
  kf->R_estimate = (1 - ALPHA) * kf->R_estimate + ALPHA * (kf->innovation * kf->innovation);
  kf->R = kf->R_estimate;  
  float variance = computeVariance(kf);
  kf->Q = max(0.001f, variance * 0.1f);  
  kf->K = kf->P / (kf->P + kf->R);
  kf->x += kf->K * kf->innovation;
  kf->P = (1.0f - kf->K) * kf->P;
  kf->measurement_window[kf->window_index % WINDOW_SIZE] = measurement;
  kf->window_index = (kf->window_index + 1) % WINDOW_SIZE;
  return kf->x;
}

double get_time_seconds() {
  return millis() / 1000.0;
}

float readDistance(int index) {
  unsigned int distance_cm = sensors[index].ping_cm();
  if (distance_cm == 0) return NAN;
  
  float distance_m = distance_cm / 100.0f;
  if (distance_m < MIN_RANGE || distance_m > MAX_RANGE) {
    return NAN;
  }
  return distance_m;
}

void fillRangeMessage(sensor_msgs__msg__Range* msg, float range, const char* suffix, double timestamp, char* frame_id_buffer) {
  msg->header.stamp.sec = (int32_t)timestamp;
  msg->header.stamp.nanosec = (uint32_t)((timestamp - msg->header.stamp.sec) * 1e9);
  snprintf(frame_id_buffer, FRAME_ID_BUFFER_SIZE, "ultrasonic_%s", suffix);
  msg->header.frame_id.data = frame_id_buffer;
  msg->header.frame_id.size = strlen(frame_id_buffer);
  msg->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  msg->field_of_view = FOV;
  msg->min_range = MIN_RANGE;
  msg->max_range = MAX_RANGE;
  msg->range = (isnan(range)) ? NAN : range;
}

void servo_service_callback(const void* req, void* res) {
  servocam_interfaces__srv__Servocam_Request* request = (servocam_interfaces__srv__Servocam_Request*) req;
  servocam_interfaces__srv__Servocam_Response* response = (servocam_interfaces__srv__Servocam_Response*) res;

  double angle_deg = request->angle_deg;
  if (angle_deg < 0 || angle_deg > 180) {
    response->success = false;
    const char* error_msg = "Error: Angle must be between 0-180 degrees";
    strncpy(servo_response_buffer, error_msg, MESSAGE_BUFFER_SIZE - 1);
    servo_response_buffer[MESSAGE_BUFFER_SIZE - 1] = '\0';
    response->message.data = servo_response_buffer;
    response->message.size = strlen(servo_response_buffer);
    return;
  }

  double pulse_us = SERVO_MIN_US + (angle_deg / 180.0) * (SERVO_MAX_US - SERVO_MIN_US);
  double duty = (pulse_us * (1 << SERVO_RESOLUTION)) / (1000000.0 / SERVO_FREQ);
  
  if (servo_configured) {
    uint32_t prev_duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, (uint32_t)duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
    
    uint32_t new_duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
    
    if (abs((int32_t)new_duty - (int32_t)prev_duty) > 10) {
      response->success = true;
      const char* success_msg = "Servo position set successfully";
      strncpy(servo_response_buffer, success_msg, MESSAGE_BUFFER_SIZE - 1);
      servo_response_buffer[MESSAGE_BUFFER_SIZE - 1] = '\0';
      response->message.data = servo_response_buffer;
      response->message.size = strlen(servo_response_buffer);
    } else {
      response->success = false;
      const char* error_msg = "Error: Servo did not respond";
      strncpy(servo_response_buffer, error_msg, MESSAGE_BUFFER_SIZE - 1);
      servo_response_buffer[MESSAGE_BUFFER_SIZE - 1] = '\0';
      response->message.data = servo_response_buffer;
      response->message.size = strlen(servo_response_buffer);
    }
  } else {
    response->success = false;
    const char* error_msg = "Error: Servo not configured";
    strncpy(servo_response_buffer, error_msg, MESSAGE_BUFFER_SIZE - 1);
    servo_response_buffer[MESSAGE_BUFFER_SIZE - 1] = '\0';
    response->message.data = servo_response_buffer;
    response->message.size = strlen(servo_response_buffer);
  }
}

void checkSensors() {
  Serial.println("\n--- Sensor Status Report ---");
  hardware_ok = true;  
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("Sensor %s (TRIG:%d, ECHO:%d): ", 
                 SENSOR_NAMES[i], TRIG_PINS[i], ECHO_PINS[i]);
    
    float dist = readDistance(i);
    if (!isnan(dist)) {
      Serial.printf("OK (%.2fm)\n", dist);
    } else {
      Serial.println("FAILED");
      hardware_ok = false;
    }
  }
  Serial.println("----------------------------");
  
  digitalWrite(2, hardware_ok ? HIGH : LOW);
}

void publishDiagnostics() {
  if (hardware_ok) {
    diagnostics_msg.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
    const char* message = "All sensors operational";
    strncpy(diagnostics_message_buffer, message, DIAGNOSTICS_BUFFER_SIZE - 1);
  } else {
    diagnostics_msg.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
    const char* message = "One or more sensors failed";
    strncpy(diagnostics_message_buffer, message, DIAGNOSTICS_BUFFER_SIZE - 1);
  }
  diagnostics_message_buffer[DIAGNOSTICS_BUFFER_SIZE - 1] = '\0';
  
  diagnostics_msg.message.data = diagnostics_message_buffer;
  diagnostics_msg.message.size = strlen(diagnostics_message_buffer);
  
  RCSOFTCHECK(rcl_publish(&diagnostics_publisher, &diagnostics_msg, NULL));
}

void sensorReadingTask(void *pvParameters) {
  while (1) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      float raw = readDistance(i);
      float filtered = NAN;
      
      if (!isnan(raw)) {
        filtered = kalmanFilter(&kf[i], raw);
      }
      if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        raw_readings[i] = raw;
        filtered_readings[i] = filtered;
        xSemaphoreGive(data_mutex);
      }
      if (!isnan(raw)) {
        Serial.printf("[%s] Raw: %.2fm | Filtered: %.2fm\n",
                     SENSOR_NAMES[i], raw, filtered);
      } else {
        Serial.printf("[%s] No valid reading\n", SENSOR_NAMES[i]);
      }
      
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

void publishingTask(void *pvParameters) {
  while (1) {
    double now = get_time_seconds();
    float current_raw[NUM_SENSORS];
    float current_filtered[NUM_SENSORS];
    
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        current_raw[i] = raw_readings[i];
        current_filtered[i] = filtered_readings[i];
      }
      xSemaphoreGive(data_mutex);
    }
    
    for (int i = 0; i < NUM_SENSORS; i++) {
      fillRangeMessage(&raw_msgs[i], current_raw[i], SENSOR_NAMES[i], now, 
                       frame_id_buffers[i * 2]);
      fillRangeMessage(&filtered_msgs[i], current_filtered[i], SENSOR_NAMES[i], now, 
                       frame_id_buffers[i * 2 + 1]);
      
      RCSOFTCHECK(rcl_publish(&raw_publishers[i], &raw_msgs[i], NULL));
      RCSOFTCHECK(rcl_publish(&filtered_publishers[i], &filtered_msgs[i], NULL));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  pinMode(2, OUTPUT); 
  digitalWrite(2, LOW);
  Serial.begin(115200);
  delay(3000);  
  Serial.println("\n\nStarting Ultrasonic Sensor System...");
  checkSensors();

  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = SERVO_RESOLUTION,
      .timer_num = SERVO_TIMER,
      .freq_hz = SERVO_FREQ,
      .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t ch_conf = {
      .gpio_num = SERVO_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = SERVO_CHANNEL,
      .timer_sel = SERVO_TIMER,
      .duty = 0,
      .hpoint = 0
  };
  if (ledc_channel_config(&ch_conf) == ESP_OK) {
    servo_configured = true;
    Serial.println("Servo configured successfully");
  } else {
    Serial.println("Servo configuration failed!");
  }

  data_mutex = xSemaphoreCreateMutex();
  set_microros_serial_transports(Serial);
  delay(1000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "ultrasonic_sensor_node", "", &support));

  rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
  sensor_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  sensor_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  sensor_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  sensor_qos.depth = 1;

  rmw_qos_profile_t diagnostics_qos = rmw_qos_profile_default;
  diagnostics_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  diagnostics_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  diagnostics_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  diagnostics_qos.depth = 5;

  for (int i = 0; i < NUM_SENSORS; i++) {
    initKalmanFilter(&kf[i]);
    
    // Create publishers with QoS
    RCCHECK(rclc_publisher_init(
      &raw_publishers[i],
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      (std::string("ultrasonic_sensor/") + SENSOR_NAMES[i] + "/raw").c_str(),
      &sensor_qos
    ));
    RCCHECK(rclc_publisher_init(
      &filtered_publishers[i],
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      (std::string("ultrasonic_sensor/") + SENSOR_NAMES[i] + "/filtered").c_str(),
      &sensor_qos
    ));

    // Initialize message buffers
    raw_msgs[i].header.frame_id.data = frame_id_buffers[i * 2];
    raw_msgs[i].header.frame_id.capacity = FRAME_ID_BUFFER_SIZE;
    raw_msgs[i].header.frame_id.size = 0;
    filtered_msgs[i].header.frame_id.data = frame_id_buffers[i * 2 + 1];
    filtered_msgs[i].header.frame_id.capacity = FRAME_ID_BUFFER_SIZE;
    filtered_msgs[i].header.frame_id.size = 0;
    
    raw_readings[i] = NAN;
    filtered_readings[i] = NAN;
  }

  RCCHECK(rclc_publisher_init(
    &diagnostics_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
    "diagnostics",
    &diagnostics_qos
  ));

  diagnostics_msg.name.data = (char*)malloc(DIAGNOSTICS_BUFFER_SIZE);
  diagnostics_msg.name.capacity = DIAGNOSTICS_BUFFER_SIZE;
  diagnostics_msg.name.size = 0;
  diagnostics_msg.message.data = diagnostics_message_buffer;
  diagnostics_msg.message.capacity = DIAGNOSTICS_BUFFER_SIZE;
  diagnostics_msg.message.size = 0;
  diagnostics_msg.hardware_id.data = (char*)"ESP32";
  diagnostics_msg.hardware_id.size = 5;
  diagnostics_msg.hardware_id.capacity = 5;

  const char* name = "ultrasonic_sensor_system";
  strncpy(diagnostics_msg.name.data, name, DIAGNOSTICS_BUFFER_SIZE - 1);
  diagnostics_msg.name.data[DIAGNOSTICS_BUFFER_SIZE - 1] = '\0';
  diagnostics_msg.name.size = strlen(diagnostics_msg.name.data);

  servo_res.success = false;
  servo_res.message.data = servo_response_buffer;
  servo_res.message.capacity = MESSAGE_BUFFER_SIZE;
  servo_res.message.size = 0;

  RCCHECK(rclc_service_init_default(
    &servo_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(servocam_interfaces, srv, Servocam),
    "servo_cam_service"
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &servo_service, &servo_req, &servo_res, servo_service_callback));
  
  xTaskCreate(
    sensorReadingTask,
    "SensorTask",  
    4096,          
    NULL,
    3,             
    NULL
  );
  
  xTaskCreate(
    publishingTask,
    "PublishTask", 
    4096,         
    NULL,
    2,             
    NULL
  );
  
  Serial.println("\nSystem initialization complete!");
  Serial.println("micro-ROS connection active");
  Serial.printf("Hardware status: %s\n", hardware_ok ? "ALL SENSORS OK" : "DEGRADED - CHECK SENSORS");
  Serial.printf("Servo status: %s\n", servo_configured ? "CONFIGURED" : "NOT CONFIGURED");
  Serial.println("Topics:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("  /ultrasonic_sensor/%s/raw\n", SENSOR_NAMES[i]);
    Serial.printf("  /ultrasonic_sensor/%s/filtered\n", SENSOR_NAMES[i]);
  }
  Serial.println("  /diagnostics");
  Serial.println("Service: /servo_cam_service");
}

void loop() {
  unsigned long current_time = millis();
  
  if (current_time - last_sensor_check >= SENSOR_CHECK_INTERVAL) {
    checkSensors();
    last_sensor_check = current_time;
  }
  
  if (current_time - last_diagnostics >= DIAGNOSTICS_INTERVAL) {
    publishDiagnostics();
    last_diagnostics = current_time;
  }
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}
