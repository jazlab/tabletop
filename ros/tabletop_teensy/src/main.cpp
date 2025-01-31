#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/set_bool.h>
#include <tabletop_msgs/msg/teensy_sensor.h>
#include <tabletop_msgs/srv/set_float.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Define pin mappings
#define ARM_DOOR_CONTROL_PIN 2
#define SMARTGLASS_CONTROL_PIN 3
#define REWARD_CONTROL_PIN 4
#define ARM_DOOR_STATE_PIN 5
#define SMARTGLASS_STATE_PIN 6
#define REWARD_STATE_PIN 7
#define HAND_FIXATION_STATE_PIN 8
#define SYNC_PULSE_PIN 9

// Sync pulse parameters
#define BASE_INTERVAL 1000  // Base interval between sync pulses in ms
#define NOISE_RANGE 200     // Range of jitter in the base interval in ms
#define PULSE_DURATION 100  // Duration of each sync pulse in ms

// Global variables
tabletop_msgs__msg__TeensySensor sensor_msg;

rcl_publisher_t sensor_publisher;
rcl_service_t arm_door_service;
rcl_service_t smartglass_service;
rcl_service_t reward_service;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t sensor_timer;
rcl_timer_t sync_pulse_timer;

std_srvs__srv__SetBool_Request arm_door_request;
std_srvs__srv__SetBool_Response arm_door_response;
std_srvs__srv__SetBool_Request smartglass_request;
std_srvs__srv__SetBool_Response smartglass_response;
tabletop_msgs__srv__SetFloat_Request reward_request;
tabletop_msgs__srv__SetFloat_Response reward_response;

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      error_loop();                                                                                                    \
    }                                                                                                                  \
  }
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
    }                                                                                                                  \
  }

// Error handle loop
void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

// Service callback for arm_door
void arm_door_callback(const void* req, void* res)
{
  std_srvs__srv__SetBool_Request* request = (std_srvs__srv__SetBool_Request*)req;
  std_srvs__srv__SetBool_Response* response = (std_srvs__srv__SetBool_Response*)res;

  digitalWrite(ARM_DOOR_CONTROL_PIN, request->data);
  if (request->data)
  {
    rosidl_runtime_c__String__assign(&response->message, "Arm door opened");
  }
  else
  {
    rosidl_runtime_c__String__assign(&response->message, "Arm door closed");
  }
  response->success = true;
}

// Service callback for smartglass
void smartglass_callback(const void* req, void* res)
{
  std_srvs__srv__SetBool_Request* request = (std_srvs__srv__SetBool_Request*)req;
  std_srvs__srv__SetBool_Response* response = (std_srvs__srv__SetBool_Response*)res;

  digitalWrite(SMARTGLASS_CONTROL_PIN, request->data);
  if (request->data)
  {
    rosidl_runtime_c__String__assign(&response->message, "Smartglass revealed");
  }
  else
  {
    rosidl_runtime_c__String__assign(&response->message, "Smartglass occluded");
  }
  response->success = true;
}

// Service callback for reward
void reward_callback(const void* req, void* res)
{
  tabletop_msgs__srv__SetFloat_Request* request = (tabletop_msgs__srv__SetFloat_Request*)req;
  tabletop_msgs__srv__SetFloat_Response* response = (tabletop_msgs__srv__SetFloat_Response*)res;

  digitalWrite(REWARD_CONTROL_PIN, HIGH);
  delay(uint32_t(request->data * 1000));
  digitalWrite(REWARD_CONTROL_PIN, LOW);

  rosidl_runtime_c__String__assign(&response->message, "Reward delivered");
  response->success = true;
}

void sensor_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Populate sensor message
    sensor_msg.arm_door_laser_state_right = digitalRead(ARM_DOOR_STATE_PIN);
    sensor_msg.arm_door_laser_state_left = digitalRead(ARM_DOOR_STATE_PIN);  // Assuming same pin for both
    sensor_msg.smart_glass_laser_state = digitalRead(SMARTGLASS_STATE_PIN);
    sensor_msg.fixation_button_state = digitalRead(HAND_FIXATION_STATE_PIN);
    for (int i = 0; i < 5; i++)
    {
      sensor_msg.tactile_glove_states[i] = (float)random(0, 100) / 100.0f;  // Simulate tactile glove data
    }
    sensor_msg.sync_pulse = digitalRead(SYNC_PULSE_PIN);

    RCSOFTCHECK(rcl_publish(&sensor_publisher, &sensor_msg, NULL));
  }
}

void sync_pulse_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Generate a random delay between 0 and NOISE_RANGE
    long delay_ms = random(0, NOISE_RANGE);

    // Wait for the delay
    delay(delay_ms);

    // Turn pulse pin ON
    digitalWrite(SYNC_PULSE_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);

    // Keep pulse pin ON for the pulse duration
    delay(PULSE_DURATION);

    // Turn pulse pin OFF
    digitalWrite(SYNC_PULSE_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup()
{
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&sensor_publisher, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, msg, TeensySensor), "sensors"));

  // create services
  RCCHECK(rclc_service_init_default(&arm_door_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                    "arm_door"));
  RCCHECK(rclc_service_init_default(&smartglass_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                    "smartglass"));
  RCCHECK(rclc_service_init_default(&reward_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, SetFloat),
                                    "reward"));

  // create timers
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&sensor_timer, &support, RCL_MS_TO_NS(timer_timeout), sensor_timer_callback));
  RCCHECK(rclc_timer_init_default(&sync_pulse_timer, &support, RCL_MS_TO_NS(timer_timeout), sync_pulse_timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_timer));
  RCCHECK(rclc_executor_add_service(&executor, &arm_door_service, &arm_door_request, &arm_door_response,
                                    &arm_door_callback));
  RCCHECK(rclc_executor_add_service(&executor, &smartglass_service, &smartglass_request, &smartglass_response,
                                    &smartglass_callback));
  RCCHECK(rclc_executor_add_service(&executor, &reward_service, &reward_request, &reward_response, &reward_callback));

  // Initialize pin states (set pins as INPUT or OUTPUT as needed)
  pinMode(ARM_DOOR_CONTROL_PIN, OUTPUT);
  pinMode(SMARTGLASS_CONTROL_PIN, OUTPUT);
  pinMode(REWARD_CONTROL_PIN, OUTPUT);
  pinMode(ARM_DOOR_STATE_PIN, INPUT);
  pinMode(SMARTGLASS_STATE_PIN, INPUT);
  pinMode(REWARD_STATE_PIN, INPUT);
  pinMode(HAND_FIXATION_STATE_PIN, INPUT);
  pinMode(SYNC_PULSE_PIN, OUTPUT);
}

void loop()
{
  rclc_executor_spin(&executor);
}
