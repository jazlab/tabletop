#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/set_bool.h>
#include <tabletop_msgs/msg/teensy_sensor.h>
#include <tabletop_msgs/srv/get_arm_door.h>
#include <tabletop_msgs/srv/get_hand_fixation.h>
#include <tabletop_msgs/srv/get_reward.h>
#include <tabletop_msgs/srv/set_arm_door.h>
#include <tabletop_msgs/srv/set_reward.h>
#include <tabletop_msgs/srv/set_smartglass.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#  error This code only supports serial transport.
#endif

// #define ARM_DOOR_ASSERTIONS
// #define DEBUG_LOGGING

// Define pin mappings

#define ARM_DOOR_OPEN_CONTROL_PIN 1
#define ARM_DOOR_CLOSE_CONTROL_PIN 1
#define SMARTGLASS_CONTROL_PIN 3
#define REWARD_CONTROL_PIN 4
#define HAND_FIXATION_STATE_PIN 34
#define ARM_DOOR_CLOSED_STATE_PIN 37
#define SYNC_PULSE_PIN 9
static const uint8_t GLOVE_STATE_PINS[] = {A0, A1, A2, A3, A4};

// Message memory configuration

#define MAX_STRING_CAPACITY 100
#define MAX_ROS2_TYPE_SEQUENCE_CAPACITY 5
#define MAX_BASIC_TYPE_SEQUENCE_CAPACITY 5
static const micro_ros_utilities_memory_conf_t memory_conf = {
    MAX_STRING_CAPACITY,
    MAX_ROS2_TYPE_SEQUENCE_CAPACITY,
    MAX_BASIC_TYPE_SEQUENCE_CAPACITY,
    NULL,
    0,
    NULL};

// ROS2 topics

#define SENSORS_TOPIC "/teensy/sensors"
#define LOG_TOPIC "/teensy/log"

// ROS2 services

#define SET_ARM_DOOR_SRV_NAME "/teensy/set_arm_door"
#define GET_ARM_DOOR_SRV_NAME "/teensy/get_arm_door"
#define SET_SMARTGLASS_SRV_NAME "/teensy/set_smartglass"
#define SET_REWARD_SRV_NAME "/teensy/set_reward"
#define GET_REWARD_SRV_NAME "/teensy/get_reward"
#define GET_HAND_FIXATION_SRV_NAME "/teensy/get_hand_fixation"

// Execution parameters

#define AGENT_RECONNECT_PERIOD_MS 100 // Timeout for agent reconnection, in ms
#define AGENT_RECONNECT_TIMEOUT_MS 50 // Timeout for agent reconnection, in ms
#define EXECUTOR_SPIN_TIMEOUT_MS 20   // Timeout for executor spin, in ms
#define AGENT_CHECK_CONNECTED_PERIOD_MS \
  5000 // Period for checking if the agent is connected, in ms
#define AGENT_CHECK_CONNECTED_TIMEOUT_MS \
  10 // Timeout for checking if the agent is connected, in ms
#define SENSOR_PERIOD_MS 10            // Sensor update period, in ms
#define SYNC_PULSE_BASE_PERIOD_MS 1000 // Base period between sync pulses, in ms
#define SYNC_PULSE_DELAY_RANGE_MS \
  200                              // Range of jitter in the base period, in ms
#define SYNC_PULSE_DURATION_MS 100 // Duration of each sync pulse, in ms
#define ARM_DOOR_PERIOD_MS 1000    // Period for arm door control, in ms

// Agent reconnection parameters

// Maximum number of retries for agent reconnect before giving up
#define AGENT_RECONNECT_MAX_RETRIES 10

// Global variables

rcl_publisher_t sensor_publisher;
rcl_publisher_t log_publisher;

rcl_service_t set_arm_door_service;
rcl_service_t get_arm_door_service;
rcl_service_t set_smartglass_service;
rcl_service_t set_reward_service;
rcl_service_t get_reward_service;
rcl_service_t get_hand_fixation_service;

rcl_timer_t sync_pulse_base_timer;
rcl_timer_t sync_pulse_start_timer;
rcl_timer_t sync_pulse_end_timer;
rcl_timer_t sensor_timer;
rcl_timer_t reward_timer;
rcl_timer_t arm_door_timer;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

tabletop_msgs__msg__TeensySensor sensor_msg;
std_msgs__msg__String log_msg;

tabletop_msgs__srv__SetArmDoor_Request set_arm_door_request;
tabletop_msgs__srv__SetArmDoor_Response set_arm_door_response;
tabletop_msgs__srv__GetArmDoor_Request get_arm_door_request;
tabletop_msgs__srv__GetArmDoor_Response get_arm_door_response;
tabletop_msgs__srv__SetSmartglass_Request set_smartglass_request;
tabletop_msgs__srv__SetSmartglass_Response set_smartglass_response;
tabletop_msgs__srv__SetReward_Request set_reward_request;
tabletop_msgs__srv__SetReward_Response set_reward_response;
tabletop_msgs__srv__GetReward_Request get_reward_request;
tabletop_msgs__srv__GetReward_Response get_reward_response;
tabletop_msgs__srv__GetHandFixation_Request get_hand_fixation_request;
tabletop_msgs__srv__GetHandFixation_Response get_hand_fixation_response;

// State tracking
static uint8_t agent_reconnect_retries;

static enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  CLIENT_ERROR
} state;

static enum arm_door_states {
  ARM_DOOR_OPEN,
  ARM_DOOR_OPENING,
  ARM_DOOR_CLOSED,
  ARM_DOOR_CLOSING
} arm_door_state;

bool sync_pulse_state;
int64_t sync_pulse_last_time_on_ms;
int64_t sync_pulse_last_time_off_ms;
int64_t hand_fixation_last_time_pressed_ms;
int64_t hand_fixation_last_time_released_ms;
bool reward_active;

// Macro definitions

// Check return code from ROS2 function, print error string and return false if
// error
#define RCCHECK(fn)                                      \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK)) {                       \
      printf("Error: %s\n", rcl_get_error_string().str); \
      return false;                                      \
    }                                                    \
  }
// Check return code from ROS2 function, continue execution
#define RCSOFTCHECK(fn)                                  \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK)) {                       \
      printf("Error: %s\n", rcl_get_error_string().str); \
    }                                                    \
  }
// Check return code from ROS2 function, logging an error if the return code is
// not OK
#define RCASSERT(fn, fmt, ...)           \
  {                                      \
    rcl_ret_t temp_rc = fn;              \
    if ((temp_rc != RCL_RET_OK)) {       \
      LOG("RC Assertion failed!");       \
      LOG("Error number: %ld", temp_rc); \
      LOG(fmt, ##__VA_ARGS__);           \
    }                                    \
  }
// Set the string message
#define STRING_SET(str_ptr, fmt, ...)                                   \
  {                                                                     \
    snprintf((str_ptr)->data, (str_ptr)->capacity, fmt, ##__VA_ARGS__); \
    (str_ptr)->size = strlen((str_ptr)->data);                          \
  }
// Log a message to the ROS2 log topic
#define LOG(fmt, ...)                                         \
  {                                                           \
    STRING_SET(&log_msg.data, fmt, ##__VA_ARGS__);            \
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL)); \
  }
// Assert a condition, logging an error if the condition is not met
#define ASSERT(fn, fmt, ...)    \
  {                             \
    if (!(fn)) {                \
      LOG("Assertion failed!"); \
      LOG(fmt, ##__VA_ARGS__);  \
    }                           \
  }

// Execute a block of code every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)   \
  {                                 \
    static int64_t init = -1;       \
    if (init == -1) {               \
      init = uxr_millis();          \
    }                               \
    if (uxr_millis() - init > MS) { \
      X;                            \
      init = uxr_millis();          \
    }                               \
  }
// Invalidate wait set
#define INVALIDATE_WAIT_SET()                          \
  {                                                    \
    if (rcl_wait_set_is_valid(&executor.wait_set)) {   \
      RCASSERT(rcl_wait_set_fini(&executor.wait_set)); \
    }                                                  \
  }

// Assertion for arm door closed state pin
#ifdef ARM_DOOR_ASSERTIONS
#  define ASSERT_ARM_DOOR ASSERT
#else
#  define ASSERT_ARM_DOOR(...)
#endif

// DEBUG: Log a message to the ROS2 log topic
#ifdef DEBUG_LOGGING
#  define DEBUG LOG
#else
#  define DEBUG(...)
#endif

// Error handle loop
void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Timer callback for publishing the sensor message
void sensor_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Populate sensor message
    sensor_msg.arm_door_state = arm_door_state;
    sensor_msg.arm_door_closed_state = digitalRead(ARM_DOOR_CLOSED_STATE_PIN);
    sensor_msg.fixation_button_state = digitalRead(HAND_FIXATION_STATE_PIN);

    // Update hand fixation timestamps
    if (sensor_msg.fixation_button_state) {
      hand_fixation_last_time_pressed_ms = rmw_uros_epoch_millis();
    } else {
      hand_fixation_last_time_released_ms = rmw_uros_epoch_millis();
    }

    // Update tactile glove states
    for (int i = 0; i < sizeof(GLOVE_STATE_PINS) / sizeof(GLOVE_STATE_PINS[0]);
         i++) {
      sensor_msg.tactile_glove_states[i] = analogRead(GLOVE_STATE_PINS[i]);
    }

    // Update sync pulse states
    sensor_msg.sync_pulse_state = sync_pulse_state;
    sensor_msg.sync_pulse_last_time_on_ms = sync_pulse_last_time_on_ms;
    sensor_msg.sync_pulse_last_time_off_ms = sync_pulse_last_time_off_ms;

    RCASSERT(rcl_publish(&sensor_publisher, &sensor_msg, NULL),
             "Failed to publish sensor message");

    DEBUG("Sensor message published");
  }
}

// Timer callback to stop the sync pulse
void sync_pulse_end_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    digitalWrite(SYNC_PULSE_PIN, LOW);
    sync_pulse_state = false;
    sync_pulse_last_time_off_ms = rmw_uros_epoch_millis();

    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel sync pulse end timer");

    DEBUG("Sync pulse ended after %lld ms",
          sync_pulse_last_time_off_ms - sync_pulse_last_time_on_ms);
  }
}

// Timer callback to start the sync pulse
void sync_pulse_start_timer_callback(rcl_timer_t* timer,
                                     int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    digitalWrite(SYNC_PULSE_PIN, HIGH);
    sync_pulse_state = true;
    sync_pulse_last_time_on_ms = rmw_uros_epoch_millis();

    RCASSERT(rcl_timer_cancel(timer),
             "Failed to cancel sync pulse start timer");

    RCASSERT(rcl_timer_reset(&sync_pulse_end_timer),
             "Failed to reset sync pulse end timer");

    DEBUG("Sync pulse started for %lld ms", SYNC_PULSE_DURATION_MS);
  }
}

// Timer callback to start the sync pulse delay timer
void sync_pulse_base_timer_callback(rcl_timer_t* timer,
                                    int64_t last_call_time) {
  if (timer != NULL) {
    ASSERT(sync_pulse_state == false, "Sync pulse state is true");

    digitalWrite(SYNC_PULSE_PIN, LOW);
    sync_pulse_state = false;

    // Generate a random delay between 0 and SYNC_PULSE_DELAY_RANGE_MS
    int64_t delay_ms = random(SYNC_PULSE_DELAY_RANGE_MS);
    int64_t old_period;

    // Exchange the timer period with the delay and reset the timer
    RCASSERT(rcl_timer_exchange_period(&sync_pulse_start_timer,
                                       RCL_MS_TO_NS(delay_ms), &old_period),
             "Failed to exchange sync pulse start timer period");
    RCASSERT(rcl_timer_reset(&sync_pulse_start_timer),
             "Failed to reset sync pulse start timer");

    DEBUG("Sync pulse start scheduled for %lld ms from now", delay_ms);
  }
}

// Timer callback to stop the reward control
void reward_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    digitalWrite(REWARD_CONTROL_PIN, LOW);
    reward_active = false;

    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel reward timer");

    LOG("Reward timer callback completed");
  }
}

// Service callback for controlling the reward
void set_reward_callback(const void* req, void* res) {
  tabletop_msgs__srv__SetReward_Request* request =
      (tabletop_msgs__srv__SetReward_Request*) req;
  tabletop_msgs__srv__SetReward_Response* response =
      (tabletop_msgs__srv__SetReward_Response*) res;

  LOG("Set reward callback started");

  bool timer_is_canceled;
  RCASSERT(rcl_timer_is_canceled(&reward_timer, &timer_is_canceled),
           "Failed to check if reward timer is canceled");
  if (!timer_is_canceled) {
    ASSERT(reward_active == true, "Reward timer is not canceled but reward is "
                                  "not active");
    response->success = false;
    STRING_SET(&response->message, "Error: Reward already active!");
    LOG(response->message.data);
    return;
  }

  digitalWrite(REWARD_CONTROL_PIN, HIGH);
  reward_active = true;

  uint32_t duration_ms = request->duration_ms;
  int64_t old_period;

  RCASSERT(rcl_timer_exchange_period(&reward_timer, RCL_MS_TO_NS(duration_ms),
                                     &old_period),
           "Failed to exchange reward timer period");
  RCASSERT(rcl_timer_reset(&reward_timer), "Failed to reset reward timer");

  response->success = true;
  STRING_SET(&response->message, "Reward started for %u ms", duration_ms);
  LOG(response->message.data);
}

// Service callback for getting the reward state
void get_reward_callback(const void* req, void* res) {
  tabletop_msgs__srv__GetReward_Request* request =
      (tabletop_msgs__srv__GetReward_Request*) req;
  tabletop_msgs__srv__GetReward_Response* response =
      (tabletop_msgs__srv__GetReward_Response*) res;

  DEBUG("Get reward callback started");

  response->is_active = reward_active;
  response->success = true;
  STRING_SET(&response->message, "Reward is %s",
             response->is_active ? "active" : "inactive");
  LOG(response->message.data);
}

// Timer callback to stop the arm door control
void arm_door_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    LOG("Arm door timer callback started");

    // Stop the arm door motors
    digitalWrite(ARM_DOOR_OPEN_CONTROL_PIN, LOW);
    digitalWrite(ARM_DOOR_CLOSE_CONTROL_PIN, LOW);

    // Check if the arm door is opening or closing
    // TODO: Check this logic once we have the arm door switch working
    switch (arm_door_state) {
    case ARM_DOOR_OPENING:
      ASSERT_ARM_DOOR(digitalRead(ARM_DOOR_CLOSED_STATE_PIN) == LOW,
                      "Arm door closed state pin is not LOW after opening");
      arm_door_state = ARM_DOOR_OPEN;
      break;
    case ARM_DOOR_CLOSING:
      ASSERT_ARM_DOOR(digitalRead(ARM_DOOR_CLOSED_STATE_PIN) == HIGH,
                      "Arm door closed state pin is not HIGH after closing");
      arm_door_state = ARM_DOOR_CLOSED;
      break;
    default:
      ASSERT(false, "Arm door state is not OPENING or CLOSING when timer "
                    "callback is called");
    }

    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel arm door timer");

    LOG("Arm door timer callback finished %s",
        arm_door_state == ARM_DOOR_OPEN ? "opening" : "closing");
  }
}

// Service callback for controlling the arm door
void set_arm_door_callback(const void* req, void* res) {
  tabletop_msgs__srv__SetArmDoor_Request* request =
      (tabletop_msgs__srv__SetArmDoor_Request*) req;
  tabletop_msgs__srv__SetArmDoor_Response* response =
      (tabletop_msgs__srv__SetArmDoor_Response*) res;

  LOG("Set arm door callback started");

  ASSERT_ARM_DOOR((digitalRead(ARM_DOOR_CLOSED_STATE_PIN) == HIGH) ==
                      (arm_door_state == ARM_DOOR_CLOSED),
                  "Arm door state is not consistent with closed state pin");

  bool timer_is_canceled;
  RCASSERT(rcl_timer_is_canceled(&arm_door_timer, &timer_is_canceled),
           "Failed to check if arm door timer is canceled");
  if (!timer_is_canceled) {
    ASSERT(arm_door_state == ARM_DOOR_CLOSING ||
               arm_door_state == ARM_DOOR_OPENING,
           "Arm door state is not CLOSING or OPENING when arm door timer is "
           "still running");
    ASSERT_ARM_DOOR(digitalRead(ARM_DOOR_CLOSED_STATE_PIN) == LOW,
                    "Arm door closed state pin is not LOW when motor is still "
                    "running");
  }


  int64_t time_since_last_call;
  RCASSERT(rcl_timer_get_time_since_last_call(&arm_door_timer,
                                              &time_since_last_call),
           "Failed to get time since last arm door timer call");

  // Duration to set the arm door control pin high for
  int64_t duration_ms;

  if (request->open) {
    switch (arm_door_state) {
    case ARM_DOOR_OPEN:
      response->success = true;
      STRING_SET(&response->message, "Arm door already open");
      LOG(response->message.data);
      return;
    case ARM_DOOR_OPENING:
      response->success = true;
      STRING_SET(&response->message, "Arm door is already opening");
      LOG(response->message.data);
      return;
    case ARM_DOOR_CLOSED:
      LOG("Arm door is closed, opening");
      duration_ms = ARM_DOOR_PERIOD_MS;
      break;
    case ARM_DOOR_CLOSING:
      LOG("Arm door is closing, reversing");
      duration_ms = time_since_last_call;
      break;
    }

    digitalWrite(ARM_DOOR_OPEN_CONTROL_PIN, HIGH);
    digitalWrite(ARM_DOOR_CLOSE_CONTROL_PIN, LOW);
    arm_door_state = ARM_DOOR_OPENING;
  } else {
    switch (arm_door_state) {
    case ARM_DOOR_OPEN:
      LOG("Arm door is open, closing");
      duration_ms = ARM_DOOR_PERIOD_MS;
      break;
    case ARM_DOOR_OPENING:
      LOG("Arm door is opening, reversing");
      duration_ms = time_since_last_call;
      break;
    case ARM_DOOR_CLOSED:
      STRING_SET(&response->message, "Arm door already closed");
      LOG(response->message.data);
      response->success = true;
      return;
    case ARM_DOOR_CLOSING:
      STRING_SET(&response->message, "Arm door is already closing");
      LOG(response->message.data);
      response->success = true;
      return;
    }

    digitalWrite(ARM_DOOR_OPEN_CONTROL_PIN, LOW);
    digitalWrite(ARM_DOOR_CLOSE_CONTROL_PIN, HIGH);
    arm_door_state = ARM_DOOR_CLOSING;
  }

  int64_t old_period;
  RCASSERT(rcl_timer_exchange_period(&arm_door_timer, RCL_MS_TO_NS(duration_ms),
                                     &old_period),
           "Failed to exchange arm door timer period");
  RCASSERT(rcl_timer_reset(&arm_door_timer), "Failed to reset arm door timer");

  response->success = true;
  STRING_SET(&response->message, "Arm door %s started for %ld ms",
             request->open ? "open" : "close", duration_ms);

  LOG(response->message.data);
}

// Service callback for getting the arm door state
void get_arm_door_callback(const void* req, void* res) {
  tabletop_msgs__srv__GetArmDoor_Request* request =
      (tabletop_msgs__srv__GetArmDoor_Request*) req;
  tabletop_msgs__srv__GetArmDoor_Response* response =
      (tabletop_msgs__srv__GetArmDoor_Response*) res;

  ASSERT_ARM_DOOR((digitalRead(ARM_DOOR_CLOSED_STATE_PIN) == HIGH) ==
                      (arm_door_state == ARM_DOOR_CLOSED),
                  "Arm door state is not consistent with closed state pin");

  response->is_closed = digitalRead(ARM_DOOR_CLOSED_STATE_PIN);
  response->state = arm_door_state;
  response->success = true;
  char state_str[10];
  switch (arm_door_state) {
  case ARM_DOOR_OPEN:
    strcpy(state_str, "open");
    break;
  case ARM_DOOR_CLOSED:
    strcpy(state_str, "closed");
    break;
  case ARM_DOOR_OPENING:
    strcpy(state_str, "opening");
    break;
  case ARM_DOOR_CLOSING:
    strcpy(state_str, "closing");
    break;
  }
  STRING_SET(&response->message,
             "Arm door closed state pin is %s and arm door "
             "state is %s",
             response->is_closed ? "HIGH" : "LOW", state_str);
  DEBUG(response->message.data);
}

// Service callback for getting the hand fixation state
void get_hand_fixation_callback(const void* req, void* res) {
  tabletop_msgs__srv__GetHandFixation_Request* request =
      (tabletop_msgs__srv__GetHandFixation_Request*) req;
  tabletop_msgs__srv__GetHandFixation_Response* response =
      (tabletop_msgs__srv__GetHandFixation_Response*) res;

  response->is_pressed = digitalRead(HAND_FIXATION_STATE_PIN);
  response->last_time_pressed_ms = hand_fixation_last_time_pressed_ms;
  response->last_time_released_ms = hand_fixation_last_time_released_ms;
  response->success = true;
  STRING_SET(&response->message, "Hand fixation is %s",
             response->is_pressed ? "pressed" : "released");
  DEBUG(response->message.data);
}

// Service callback for controlling the smartglass
void set_smartglass_callback(const void* req, void* res) {
  tabletop_msgs__srv__SetSmartglass_Request* request =
      (tabletop_msgs__srv__SetSmartglass_Request*) req;
  tabletop_msgs__srv__SetSmartglass_Response* response =
      (tabletop_msgs__srv__SetSmartglass_Response*) res;

  digitalWrite(SMARTGLASS_CONTROL_PIN, request->is_revealed);

  response->success = true;
  STRING_SET(&response->message, "Smartglass %s",
             request->is_revealed ? "revealed" : "occluded");
  LOG(response->message.data);
}

bool create_entities() {
  // create allocator
  allocator = rcl_get_default_allocator();
  printf("Allocator initialized\n");

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  printf("Support initialized\n");

  // create node
  RCCHECK(rclc_node_init_default(&node, "teensy", "", &support));
  printf("Node initialized\n");

  // create publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &sensor_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, msg, TeensySensor),
      SENSORS_TOPIC));
  RCCHECK(rclc_publisher_init_default(
      &log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      LOG_TOPIC));
  // can now log to the ROS2 teensy/log topic
  LOG("Publishers initialized");

  // create services
  RCCHECK(rclc_service_init_default(
      &set_arm_door_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, SetArmDoor),
      SET_ARM_DOOR_SRV_NAME));
  RCCHECK(rclc_service_init_default(
      &get_arm_door_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, GetArmDoor),
      GET_ARM_DOOR_SRV_NAME));
  RCCHECK(rclc_service_init_default(
      &set_smartglass_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, SetSmartglass),
      SET_SMARTGLASS_SRV_NAME));
  RCCHECK(rclc_service_init_default(
      &set_reward_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, SetReward),
      SET_REWARD_SRV_NAME));
  RCCHECK(rclc_service_init_default(
      &get_reward_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, GetReward),
      GET_REWARD_SRV_NAME));
  RCCHECK(rclc_service_init_default(
      &get_hand_fixation_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, GetHandFixation),
      GET_HAND_FIXATION_SRV_NAME));
  LOG("Services initialized");

  // create timers
  RCCHECK(rclc_timer_init_default2(&sync_pulse_base_timer, &support,
                                   RCL_MS_TO_NS(SYNC_PULSE_BASE_PERIOD_MS),
                                   sync_pulse_base_timer_callback, true));
  RCCHECK(rclc_timer_init_default2(&sync_pulse_start_timer, &support,
                                   RCL_MS_TO_NS(SYNC_PULSE_DELAY_RANGE_MS),
                                   sync_pulse_start_timer_callback, false));
  RCCHECK(rclc_timer_init_default2(&sync_pulse_end_timer, &support,
                                   RCL_MS_TO_NS(SYNC_PULSE_DURATION_MS),
                                   sync_pulse_end_timer_callback, false));
  RCCHECK(rclc_timer_init_default2(&sensor_timer, &support,
                                   RCL_MS_TO_NS(SENSOR_PERIOD_MS),
                                   sensor_timer_callback, true));
  RCCHECK(rclc_timer_init_default2(&reward_timer, &support, RCL_MS_TO_NS(1000),
                                   reward_timer_callback, false));
  RCCHECK(rclc_timer_init_default2(&arm_door_timer, &support,
                                   RCL_MS_TO_NS(ARM_DOOR_PERIOD_MS),
                                   arm_door_timer_callback, false));
  LOG("Timers initialized");

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 12, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_base_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_start_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_end_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &reward_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &arm_door_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));

  RCCHECK(rclc_executor_add_service(
      &executor, &set_arm_door_service, &set_arm_door_request,
      &set_arm_door_response, &set_arm_door_callback));
  RCCHECK(rclc_executor_add_service(
      &executor, &get_arm_door_service, &get_arm_door_request,
      &get_arm_door_response, &get_arm_door_callback));
  RCCHECK(rclc_executor_add_service(
      &executor, &set_smartglass_service, &set_smartglass_request,
      &set_smartglass_response, &set_smartglass_callback));
  RCCHECK(rclc_executor_add_service(&executor, &set_reward_service,
                                    &set_reward_request, &set_reward_response,
                                    &set_reward_callback));
  RCCHECK(rclc_executor_add_service(&executor, &get_reward_service,
                                    &get_reward_request, &get_reward_response,
                                    &get_reward_callback));
  RCCHECK(rclc_executor_add_service(
      &executor, &get_hand_fixation_service, &get_hand_fixation_request,
      &get_hand_fixation_response, &get_hand_fixation_callback));

  LOG("Executor initialized");

  RCCHECK(rmw_uros_sync_session(1000));

  LOG("Session synced");

  return true;
}

void destroy_entities() {
  // Set context entity destroy session timeout
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  RCSOFTCHECK(
      rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  // Destroy entities
  RCSOFTCHECK(rcl_publisher_fini(&sensor_publisher, &node));
  RCSOFTCHECK(rcl_publisher_fini(&log_publisher, &node));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_base_timer));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_start_timer));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_end_timer));
  RCSOFTCHECK(rcl_timer_fini(&sensor_timer));
  RCSOFTCHECK(rcl_timer_fini(&reward_timer));
  RCSOFTCHECK(rcl_timer_fini(&arm_door_timer));
  RCSOFTCHECK(rcl_service_fini(&set_arm_door_service, &node));
  RCSOFTCHECK(rcl_service_fini(&get_arm_door_service, &node));
  RCSOFTCHECK(rcl_service_fini(&set_smartglass_service, &node));
  RCSOFTCHECK(rcl_service_fini(&set_reward_service, &node));
  RCSOFTCHECK(rcl_service_fini(&get_reward_service, &node));
  RCSOFTCHECK(rcl_service_fini(&get_hand_fixation_service, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  printf("Entities destroyed\n");
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  printf("Serial transport initialized\n");

  // Initialize output pins
  pinMode(ARM_DOOR_OPEN_CONTROL_PIN, OUTPUT);
  pinMode(ARM_DOOR_CLOSE_CONTROL_PIN, OUTPUT);
  pinMode(SMARTGLASS_CONTROL_PIN, OUTPUT);
  pinMode(REWARD_CONTROL_PIN, OUTPUT);
  pinMode(SYNC_PULSE_PIN, OUTPUT);
  digitalWrite(SYNC_PULSE_PIN, LOW);

  // Initialize input pins
  pinMode(ARM_DOOR_CLOSED_STATE_PIN, INPUT);
  pinMode(HAND_FIXATION_STATE_PIN, INPUT);

  printf("Pins initialized\n");

  // Initialize state variables
  state = WAITING_AGENT;
  agent_reconnect_retries = 0;

  sync_pulse_state = false;
  sync_pulse_last_time_on_ms = -1;
  sync_pulse_last_time_off_ms = -1;

  hand_fixation_last_time_pressed_ms = -1;
  hand_fixation_last_time_released_ms = -1;

  reward_active = false;

  if (digitalRead(ARM_DOOR_CLOSED_STATE_PIN) == HIGH) {
    arm_door_state = ARM_DOOR_CLOSED;
  } else {
    arm_door_state = ARM_DOOR_OPEN;
  }

  // create message memories
  bool success = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, srv, SetArmDoor_Response),
      &set_arm_door_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, srv, GetArmDoor_Response),
      &get_arm_door_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, srv, SetSmartglass_Response),
      &set_smartglass_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, srv, SetReward_Response),
      &set_reward_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, srv, GetReward_Response),
      &get_reward_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, srv, GetHandFixation_Response),
      &get_hand_fixation_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), &log_msg,
      memory_conf);
  if (!success) {
    printf("Failed to create message memories\n");
    error_loop();
  }

  delay(2000);
}

void loop() {
  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(
        AGENT_RECONNECT_PERIOD_MS,
        state =
            (RMW_RET_OK == rmw_uros_ping_agent(AGENT_RECONNECT_TIMEOUT_MS, 1))
                ? AGENT_AVAILABLE
                : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    static bool success = true;
    EXECUTE_EVERY_N_MS(
        AGENT_CHECK_CONNECTED_PERIOD_MS,
        success = (RMW_RET_OK ==
                   rmw_uros_ping_agent(AGENT_CHECK_CONNECTED_TIMEOUT_MS, 1)););
    agent_reconnect_retries = success ? 0 : agent_reconnect_retries + 1;
    if (agent_reconnect_retries < AGENT_RECONNECT_MAX_RETRIES) {
      rclc_executor_spin_some(&executor,
                              RCL_MS_TO_NS(EXECUTOR_SPIN_TIMEOUT_MS));
    } else {
      state = AGENT_DISCONNECTED;
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    printf("Unknown state\n");
    error_loop();
    break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
