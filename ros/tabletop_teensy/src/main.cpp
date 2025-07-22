#include <Arduino.h>
#include <builtin_interfaces/msg/duration.h>
#include <builtin_interfaces/msg/time.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <tabletop_interfaces/msg/teensy_sensor.h>
#include <tabletop_interfaces/srv/set_arm_lock.h>
#include <tabletop_interfaces/srv/set_reward.h>
#include <tabletop_interfaces/srv/set_smartglass.h>

#include "core_pins.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#  error This code only supports serial transport.
#endif

// #define DEBUG_LOGGING

// Define pin mappings
#define LEFT_ARM_LOCK_CONTROL_PIN 4
#define RIGHT_ARM_LOCK_CONTROL_PIN 5
#define SMARTGLASS_CONTROL_PIN 3
#define REWARD_CONTROL_PIN 1
#define SYNC_PULSE_CONTROL_PIN 9
#define LEFT_ARM_LOCKED_STATE_PIN 34
#define RIGHT_ARM_LOCKED_STATE_PIN 35
#define SAFETY_LASER_UNBROKEN_STATE_PIN 36
static const uint8_t LEFT_GLOVE_STATE_PINS[] = {A0, A1, A2, A3, A4};
static const uint8_t RIGHT_GLOVE_STATE_PINS[] = {A5, A6, A7, A8, A9};

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
#define SENSOR_TOPIC "/teensy/sensor"
#define LOG_TOPIC "/teensy/log"

// ROS2 services
#define SET_ARM_LOCK_SRV_NAME "/teensy/set_arm_lock"
#define SET_SMARTGLASS_SRV_NAME "/teensy/set_smartglass"
#define SET_REWARD_SRV_NAME "/teensy/set_reward"

// Execution parameters
#define AGENT_RECONNECT_PERIOD_MS 100
#define AGENT_RECONNECT_TIMEOUT_MS 50
#define EXECUTOR_SPIN_TIMEOUT_MS 20
#define AGENT_CHECK_CONNECTED_PERIOD_MS 5000
#define AGENT_CHECK_CONNECTED_TIMEOUT_MS 10
#define SENSOR_PERIOD_MS 10
#define SYNC_PULSE_BASE_PERIOD_MS 1000
#define SYNC_PULSE_DELAY_RANGE_MS 200
#define SYNC_PULSE_DURATION_MS 100

// Agent reconnection parameters
#define AGENT_RECONNECT_MAX_RETRIES 10

// Global variables
rcl_publisher_t sensor_publisher;
rcl_publisher_t log_publisher;

rcl_service_t set_arm_lock_service;
rcl_service_t set_smartglass_service;
rcl_service_t set_reward_service;

rcl_timer_t sync_pulse_base_timer;
rcl_timer_t sync_pulse_start_timer;
rcl_timer_t sync_pulse_end_timer;
rcl_timer_t sensor_timer;
rcl_timer_t reward_timer;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

tabletop_interfaces__msg__TeensySensor sensor_msg;
std_msgs__msg__String log_msg;

tabletop_interfaces__srv__SetArmLock_Request set_arm_lock_request;
tabletop_interfaces__srv__SetArmLock_Response set_arm_lock_response;
tabletop_interfaces__srv__SetSmartglass_Request set_smartglass_request;
tabletop_interfaces__srv__SetSmartglass_Response set_smartglass_response;
tabletop_interfaces__srv__SetReward_Request set_reward_request;
tabletop_interfaces__srv__SetReward_Response set_reward_response;

// State tracking
static uint8_t agent_reconnect_retries;

static enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  CLIENT_ERROR
} state;

bool sync_pulse_state;
builtin_interfaces__msg__Time sync_pulse_last_time_on;
builtin_interfaces__msg__Time sync_pulse_last_time_off;
bool reward_active;
bool smartglass_revealed;

// Macro definitions
#define RCCHECK(fn)                                      \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK)) {                       \
      printf("Error: %s\n", rcl_get_error_string().str); \
      return false;                                      \
    }                                                    \
  }
#define RCSOFTCHECK(fn)                                  \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK)) {                       \
      printf("Error: %s\n", rcl_get_error_string().str); \
    }                                                    \
  }
#define STRING_SET(str_ptr, fmt, ...)                                   \
  {                                                                     \
    snprintf((str_ptr)->data, (str_ptr)->capacity, fmt, ##__VA_ARGS__); \
    (str_ptr)->size = strlen((str_ptr)->data);                          \
  }
#define LOG(fmt, ...)                                         \
  {                                                           \
    STRING_SET(&log_msg.data, fmt, ##__VA_ARGS__);            \
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL)); \
  }
#define RCASSERT(fn, fmt, ...)          \
  {                                     \
    rcl_ret_t temp_rc = fn;             \
    if ((temp_rc != RCL_RET_OK)) {      \
      LOG("RC Assertion failed!");      \
      LOG("Error number: %d", temp_rc); \
      LOG(fmt, ##__VA_ARGS__);          \
    }                                   \
  }
#define ASSERT(fn, fmt, ...)    \
  {                             \
    if (!(fn)) {                \
      LOG("Assertion failed!"); \
      LOG(fmt, ##__VA_ARGS__);  \
    }                           \
  }
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

// DEBUG: Log a message to the ROS2 log topic
#ifdef DEBUG_LOGGING
#  define DEBUG LOG
#else
#  define DEBUG(...)
#endif

#define RCL_S_TO_MS(sec) (sec * 1000LL)
#define ROS_TIME_TO_MS(time_msg) \
  (RCL_S_TO_MS(time_msg.sec) + RCL_NS_TO_MS(time_msg.nanosec))
#define ROS_TIME_TO_NS(time_msg) (RCL_S_TO_NS(time_msg.sec) + time_msg.nanosec)
#define GET_CURRENT_ROS_TIME(time_msg)                      \
  {                                                         \
    int64_t now_ns = rmw_uros_epoch_nanos();                \
    time_msg.sec = now_ns / (1000LL * 1000LL * 1000LL);     \
    time_msg.nanosec = now_ns % (1000LL * 1000LL * 1000LL); \
  }

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
    GET_CURRENT_ROS_TIME(sensor_msg.timestamp);
    sensor_msg.is_safety_laser_broken =
        !digitalRead(SAFETY_LASER_UNBROKEN_STATE_PIN);
    sensor_msg.is_left_arm_locked = digitalRead(LEFT_ARM_LOCKED_STATE_PIN);
    sensor_msg.is_right_arm_locked = digitalRead(RIGHT_ARM_LOCKED_STATE_PIN);
    sensor_msg.is_reward_active = reward_active;
    sensor_msg.is_smartglass_revealed = smartglass_revealed;

    // Update tactile glove states
    for (size_t i = 0; i < 5; i++) {
      sensor_msg.left_tactile_glove_states[i] =
          analogRead(LEFT_GLOVE_STATE_PINS[i]);
    }
    for (size_t i = 0; i < 5; i++) {
      sensor_msg.right_tactile_glove_states[i] =
          analogRead(RIGHT_GLOVE_STATE_PINS[i]);
    }

    // Update sync pulse states
    sensor_msg.sync_pulse_state = sync_pulse_state;
    sensor_msg.sync_pulse_last_time_on = sync_pulse_last_time_on;
    sensor_msg.sync_pulse_last_time_off = sync_pulse_last_time_off;

    RCASSERT(rcl_publish(&sensor_publisher, &sensor_msg, NULL),
             "Failed to publish sensor message");
    DEBUG("Sensor message published");
  }
}

// Timer callback to stop the sync pulse
void sync_pulse_end_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    digitalWrite(SYNC_PULSE_CONTROL_PIN, LOW);
    sync_pulse_state = false;
    GET_CURRENT_ROS_TIME(sync_pulse_last_time_off);

    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel sync pulse end timer");

    DEBUG("Sync pulse ended after %lld ms",
          ROS_TIME_TO_MS(sync_pulse_last_time_off) -
              ROS_TIME_TO_MS(sync_pulse_last_time_on));
  }
}

// Timer callback to start the sync pulse
void sync_pulse_start_timer_callback(rcl_timer_t* timer,
                                     int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    digitalWrite(SYNC_PULSE_CONTROL_PIN, HIGH);
    sync_pulse_state = true;
    GET_CURRENT_ROS_TIME(sync_pulse_last_time_on);

    RCASSERT(rcl_timer_cancel(timer),
             "Failed to cancel sync pulse start timer");
    RCASSERT(rcl_timer_reset(&sync_pulse_end_timer),
             "Failed to reset sync pulse end timer");

    DEBUG("Sync pulse started for %d ms", SYNC_PULSE_DURATION_MS);
  }
}

// Timer callback to start the sync pulse delay timer
void sync_pulse_base_timer_callback(rcl_timer_t* timer,
                                    int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    ASSERT(sync_pulse_state == false, "Sync pulse state is true");

    digitalWrite(SYNC_PULSE_CONTROL_PIN, LOW);
    sync_pulse_state = false;

    int64_t delay_ms = random(SYNC_PULSE_DELAY_RANGE_MS);
    int64_t old_period;
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
    LOG("Reward finished");
  }
}

// Service callback for controlling the reward
void set_reward_callback(const void* req, void* res) {
  const tabletop_interfaces__srv__SetReward_Request* request =
      static_cast<const tabletop_interfaces__srv__SetReward_Request*>(req);
  tabletop_interfaces__srv__SetReward_Response* response =
      static_cast<tabletop_interfaces__srv__SetReward_Response*>(res);

  bool timer_is_canceled;
  RCASSERT(rcl_timer_is_canceled(&reward_timer, &timer_is_canceled),
           "Failed to check if reward timer is canceled");
  if (!timer_is_canceled) {
    ASSERT(reward_active == true,
           "Reward timer is not canceled but reward is not active");
    response->success = false;
    STRING_SET(&response->message, "Error: Reward already active!");
    LOG("%s", response->message.data);
    return;
  }

  digitalWrite(REWARD_CONTROL_PIN, HIGH);
  reward_active = true;

  int64_t duration_ns = ROS_TIME_TO_NS(request->duration);
  int64_t old_period;
  RCASSERT(rcl_timer_exchange_period(&reward_timer, duration_ns, &old_period),
           "Failed to exchange reward timer period");
  RCASSERT(rcl_timer_reset(&reward_timer), "Failed to reset reward timer");

  response->success = true;
  double duration_s = duration_ns / 1e9;
  STRING_SET(&response->message, "Reward started for %.2f s", duration_s);
  LOG("%s", response->message.data);
}

// Service callback for controlling the arm lock
void set_arm_lock_callback(const void* req, void* res) {
  const tabletop_interfaces__srv__SetArmLock_Request* request =
      static_cast<const tabletop_interfaces__srv__SetArmLock_Request*>(req);
  tabletop_interfaces__srv__SetArmLock_Response* response =
      static_cast<tabletop_interfaces__srv__SetArmLock_Response*>(res);

  if (!request->left_arm && !request->right_arm) {
    response->success = false;
    STRING_SET(&response->message, "No arm specified");
    LOG("%s", response->message.data);
    return;
  }

  uint8_t pin_state = request->lock ? HIGH : LOW;
  char message_arm[20] = "";

  if (request->left_arm) {
    digitalWrite(LEFT_ARM_LOCK_CONTROL_PIN, pin_state);
    if (!request->right_arm) {
      strcpy(message_arm, "Left arm");
    }
  }
  if (request->right_arm) {
    digitalWrite(RIGHT_ARM_LOCK_CONTROL_PIN, pin_state);
    if (!request->left_arm) {
      strcpy(message_arm, "Right arm");
    }
  }

  if (request->left_arm && request->right_arm) {
    strcpy(message_arm, "Both arms");
  }

  response->success = true;
  STRING_SET(&response->message, "%s %s", message_arm,
             request->lock ? "locked" : "released");
  LOG("%s", response->message.data);
}

// Service callback for controlling the smartglass
void set_smartglass_callback(const void* req, void* res) {
  const tabletop_interfaces__srv__SetSmartglass_Request* request =
      static_cast<const tabletop_interfaces__srv__SetSmartglass_Request*>(req);
  tabletop_interfaces__srv__SetSmartglass_Response* response =
      static_cast<tabletop_interfaces__srv__SetSmartglass_Response*>(res);

  digitalWrite(SMARTGLASS_CONTROL_PIN, request->reveal ? HIGH : LOW);
  smartglass_revealed = request->reveal;

  response->success = true;
  STRING_SET(&response->message, "Smartglass %s",
             request->reveal ? "revealed" : "occluded");
  LOG("%s", response->message.data);
}

void reset_pins() {
  digitalWrite(LEFT_ARM_LOCK_CONTROL_PIN, HIGH);
  digitalWrite(RIGHT_ARM_LOCK_CONTROL_PIN, HIGH);
  digitalWrite(SMARTGLASS_CONTROL_PIN, LOW);
  digitalWrite(REWARD_CONTROL_PIN, LOW);
  digitalWrite(SYNC_PULSE_CONTROL_PIN, LOW);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &sensor_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, msg, TeensySensor),
      SENSOR_TOPIC));
  RCCHECK(rclc_publisher_init_default(
      &log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      LOG_TOPIC));
  LOG("Publishers initialized");

  // Services
  RCCHECK(rclc_service_init_default(
      &set_arm_lock_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, SetArmLock),
      SET_ARM_LOCK_SRV_NAME));
  RCCHECK(rclc_service_init_default(
      &set_smartglass_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, SetSmartglass),
      SET_SMARTGLASS_SRV_NAME));
  RCCHECK(rclc_service_init_default(
      &set_reward_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, SetReward),
      SET_REWARD_SRV_NAME));
  LOG("Services initialized");

  // Timers
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
  LOG("Timers initialized");

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_base_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_start_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_end_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &reward_timer));
  RCCHECK(rclc_executor_add_service(
      &executor, &set_arm_lock_service, &set_arm_lock_request,
      &set_arm_lock_response, set_arm_lock_callback));
  RCCHECK(rclc_executor_add_service(
      &executor, &set_smartglass_service, &set_smartglass_request,
      &set_smartglass_response, set_smartglass_callback));
  RCCHECK(rclc_executor_add_service(&executor, &set_reward_service,
                                    &set_reward_request, &set_reward_response,
                                    set_reward_callback));
  LOG("Executor initialized");

  RCCHECK(rmw_uros_sync_session(1000));
  LOG("Session synced");

  delay(1000);

  return true;
}

void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  RCSOFTCHECK(
      rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  RCSOFTCHECK(rcl_publisher_fini(&sensor_publisher, &node));
  RCSOFTCHECK(rcl_publisher_fini(&log_publisher, &node));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_base_timer));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_start_timer));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_end_timer));
  RCSOFTCHECK(rcl_timer_fini(&sensor_timer));
  RCSOFTCHECK(rcl_timer_fini(&reward_timer));
  RCSOFTCHECK(rcl_service_fini(&set_arm_lock_service, &node));
  RCSOFTCHECK(rcl_service_fini(&set_smartglass_service, &node));
  RCSOFTCHECK(rcl_service_fini(&set_reward_service, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  printf("Entities destroyed\n");
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  printf("Serial transport initialized\n");

  // Initialize output pins
  pinMode(LEFT_ARM_LOCK_CONTROL_PIN, OUTPUT);
  pinMode(RIGHT_ARM_LOCK_CONTROL_PIN, OUTPUT);
  pinMode(SMARTGLASS_CONTROL_PIN, OUTPUT);
  pinMode(REWARD_CONTROL_PIN, OUTPUT);
  pinMode(SYNC_PULSE_CONTROL_PIN, OUTPUT);
  reset_pins();

  // Initialize input pins
  pinMode(LEFT_ARM_LOCKED_STATE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ARM_LOCKED_STATE_PIN, INPUT_PULLUP);
  pinMode(SAFETY_LASER_UNBROKEN_STATE_PIN, INPUT_PULLUP);
  for (size_t i = 0; i < 5; i++) {
    pinMode(LEFT_GLOVE_STATE_PINS[i], INPUT);
  }
  for (size_t i = 0; i < 5; i++) {
    pinMode(RIGHT_GLOVE_STATE_PINS[i], INPUT);
  }
  printf("Pins initialized\n");

  // Initialize state variables
  state = WAITING_AGENT;
  agent_reconnect_retries = 0;
  sync_pulse_state = false;
  reward_active = false;
  smartglass_revealed = false;
  sync_pulse_last_time_on.sec = 0;
  sync_pulse_last_time_on.nanosec = 0;
  sync_pulse_last_time_off.sec = 0;
  sync_pulse_last_time_off.nanosec = 0;

  // create message memories
  bool success = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, srv,
                                  SetArmLock_Response),
      &set_arm_lock_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, srv,
                                  SetSmartglass_Response),
      &set_smartglass_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, srv, SetReward_Response),
      &set_reward_response, memory_conf);
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
    reset_pins();
    state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    } else {
      agent_reconnect_retries = 0;
    }
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
    reset_pins();
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

// TODO:
// - Add a timer to check if the agent is connected
// - Add a timer to check if the agent is connected
