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
#include <tabletop_msgs/srv/set_uint32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#  error This code only supports serial transport.
#endif

// #define DEBUG_LOGGING
// Macro definitions

// Check return code from ROS2 function, print error string and return NULL if
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
#define RCASSERT(fn, fmt, ...)           \
  {                                      \
    rcl_ret_t temp_rc = fn;              \
    if ((temp_rc != RCL_RET_OK)) {       \
      LOG("RC Assertion failed!");       \
      LOG("Error number: %ld", temp_rc); \
      LOG(fmt, ##__VA_ARGS__);           \
    }                                    \
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

// DEBUG: Log a message to the ROS2 log topic
#ifdef DEBUG_LOGGING
#  define DEBUG LOG
#else
#  define DEBUG(...)
#endif

// Define pin mappings

#define ARM_DOOR_CONTROL_PIN 1
#define SMARTGLASS_CONTROL_PIN 2
#define REWARD_CONTROL_PIN 3
#define ARM_DOOR_STATE_PIN 4
#define SMARTGLASS_STATE_PIN 5
#define REWARD_STATE_PIN 6
#define HAND_FIXATION_STATE_PIN 7
#define SYNC_PULSE_PIN 9
static const uint8_t GLOVE_STATE_PINS[] = {A0, A1, A2, A3, A4};

// Message memory configuration

#define MAX_STRING_CAPACITY 50
#define MAX_ROS2_TYPE_SEQUENCE_CAPACITY 5
#define MAX_BASIC_TYPE_SEQUENCE_CAPACITY 5
static const micro_ros_utilities_memory_conf_t memory_conf = {
    MAX_STRING_CAPACITY,
    MAX_ROS2_TYPE_SEQUENCE_CAPACITY,
    MAX_BASIC_TYPE_SEQUENCE_CAPACITY,
    NULL,
    0,
    NULL};

// Execution parameters

#define EXECUTOR_SPIN_TIMEOUT_MS 20    // Timeout for executor spin, in ms
#define AGENT_RECONNECT_PERIOD_MS 1000 // Period for agent reconnection, in ms
#define AGENT_RECONNECT_TIMEOUT_MS 5   // Timeout for agent reconnection, in ms
#define SENSOR_PERIOD_MS 10            // Sensor update period, in ms
#define SYNC_PULSE_BASE_PERIOD_MS 1000 // Base period between sync pulses, in ms
#define SYNC_PULSE_DELAY_RANGE_MS \
  200                              // Range of jitter in the base period, in ms
#define SYNC_PULSE_DURATION_MS 100 // Duration of each sync pulse, in ms

// Agent reconnection parameters

#define AGENT_RECONNECT_MAX_RETRIES \
  10 // Maximum number of retries for agent reconnect before giving up

// Global variables

rcl_publisher_t sensor_publisher;
rcl_publisher_t log_publisher;

rcl_service_t arm_door_service;
rcl_service_t smartglass_service;
rcl_service_t reward_service;

rcl_timer_t sync_pulse_base_timer;
rcl_timer_t sync_pulse_start_timer;
rcl_timer_t sync_pulse_end_timer;
rcl_timer_t sensor_timer;
rcl_timer_t reward_timer;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

tabletop_msgs__msg__TeensySensor sensor_msg;
std_msgs__msg__String log_msg;

std_srvs__srv__SetBool_Request arm_door_request;
std_srvs__srv__SetBool_Response arm_door_response;
std_srvs__srv__SetBool_Request smartglass_request;
std_srvs__srv__SetBool_Response smartglass_response;
tabletop_msgs__srv__SetUint32_Request reward_request;
tabletop_msgs__srv__SetUint32_Response reward_response;

static uint8_t agent_reconnect_retries;

bool sync_pulse_state;
static enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  CLIENT_ERROR
} state;

// Error handle loop
void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void sync_pulse_end_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    DEBUG("Sync pulse end");

    digitalWrite(SYNC_PULSE_PIN, LOW);
    sync_pulse_state = false;

    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel sync pulse end timer");
  }
}

void sync_pulse_start_timer_callback(rcl_timer_t* timer,
                                     int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    DEBUG("Sync pulse start");

    RCASSERT(rcl_timer_reset(&sync_pulse_end_timer),
             "Failed to reset sync pulse end timer");
    digitalWrite(SYNC_PULSE_PIN, HIGH);
    sync_pulse_state = true;

    RCASSERT(rcl_timer_cancel(timer),
             "Failed to cancel sync pulse start timer");

    DEBUG("Sync pulse started for %lld ms", SYNC_PULSE_DURATION_MS);
  }
}

void sync_pulse_base_timer_callback(rcl_timer_t* timer,
                                    int64_t last_call_time) {
  if (timer != NULL) {
    DEBUG("Sync pulse base timer callback");

    ASSERT(sync_pulse_state == false, "Sync pulse state is true");

    // Generate a random delay between 0 and SYNC_PULSE_DELAY_RANGE_MS
    int64_t delay_ms = random(SYNC_PULSE_DELAY_RANGE_MS);
    int64_t old_period;

    // Exchange the timer period with the delay and reset the timer
    RCASSERT(rcl_timer_exchange_period(&sync_pulse_start_timer,
                                       RCL_MS_TO_NS(delay_ms), &old_period),
             "Failed to exchange sync pulse start timer period");
    RCASSERT(rcl_timer_reset(&sync_pulse_start_timer),
             "Failed to reset sync pulse start timer");
    digitalWrite(SYNC_PULSE_PIN, LOW);
    sync_pulse_state = false;

    DEBUG("delay: %lld ms", delay_ms);
  }
}

void sensor_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    DEBUG("Sensor timer callback");
    // Populate sensor message
    sensor_msg.arm_door_right_state = digitalRead(ARM_DOOR_STATE_PIN);
    sensor_msg.arm_door_left_state = digitalRead(ARM_DOOR_STATE_PIN);
    sensor_msg.smart_glass_laser_state = digitalRead(SMARTGLASS_STATE_PIN);
    sensor_msg.fixation_button_state = digitalRead(HAND_FIXATION_STATE_PIN);
    for (int i = 0; i < 5; i++) {
      sensor_msg.tactile_glove_states[i] = analogRead(GLOVE_STATE_PINS[i]);
    }
    sensor_msg.sync_pulse_state = sync_pulse_state;

    RCASSERT(rcl_publish(&sensor_publisher, &sensor_msg, NULL),
             "Failed to publish sensor message");
  }
}

void reward_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    LOG("Reward timer callback started");

    digitalWrite(REWARD_CONTROL_PIN, LOW);
    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel reward timer");

    LOG("Reward timer callback completed");
  }
}

// Service callback for reward
void reward_callback(const void* req, void* res) {
  tabletop_msgs__srv__SetUint32_Request* request =
      (tabletop_msgs__srv__SetUint32_Request*) req;
  tabletop_msgs__srv__SetUint32_Response* response =
      (tabletop_msgs__srv__SetUint32_Response*) res;

  LOG("Reward service callback started");

  uint32_t duration_ms = request->data;
  int64_t old_period;

  RCASSERT(rcl_timer_exchange_period(&reward_timer, RCL_MS_TO_NS(duration_ms),
                                     &old_period),
           "Failed to exchange reward timer period");
  RCASSERT(rcl_timer_reset(&reward_timer), "Failed to reset reward timer");
  digitalWrite(REWARD_CONTROL_PIN, HIGH);

  STRING_SET(&response->message, "Reward started for %lu ms", duration_ms);
  response->success = true;

  LOG("Reward service callback completed");
}

// Service callback for arm_door
void arm_door_callback(const void* req, void* res) {
  std_srvs__srv__SetBool_Request* request =
      (std_srvs__srv__SetBool_Request*) req;
  std_srvs__srv__SetBool_Response* response =
      (std_srvs__srv__SetBool_Response*) res;

  LOG("Arm door callback started");

  digitalWrite(ARM_DOOR_CONTROL_PIN, request->data);
  if (request->data) {
    STRING_SET(&response->message, "Arm door opened");
    LOG("Arm door opened");
  } else {
    STRING_SET(&response->message, "Arm door closed");
    LOG("Arm door closed");
  }
  response->success = true;
}

// Service callback for smartglass
void smartglass_callback(const void* req, void* res) {
  std_srvs__srv__SetBool_Request* request =
      (std_srvs__srv__SetBool_Request*) req;
  std_srvs__srv__SetBool_Response* response =
      (std_srvs__srv__SetBool_Response*) res;

  LOG("Smartglass callback started");

  digitalWrite(SMARTGLASS_CONTROL_PIN, request->data);
  if (request->data) {
    STRING_SET(&response->message, "Smartglass revealed");
    LOG("Smartglass revealed");
  } else {
    STRING_SET(&response->message, "Smartglass occluded");
    LOG("Smartglass occluded");
  }
  response->success = true;
  LOG("Smartglass callback completed");
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
      "teensy/sensors"));
  RCCHECK(rclc_publisher_init_default(
      &log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "teensy/log"));
  // can now log to the ROS2 teensy/log topic
  LOG("Publishers initialized");

  // create services
  RCCHECK(rclc_service_init_default(
      &arm_door_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "teensy/arm_door"));
  RCCHECK(rclc_service_init_default(
      &smartglass_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
      "teensy/smartglass"));
  RCCHECK(rclc_service_init_default(
      &reward_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, SetUint32),
      "teensy/reward"));
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
  LOG("Timers initialized");

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_base_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_start_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_end_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &reward_timer));
  RCCHECK(rclc_executor_add_service(&executor, &reward_service, &reward_request,
                                    &reward_response, &reward_callback));
  RCCHECK(rclc_executor_add_service(&executor, &arm_door_service,
                                    &arm_door_request, &arm_door_response,
                                    &arm_door_callback));
  RCCHECK(rclc_executor_add_service(&executor, &smartglass_service,
                                    &smartglass_request, &smartglass_response,
                                    &smartglass_callback));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
  LOG("Executor initialized");

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
  RCSOFTCHECK(rcl_service_fini(&arm_door_service, &node));
  RCSOFTCHECK(rcl_service_fini(&smartglass_service, &node));
  RCSOFTCHECK(rcl_service_fini(&reward_service, &node));
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
  pinMode(ARM_DOOR_CONTROL_PIN, OUTPUT);
  pinMode(SMARTGLASS_CONTROL_PIN, OUTPUT);
  pinMode(REWARD_CONTROL_PIN, OUTPUT);
  pinMode(SYNC_PULSE_PIN, OUTPUT);
  digitalWrite(SYNC_PULSE_PIN, LOW);

  // Initialize input pins
  pinMode(ARM_DOOR_STATE_PIN, INPUT);
  pinMode(SMARTGLASS_STATE_PIN, INPUT);
  pinMode(REWARD_STATE_PIN, INPUT);
  pinMode(HAND_FIXATION_STATE_PIN, INPUT);

  printf("Pins initialized\n");

  // Initialize global variables
  agent_reconnect_retries = 0;
  sync_pulse_state = false;
  state = WAITING_AGENT;

  // create message memories
  bool success = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_srvs, srv, SetBool_Response),
      &arm_door_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_srvs, srv, SetBool_Response),
      &smartglass_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, srv, SetUint32_Response),
      &reward_response, memory_conf);
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
        200, state = (RMW_RET_OK ==
                      rmw_uros_ping_agent(AGENT_RECONNECT_TIMEOUT_MS, 1))
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
        AGENT_RECONNECT_PERIOD_MS,
        success = (RMW_RET_OK ==
                   rmw_uros_ping_agent(AGENT_RECONNECT_TIMEOUT_MS, 1)););
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
