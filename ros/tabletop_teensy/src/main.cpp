#include <stdarg.h>

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/set_bool.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <tabletop_msgs/msg/teensy_sensor.h>
#include <tabletop_msgs/srv/set_float.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This code only supports serial transport.
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
static const micro_ros_utilities_memory_conf_t memory_conf = {MAX_STRING_CAPACITY, MAX_ROS2_TYPE_SEQUENCE_CAPACITY, MAX_BASIC_TYPE_SEQUENCE_CAPACITY, NULL, 0, NULL};

// Execution parameters

#define EXECUTOR_SPIN_TIMEOUT_MS 20    // Timeout for executor spin, in ms
#define AGENT_RECONNECT_PERIOD_MS 1000 // Period for agent reconnection, in ms
#define AGENT_RECONNECT_TIMEOUT_MS 5   // Timeout for agent reconnection, in ms
#define SENSOR_PERIOD_MS 10            // Sensor update period, in ms
#define SYNC_PULSE_BASE_PERIOD_MS 1000 // Base period between sync pulses, in ms
#define SYNC_PULSE_DELAY_RANGE_MS 200  // Range of jitter in the base period, in ms
#define SYNC_PULSE_DURATION_MS 100     // Duration of each sync pulse, in ms

// Agent reconnection parameters

#define AGENT_RECONNECT_MAX_RETRIES 10 // Maximum number of retries for agent reconnect before giving up

// Global variables

rcl_publisher_t sensor_publisher;
rcl_publisher_t log_publisher;

rcl_service_t arm_door_service;
rcl_service_t smartglass_service;
rcl_service_t reward_service;

rcl_timer_t sensor_timer;
rcl_timer_t sync_pulse_base_timer;
rcl_timer_t sync_pulse_start_timer;
rcl_timer_t sync_pulse_end_timer;

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
tabletop_msgs__srv__SetFloat_Request reward_request;
tabletop_msgs__srv__SetFloat_Response reward_response;

static bool sync_pulse_state;
static uint8_t agent_reconnect_retries;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  CLIENT_ERROR
} state;

// Macro definitions

// RCCHECK: Check return code from ROS2 function, print error string and return NULL if error
#define RCCHECK(fn)                                      \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK))                         \
    {                                                    \
      printf("Error: %s\n", rcl_get_error_string().str); \
      return NULL;                                       \
    }                                                    \
  }
// RCSOFTCHECK: Check return code from ROS2 function, continue execution
#define RCSOFTCHECK(fn)                                  \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK))                         \
    {                                                    \
      printf("Error: %s\n", rcl_get_error_string().str); \
    }                                                    \
  }
// LOG: Log a message to the ROS2 log topic
#define LOG(fmt, ...)                                         \
  {                                                           \
    rosidl_runtime_c__String *str = &log_msg.data;            \
    snprintf(str->data, str->capacity, fmt, ##__VA_ARGS__);   \
    str->size = strlen(str->data);                            \
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL)); \
  }
// ASSERT: Assert a condition, logging an error if the condition is not met
#define ASSERT(fn, fmt, ...)       \
  {                                \
    if (!(fn))                     \
    {                              \
      printf("Assertion failed!"); \
      printf(fmt, ##__VA_ARGS__);  \
    }                              \
  }
// Execute a block of code every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X) \
  {                               \
    static int64_t init = -1;     \
    if (init == -1)               \
    {                             \
      init = uxr_millis();        \
    }                             \
    if (uxr_millis() - init > MS) \
    {                             \
      X;                          \
      init = uxr_millis();        \
    }                             \
  }

// Error handle loop
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Service callback for arm_door
void arm_door_callback(const void *req, void *res)
{
  std_srvs__srv__SetBool_Request *request = (std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res;

  digitalWrite(ARM_DOOR_CONTROL_PIN, request->data);
  if (request->data)
  {
    strcpy(response->message.data, "Arm door opened");
  }
  else
  {
    strcpy(response->message.data, "Arm door closed");
  }
  response->success = true;
}

// Service callback for smartglass
void smartglass_callback(const void *req, void *res)
{
  std_srvs__srv__SetBool_Request *request = (std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res;

  digitalWrite(SMARTGLASS_CONTROL_PIN, request->data);
  if (request->data)
  {
    strcpy(response->message.data, "Smartglass revealed");
  }
  else
  {
    strcpy(response->message.data, "Smartglass occluded");
  }
  response->success = true;
}

// Service callback for reward
void reward_callback(const void *req, void *res)
{
  tabletop_msgs__srv__SetFloat_Request *request = (tabletop_msgs__srv__SetFloat_Request *)req;
  tabletop_msgs__srv__SetFloat_Response *response = (tabletop_msgs__srv__SetFloat_Response *)res;

  digitalWrite(REWARD_CONTROL_PIN, HIGH);
  delay(request->data * 1000);
  digitalWrite(REWARD_CONTROL_PIN, LOW);

  strcpy(response->message.data, "Reward delivered");
  response->success = true;
}

void sensor_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Populate sensor message
    sensor_msg.arm_door_laser_state_right = digitalRead(ARM_DOOR_STATE_PIN);
    sensor_msg.arm_door_laser_state_left = digitalRead(ARM_DOOR_STATE_PIN); // Assuming same pin for both
    sensor_msg.smart_glass_laser_state = digitalRead(SMARTGLASS_STATE_PIN);
    sensor_msg.fixation_button_state = digitalRead(HAND_FIXATION_STATE_PIN);
    for (int i = 0; i < 5; i++)
    {
      sensor_msg.tactile_glove_states[i] = analogRead(GLOVE_STATE_PINS[i]);
    }
    sensor_msg.sync_pulse = sync_pulse_state;

    RCSOFTCHECK(rcl_publish(&sensor_publisher, &sensor_msg, NULL));
  }
}

void sync_pulse_end_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    LOG("Sync pulse end");
    digitalWrite(SYNC_PULSE_PIN, LOW);
    sync_pulse_state = false;
    RCSOFTCHECK(rcl_timer_cancel(timer));
  }
}

void sync_pulse_start_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    LOG("Sync pulse start");
    digitalWrite(SYNC_PULSE_PIN, HIGH);
    sync_pulse_state = true;
    RCSOFTCHECK(rcl_timer_exchange_period(&sync_pulse_end_timer, RCL_MS_TO_NS(SYNC_PULSE_DURATION_MS), NULL));
    RCSOFTCHECK(rcl_timer_reset(&sync_pulse_end_timer));
    RCSOFTCHECK(rcl_timer_cancel(timer));
  }
}

void sync_pulse_base_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  if (timer != NULL)
  {
    static int64_t uxr_last_time = uxr_millis();
    int64_t uxr_now = uxr_millis();
    LOG("uxr_now: %lld ms", uxr_now);
    LOG("uxr_last_time: %lld ms", uxr_last_time);
    LOG("uxr_now - uxr_last_time: %lld ms", uxr_now - uxr_last_time);
    uxr_last_time = uxr_now;

    static rcl_time_point_value_t rcl_last_time = -1;
    rcl_time_point_value_t rcl_now;
    RCSOFTCHECK(rcl_clock_get_now(&support.clock, &rcl_now));
    LOG("rcl_now: %lld ms", RCL_NS_TO_MS(rcl_now));
    LOG("rcl_last_time: %lld ms", RCL_NS_TO_MS(rcl_last_time));
    LOG("last_call_time: %lld ms", RCL_NS_TO_MS(last_call_time));
    LOG("rcl_now - rcl_last_time: %lld ms", RCL_NS_TO_MS(rcl_now - rcl_last_time));
    int64_t correction_ns;
    if (rcl_last_time < -1)
    {
      correction_ns = 0;
    }
    else
    {
      correction_ns = (rcl_now - rcl_last_time) - RCL_MS_TO_NS(SYNC_PULSE_BASE_PERIOD_MS);
      ASSERT(correction_ns >= 0, "correction_ns < 0");
    }
    rcl_last_time = rcl_now;

    // Generate a random delay between 0 and NOISE_RANGE
    int64_t delay_ns = RCL_MS_TO_NS(random(SYNC_PULSE_DELAY_RANGE_MS)) + correction_ns;

    // uint32_t delay_ms = random(SYNC_PULSE_DELAY_RANGE_MS);

    // Wait for the delay
    LOG("delay_ms: %lld ms", RCL_NS_TO_MS(delay_ns));
    digitalWrite(SYNC_PULSE_PIN, LOW);
    sync_pulse_state = false;
    RCSOFTCHECK(rcl_timer_exchange_period(&sync_pulse_start_timer, delay_ns, NULL));
    RCSOFTCHECK(rcl_timer_reset(&sync_pulse_start_timer));
  }
}

bool create_entities()
{
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
  RCCHECK(rclc_publisher_init_best_effort(&sensor_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_msgs, msg, TeensySensor), "teensy/sensors"));
  RCCHECK(rclc_publisher_init_best_effort(&log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "teensy/log"));
  LOG("Publishers initialized");

  // create services
  RCCHECK(rclc_service_init_default(&arm_door_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                    "arm_door"));
  RCCHECK(rclc_service_init_default(&smartglass_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                    "smartglass"));
  RCCHECK(rclc_service_init_default(&reward_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_msgs, srv, SetFloat),
                                    "reward"));
  LOG("Services initialized");

  // create timers
  RCCHECK(rclc_timer_init_default2(&sensor_timer, &support, RCL_MS_TO_NS(SENSOR_PERIOD_MS), sensor_timer_callback, true));
  RCCHECK(rclc_timer_init_default2(&sync_pulse_base_timer, &support, RCL_MS_TO_NS(SYNC_PULSE_BASE_PERIOD_MS), sync_pulse_base_timer_callback, true));
  RCCHECK(rclc_timer_init_default2(&sync_pulse_start_timer, &support, RCL_MS_TO_NS(SYNC_PULSE_DELAY_RANGE_MS), sync_pulse_start_timer_callback, false));
  RCCHECK(rclc_timer_init_default2(&sync_pulse_end_timer, &support, RCL_MS_TO_NS(SYNC_PULSE_DURATION_MS), sync_pulse_end_timer_callback, false));
  LOG("Timers initialized");

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_base_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_start_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_end_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
  RCCHECK(rclc_executor_add_service(&executor, &arm_door_service, &arm_door_request, &arm_door_response,
                                    &arm_door_callback));
  RCCHECK(rclc_executor_add_service(&executor, &smartglass_service, &smartglass_request, &smartglass_response,
                                    &smartglass_callback));
  RCCHECK(rclc_executor_add_service(&executor, &reward_service, &reward_request, &reward_response, &reward_callback));
  LOG("Executor initialized");

  LOG("Message memories created");

  return true;
}

void destroy_entities()
{
  // Set context entity destroy session timeout
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  RCSOFTCHECK(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  // Destroy entities
  RCSOFTCHECK(rcl_publisher_fini(&sensor_publisher, &node));
  RCSOFTCHECK(rcl_publisher_fini(&log_publisher, &node));
  RCSOFTCHECK(rcl_service_fini(&arm_door_service, &node));
  RCSOFTCHECK(rcl_service_fini(&smartglass_service, &node));
  RCSOFTCHECK(rcl_service_fini(&reward_service, &node));
  RCSOFTCHECK(rcl_timer_fini(&sensor_timer));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_base_timer));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_start_timer));
  RCSOFTCHECK(rcl_timer_fini(&sync_pulse_end_timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  printf("Entities destroyed\n");
}

void setup()
{
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
  sync_pulse_state = false;
  agent_reconnect_retries = 0;
  state = WAITING_AGENT;

  // create message memories
  bool success = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_srvs, srv, SetBool_Response),
      &arm_door_response,
      memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_srvs, srv, SetBool_Response),
      &smartglass_response,
      memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_srvs, srv, SetBool_Response),
      &reward_response,
      memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      &log_msg,
      memory_conf);
  if (!success)
  {
    printf("Failed to create message memories\n");
    error_loop();
  }

  delay(2000);
}

void loop()
{
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_RECONNECT_TIMEOUT_MS, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    static bool success = true;
    EXECUTE_EVERY_N_MS(AGENT_RECONNECT_PERIOD_MS, success = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_RECONNECT_TIMEOUT_MS, 1)););
    agent_reconnect_retries = success ? 0 : agent_reconnect_retries + 1;
    if (agent_reconnect_retries < AGENT_RECONNECT_MAX_RETRIES)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_TIMEOUT_MS));
    }
    else
    {
      state = AGENT_DISCONNECTED;
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  case CLIENT_ERROR:
    destroy_entities();
    printf("Client error\n");
    error_loop();
    break;
  default:
    printf("Unknown state\n");
    error_loop();
    break;
  }

  if (state == AGENT_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
