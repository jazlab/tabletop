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
#include <tabletop_interfaces/srv/ping.h>
#include <tabletop_interfaces/srv/set_arm_lock.h>
#include <tabletop_interfaces/srv/set_reward.h>
#include <tabletop_interfaces/srv/set_smartglass.h>
#include <tabletop_interfaces/srv/set_solenoid.h>
#include <atomic.h>

#include "core_pins.h"
#include "rmw/qos_profiles.h"
#include "rmw_microros/time_sync.h"

#ifndef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
#error This code only supports serial transport.
#endif

// #define DEBUG_LOGGING

// Define pin mappings
// NOTE: Pin 37 does not work
#define LEFT_ARM_LOCK_CONTROL_PIN 4
#define RIGHT_ARM_LOCK_CONTROL_PIN 5
#define LEFT_ARM_BUZZER_CONTROL_PIN 41
#define RIGHT_ARM_BUZZER_CONTROL_PIN 40
#define SMARTGLASS_CONTROL_PIN 3
#define REWARD_CONTROL_PIN 26
#define SYNC_PULSE_CONTROL_PIN 9
#define SOLENOID_CONTROL_PIN 12
#define SAFETY_LASER_STATE_PIN 25
#define LEFT_ARM_LOCK_STATE_PIN 38  // TODO: change back to 36
#define RIGHT_ARM_LOCK_STATE_PIN 39
#define BUTTON_STATE_PIN 36
static const uint8_t LEFT_GLOVE_STATE_PINS[] = { A0, A1, A2, A3, A4 };
static const uint8_t RIGHT_GLOVE_STATE_PINS[] = { A5, A6, A7, A8, A9 };

// Define pin states
#define SAFETY_LASER_BROKEN_STATE HIGH
#define LEFT_ARM_LOCKED_STATE LOW
#define RIGHT_ARM_LOCKED_STATE LOW
#define BUTTON_PRESSED_STATE LOW

// Define interrupt trigger conditions
#define SAFETY_LASER_ISR_TRIGGER CHANGE
#define LEFT_ARM_ISR_TRIGGER CHANGE
#define RIGHT_ARM_ISR_TRIGGER CHANGE
#define BUTTON_ISR_TRIGGER CHANGE

// Message memory configuration
static const micro_ros_utilities_memory_conf_t memory_conf = { 100, 5, 5, NULL, 0, NULL };

// ROS2 node name
#define NODE_NAME "teensy"
#define NODE_NS ""

// ROS2 topics
#define SENSOR_TOPIC "~/sensor"
#define LOG_TOPIC "~/log"

// ROS2 services
#define PING_SRV_NAME "~/ping"
#define SET_ARM_LOCK_SRV_NAME "~/set_arm_lock"
#define SET_SMARTGLASS_SRV_NAME "~/set_smartglass"
#define SET_REWARD_SRV_NAME "~/set_reward"
#define SET_SOLENOID_SRV_NAME "~/set_solenoid"

// Execution parameters
#define AGENT_RECONNECT_PERIOD_MS 100
#define AGENT_RECONNECT_TIMEOUT_MS 20
#define EXECUTOR_SPIN_TIMEOUT_MS 50
#define AGENT_SYNC_PERIOD_MS 200
#define AGENT_SYNC_TIMEOUT_MS 1
#define AGENT_SYNC_MAX_RETRIES 3
#define SENSOR_PERIOD_MS 10
#define SYNC_PULSE_BASE_PERIOD_MS 1000
#define SYNC_PULSE_DELAY_MIN_MS 50
#define SYNC_PULSE_DELAY_MAX_MS 200
#define SYNC_PULSE_DURATION_MS 100
#define ARM_BUZZER_DURATION_MS 1000
#define DEBOUNCE_DELAY_MS 1
#define DEBOUNCE_DELAY_NS RCL_MS_TO_NS(DEBOUNCE_DELAY_MS)

// Builtin LED agent state indicator
//    Steady on:    WAITING_AGENT
//    Steady off:   AGENT_AVAILABLE | AGENT_DISCONNECTED
//    Blink slow:   AGENT_CONNECTED
//    Blink fast:   UNRECOVERABLE_ERROR
// If UNRECOVERABLE_ERROR state is reached, the teensy needs to be rebooted
// This should not usually happen and indicates an error in the source code.
#define BLINK_CONNECTED_PERIOD_MS 500
#define BLINK_ERROR_PERIOD_MS 100

// Global variables
rcl_publisher_t sensor_publisher;
rcl_publisher_t log_publisher;

rcl_service_t ping_service;
rcl_service_t set_arm_lock_service;
rcl_service_t set_smartglass_service;
rcl_service_t set_reward_service;
rcl_service_t set_solenoid_service;

rcl_timer_t sync_pulse_base_timer;
rcl_timer_t sync_pulse_start_timer;
rcl_timer_t sync_pulse_end_timer;
rcl_timer_t sensor_timer;
rcl_timer_t reward_timer;
rcl_timer_t arm_buzzer_timer;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

tabletop_interfaces__msg__TeensySensor sensor_msg;
std_msgs__msg__String log_msg;

tabletop_interfaces__srv__Ping_Request ping_request;
tabletop_interfaces__srv__Ping_Response ping_response;
tabletop_interfaces__srv__SetArmLock_Request set_arm_lock_request;
tabletop_interfaces__srv__SetArmLock_Response set_arm_lock_response;
tabletop_interfaces__srv__SetSmartglass_Request set_smartglass_request;
tabletop_interfaces__srv__SetSmartglass_Response set_smartglass_response;
tabletop_interfaces__srv__SetReward_Request set_reward_request;
tabletop_interfaces__srv__SetReward_Response set_reward_response;
tabletop_interfaces__srv__SetSolenoid_Request set_solenoid_request;
tabletop_interfaces__srv__SetSolenoid_Response set_solenoid_response;

// State tracking
uint8_t agent_sync_retries;

enum agent_states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  UNCRECOVERABLE_ERROR,
} agent_state;

bool is_reward_active;
bool is_smartglass_revealed;
bool is_solenoid_active;

bool sync_pulse_state;
builtin_interfaces__msg__Time sync_pulse_last_time_on;
builtin_interfaces__msg__Time sync_pulse_last_time_off;

volatile int64_t safety_laser_last_time_broken_ns;
volatile int64_t safety_laser_last_time_bounced_ns;
volatile uint8_t safety_laser_state_stable;

volatile int64_t left_arm_last_time_locked_ns;
volatile int64_t left_arm_last_time_bounced_ns;
volatile uint8_t left_arm_state_stable;

volatile int64_t right_arm_last_time_locked_ns;
volatile int64_t right_arm_last_time_bounced_ns;
volatile uint8_t right_arm_state_stable;

volatile int64_t button_last_time_pressed_ns;
volatile int64_t button_last_time_bounced_ns;
volatile uint8_t button_state_stable;

// Macro definitions
#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      printf("Error: %s\n", rcl_get_error_string().str);                                                               \
      return false;                                                                                                    \
    }                                                                                                                  \
  }
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      printf("Error: %s\n", rcl_get_error_string().str);                                                               \
    }                                                                                                                  \
  }
#define STRING_SET(str_ptr, fmt, ...)                                                                                  \
  {                                                                                                                    \
    snprintf((str_ptr)->data, (str_ptr)->capacity, fmt, ##__VA_ARGS__);                                                \
    (str_ptr)->size = strlen((str_ptr)->data);                                                                         \
  }
#define LOG(fmt, ...)                                                                                                  \
  {                                                                                                                    \
    STRING_SET(&log_msg.data, fmt, ##__VA_ARGS__);                                                                     \
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));                                                          \
  }
#ifdef DEBUG_LOGGING
#define DEBUG LOG
#else
#define DEBUG(...)
#endif
#define RCASSERT(fn, fmt, ...)                                                                                         \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      LOG("RC Assertion failed!");                                                                                     \
      LOG("Error: %s", rcl_get_error_string().str);                                                                    \
      LOG(fmt, ##__VA_ARGS__);                                                                                         \
    }                                                                                                                  \
  }
#define ASSERT(fn, fmt, ...)                                                                                           \
  {                                                                                                                    \
    bool temp_success = fn;                                                                                            \
    if (!temp_success)                                                                                                 \
    {                                                                                                                  \
      LOG("Assertion failed!");                                                                                        \
      LOG(fmt, ##__VA_ARGS__);                                                                                         \
    }                                                                                                                  \
  }
#define EXECUTE_EVERY_N_MS(MS, X)                                                                                      \
  {                                                                                                                    \
    static int64_t init = -1;                                                                                          \
    if (init == -1)                                                                                                    \
    {                                                                                                                  \
      init = uxr_millis();                                                                                             \
    }                                                                                                                  \
    if (uxr_millis() - init > MS)                                                                                      \
    {                                                                                                                  \
      X;                                                                                                               \
      init = uxr_millis();                                                                                             \
    }                                                                                                                  \
  }
#define RCL_S_TO_MS(sec) (sec * 1000LL)
#define ROS_TIME_TO_MS(time_msg) (RCL_S_TO_MS(time_msg.sec) + RCL_NS_TO_MS(time_msg.nanosec))
#define ROS_TIME_TO_NS(time_msg) (RCL_S_TO_NS(time_msg.sec) + time_msg.nanosec)
#define NS_TO_ROS_TIME(time_msg, ns)                                                                                   \
  {                                                                                                                    \
    time_msg.sec = ns / (1000LL * 1000LL * 1000LL);                                                                    \
    time_msg.nanosec = ns % (1000LL * 1000LL * 1000LL);                                                                \
  }
#define GET_CURRENT_ROS_TIME(time_msg)                                                                                 \
  {                                                                                                                    \
    int64_t now_ns = rmw_uros_epoch_nanos();                                                                           \
    NS_TO_ROS_TIME(time_msg, now_ns);                                                                                  \
  }

static void safety_laser_broken_isr()
{
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (safety_laser_state_stable != SAFETY_LASER_BROKEN_STATE)
  {
    safety_laser_last_time_broken_ns = now_ns;
  }
  safety_laser_last_time_bounced_ns = now_ns;
}
static void left_arm_locked_isr()
{
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (left_arm_state_stable != LEFT_ARM_LOCKED_STATE)
  {
    left_arm_last_time_locked_ns = now_ns;
  }
  left_arm_last_time_bounced_ns = now_ns;
}
static void right_arm_locked_isr()
{
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (right_arm_state_stable != RIGHT_ARM_LOCKED_STATE)
  {
    right_arm_last_time_locked_ns = now_ns;
  }
  right_arm_last_time_bounced_ns = now_ns;
}
static void button_pressed_isr()
{
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (button_state_stable != BUTTON_PRESSED_STATE)
  {
    button_last_time_pressed_ns = now_ns;
  }
  button_last_time_bounced_ns = now_ns;
}

static inline void set_left_arm_lock(bool lock)
{
  digitalWriteFast(LEFT_ARM_LOCK_CONTROL_PIN, lock ? HIGH : LOW);
}
static inline void set_right_arm_lock(bool lock)
{
  digitalWriteFast(RIGHT_ARM_LOCK_CONTROL_PIN, lock ? HIGH : LOW);
}

static inline void set_smartglass(bool reveal)
{
  digitalWriteFast(SMARTGLASS_CONTROL_PIN, reveal ? HIGH : LOW);
  is_smartglass_revealed = reveal;
}
static inline void set_reward(bool activate)
{
  digitalWriteFast(REWARD_CONTROL_PIN, activate ? HIGH : LOW);
  is_reward_active = activate;
}
static inline void set_sync_pulse(bool activate)
{
  digitalWriteFast(SYNC_PULSE_CONTROL_PIN, activate ? HIGH : LOW);
  if (is_solenoid_active)
  {
    digitalWriteFast(SOLENOID_CONTROL_PIN, activate ? HIGH : LOW);
  }
  sync_pulse_state = activate;
}
static inline void set_solenoid(bool activate)
{
  if (!activate)
  {
    digitalWriteFast(SOLENOID_CONTROL_PIN, LOW);
  }
  is_solenoid_active = activate;
}

// Timer callback for publishing the sensor message
void sensor_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  if (timer != NULL)
  {
    noInterrupts();

    // Update stable sensor state
    int64_t now_ns = rmw_uros_epoch_nanos();
    int64_t time_since_safety_laser_bounced = now_ns - safety_laser_last_time_bounced_ns;
    int64_t time_since_left_arm_bounced = now_ns - left_arm_last_time_bounced_ns;
    int64_t time_since_right_arm_bounced = now_ns - right_arm_last_time_bounced_ns;
    int64_t time_since_button_bounced = now_ns - button_last_time_bounced_ns;
    if (time_since_safety_laser_bounced > DEBOUNCE_DELAY_NS)
    {
      safety_laser_state_stable = digitalReadFast(SAFETY_LASER_STATE_PIN);
    }
    if (time_since_left_arm_bounced > DEBOUNCE_DELAY_NS)
    {
      left_arm_state_stable = digitalReadFast(LEFT_ARM_LOCK_STATE_PIN);
    }
    if (time_since_right_arm_bounced > DEBOUNCE_DELAY_NS)
    {
      right_arm_state_stable = digitalReadFast(RIGHT_ARM_LOCK_STATE_PIN);
    }
    if (time_since_button_bounced > DEBOUNCE_DELAY_NS)
    {
      button_state_stable = digitalReadFast(BUTTON_STATE_PIN);
    }

    // Safety laser/arm lock/button states
    sensor_msg.is_safety_laser_broken =
        (safety_laser_state_stable == SAFETY_LASER_BROKEN_STATE) || (time_since_safety_laser_bounced < last_call_time);
    sensor_msg.is_left_arm_locked =
        (left_arm_state_stable == LEFT_ARM_LOCKED_STATE) && (time_since_left_arm_bounced > last_call_time);
    sensor_msg.is_right_arm_locked =
        (right_arm_state_stable == RIGHT_ARM_LOCKED_STATE) && (time_since_right_arm_bounced > last_call_time);
    sensor_msg.is_button_pressed = button_state_stable == BUTTON_PRESSED_STATE;

    // Safety laser/button last time broken/pressed
    NS_TO_ROS_TIME(sensor_msg.safety_laser_last_time_broken, safety_laser_last_time_broken_ns);
    NS_TO_ROS_TIME(sensor_msg.button_last_time_pressed, button_last_time_pressed_ns);

    interrupts();

    // Populate remaining sensor message
    GET_CURRENT_ROS_TIME(sensor_msg.header.stamp);
    sensor_msg.is_reward_active = is_reward_active;
    sensor_msg.is_smartglass_revealed = is_smartglass_revealed;

    // Update tactile glove states
    for (size_t i = 0; i < 5; i++)
    {
      sensor_msg.left_tactile_glove_states[i] = analogRead(LEFT_GLOVE_STATE_PINS[i]);
    }
    for (size_t i = 0; i < 5; i++)
    {
      sensor_msg.right_tactile_glove_states[i] = analogRead(RIGHT_GLOVE_STATE_PINS[i]);
    }

    // Update sync pulse states
    sensor_msg.sync_pulse_state = sync_pulse_state;
    sensor_msg.sync_pulse_last_time_on = sync_pulse_last_time_on;
    sensor_msg.sync_pulse_last_time_off = sync_pulse_last_time_off;

    RCASSERT(rcl_publish(&sensor_publisher, &sensor_msg, NULL), "Failed to publish sensor message");
  }
}

// Timer callback to stop the sync pulse
void sync_pulse_end_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    set_sync_pulse(false);
    GET_CURRENT_ROS_TIME(sync_pulse_last_time_off);

    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel sync pulse end timer");

    DEBUG("Sync pulse ended after %lld ms",
          ROS_TIME_TO_MS(sync_pulse_last_time_off) - ROS_TIME_TO_MS(sync_pulse_last_time_on));
  }
}

// Timer callback to start the sync pulse
void sync_pulse_start_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    set_sync_pulse(true);
    GET_CURRENT_ROS_TIME(sync_pulse_last_time_on);

    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel sync pulse start timer");
    RCASSERT(rcl_timer_reset(&sync_pulse_end_timer), "Failed to reset sync pulse end timer");

    DEBUG("Sync pulse started for %d ms", SYNC_PULSE_DURATION_MS);
  }
}

// Timer callback to start the sync pulse delay timer
void sync_pulse_base_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    ASSERT(!sync_pulse_state, "Sync pulse is should be off when the base timer is called");

    int64_t delay_ms = random(SYNC_PULSE_DELAY_MAX_MS - SYNC_PULSE_DELAY_MIN_MS) + SYNC_PULSE_DELAY_MIN_MS;
    int64_t old_period;
    RCASSERT(rcl_timer_exchange_period(&sync_pulse_start_timer, RCL_MS_TO_NS(delay_ms), &old_period),
             "Failed to exchange sync pulse start timer period");
    RCASSERT(rcl_timer_reset(&sync_pulse_start_timer), "Failed to reset sync pulse start timer");

    DEBUG("Sync pulse start scheduled for %lld ms from now", delay_ms);
  }
}

// Service callback for ping
void ping_callback(const void* req, void* res)
{
  RCLC_UNUSED(req);
  tabletop_interfaces__srv__Ping_Response* response = static_cast<tabletop_interfaces__srv__Ping_Response*>(res);

  GET_CURRENT_ROS_TIME(response->received_time);
  response->success = (response->received_time.sec != 0) || (response->received_time.nanosec != 0);
}

// Timer callback to stop the reward control
void reward_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    set_reward(false);
    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel reward timer");
    LOG("Reward finished");
  }
}

// Service callback for controlling the reward
void set_reward_callback(const void* req, void* res)
{
  const tabletop_interfaces__srv__SetReward_Request* request =
      static_cast<const tabletop_interfaces__srv__SetReward_Request*>(req);
  tabletop_interfaces__srv__SetReward_Response* response =
      static_cast<tabletop_interfaces__srv__SetReward_Response*>(res);

  bool timer_is_canceled;
  RCASSERT(rcl_timer_is_canceled(&reward_timer, &timer_is_canceled), "Failed to check if reward timer is canceled");
  ASSERT(is_reward_active != timer_is_canceled, "Reward timer state and reward_active state are inconsistent");

  if (request->activate)
  {
    // Activate reward for the duration specified in the request

    if (request->duration.sec == 0 && request->duration.nanosec == 0)
    {
      // If the reward is being activated, the duration should be provided
      response->success = false;
      STRING_SET(&response->message, "Reward duration should be provided when activating");
      LOG("%s", response->message.data);
      return;
    }

    // Activate the reward
    set_reward(true);

    int64_t duration_ns = ROS_TIME_TO_NS(request->duration);
    int64_t old_period_ns;
    RCASSERT(rcl_timer_exchange_period(&reward_timer, duration_ns, &old_period_ns),
             "Failed to exchange reward timer period");
    RCASSERT(rcl_timer_reset(&reward_timer), "Failed to reset reward timer");

    double duration_s = duration_ns / 1e9;
    STRING_SET(&response->message, "Reward %s for %.3f s", timer_is_canceled ? "started" : "extended", duration_s);
  }
  else
  {
    // Cancel the reward timer if it is not canceled and stop the reward

    if (request->duration.sec != 0 || request->duration.nanosec != 0)
    {
      // If the reward is being deactivated, the duration should be 0
      response->success = false;
      STRING_SET(&response->message, "Reward duration should not be provided when deactivating");
      LOG("%s", response->message.data);
      return;
    }

    // Deactivate the reward
    set_reward(false);

    // Cancel the reward timer if it is not canceled
    if (!timer_is_canceled)
    {
      int64_t time_until_ns;
      int64_t old_period_ns;
      RCASSERT(rcl_timer_get_period(&reward_timer, &old_period_ns), "Failed to get reward timer period");
      RCASSERT(rcl_timer_get_time_until_next_call(&reward_timer, &time_until_ns), "Failed to get time until next call");
      RCASSERT(rcl_timer_cancel(&reward_timer), "Failed to cancel reward timer");

      double time_since_s = (old_period_ns - time_until_ns) / 1e9;
      STRING_SET(&response->message, "Reward stopped after %.3f s", time_since_s);
    }
    else
    {
      STRING_SET(&response->message, "Reward already stopped");
    }
  }

  response->success = true;
  LOG("%s", response->message.data);
}

// Timer callback to stop the arm buzzer control
void arm_buzzer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    digitalWriteFast(LEFT_ARM_BUZZER_CONTROL_PIN, LOW);
    digitalWriteFast(RIGHT_ARM_BUZZER_CONTROL_PIN, LOW);
    RCASSERT(rcl_timer_cancel(timer), "Failed to cancel arm buzzer timer");
    LOG("Arm buzzers stopped");
  }
}

// Service callback for controlling the arm lock
void set_arm_lock_callback(const void* req, void* res)
{
  const tabletop_interfaces__srv__SetArmLock_Request* request =
      static_cast<const tabletop_interfaces__srv__SetArmLock_Request*>(req);
  tabletop_interfaces__srv__SetArmLock_Response* response =
      static_cast<tabletop_interfaces__srv__SetArmLock_Response*>(res);

  char message_arm[20] = "";
  if (request->left_arm && request->right_arm)
  {
    strcpy(message_arm, "Both arms");
  }
  else if (request->left_arm)
  {
    strcpy(message_arm, "Left arm");
  }
  else if (request->right_arm)
  {
    strcpy(message_arm, "Right arm");
  }
  else
  {
    response->success = false;
    STRING_SET(&response->message, "No arm specified");
    LOG("%s", response->message.data);
    return;
  }

  if (request->left_arm)
  {
    set_left_arm_lock(request->lock);
  }
  if (request->right_arm)
  {
    set_right_arm_lock(request->lock);
  }

  if (!request->lock)
  {
    if (request->left_arm)
    {
      digitalWriteFast(LEFT_ARM_BUZZER_CONTROL_PIN, HIGH);
    }
    if (request->right_arm)
    {
      digitalWriteFast(RIGHT_ARM_BUZZER_CONTROL_PIN, HIGH);
    }
    RCASSERT(rcl_timer_reset(&arm_buzzer_timer), "Failed to reset arm buzzer timer");
  }

  response->success = true;
  STRING_SET(&response->message, "%s %s", message_arm, request->lock ? "locked" : "released");
  LOG("%s", response->message.data);
}

// Service callback for controlling the smartglass
void set_smartglass_callback(const void* req, void* res)
{
  const tabletop_interfaces__srv__SetSmartglass_Request* request =
      static_cast<const tabletop_interfaces__srv__SetSmartglass_Request*>(req);
  tabletop_interfaces__srv__SetSmartglass_Response* response =
      static_cast<tabletop_interfaces__srv__SetSmartglass_Response*>(res);

  set_smartglass(request->reveal);

  response->success = true;
  STRING_SET(&response->message, "Smartglass %s", request->reveal ? "revealed" : "occluded");
  LOG("%s", response->message.data);
}

// Service callback for controlling the solenoid
void set_solenoid_callback(const void* req, void* res)
{
  const tabletop_interfaces__srv__SetSolenoid_Request* request =
      static_cast<const tabletop_interfaces__srv__SetSolenoid_Request*>(req);
  tabletop_interfaces__srv__SetSolenoid_Response* response =
      static_cast<tabletop_interfaces__srv__SetSolenoid_Response*>(res);

  set_solenoid(request->activate);

  response->success = true;
  STRING_SET(&response->message, "Solenoid %s", request->activate ? "activated" : "deactivated");
  LOG("%s", response->message.data);
}

void reset_state()
{
  set_left_arm_lock(true);
  set_right_arm_lock(true);
  set_smartglass(true);
  set_reward(false);
  set_sync_pulse(false);
  set_solenoid(false);

  agent_sync_retries = 0;
  sync_pulse_last_time_on.sec = 0;
  sync_pulse_last_time_on.nanosec = 0;
  sync_pulse_last_time_off.sec = 0;
  sync_pulse_last_time_off.nanosec = 0;
  safety_laser_last_time_broken_ns = 0;
  safety_laser_last_time_bounced_ns = 0;
  left_arm_last_time_locked_ns = 0;
  left_arm_last_time_bounced_ns = 0;
  right_arm_last_time_locked_ns = 0;
  right_arm_last_time_bounced_ns = 0;
  button_last_time_pressed_ns = 0;
  button_last_time_bounced_ns = 0;

  noInterrupts();
  safety_laser_state_stable = digitalReadFast(SAFETY_LASER_STATE_PIN);
  left_arm_state_stable = digitalReadFast(LEFT_ARM_LOCK_STATE_PIN);
  right_arm_state_stable = digitalReadFast(RIGHT_ARM_LOCK_STATE_PIN);
  button_state_stable = digitalReadFast(BUTTON_STATE_PIN);
  interrupts();
}

bool init_client()
{
  // Reset output pins
  reset_state();

  // Initialize allocator
  allocator = rcl_get_default_allocator();

  // Initialize support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // RCCHECK(rclc_support_init_with_clock(&support, 0, NULL, RCL_STEADY_TIME,
  //                                      &allocator));

  // Initialize node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NS, &support));

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(
      &sensor_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, msg, TeensySensor), SENSOR_TOPIC));
  RCCHECK(rclc_publisher_init_default(&log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                      LOG_TOPIC));
  LOG("Publishers initialized");

  // Services
  RCCHECK(rclc_service_init_default(&ping_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, Ping),
                                    PING_SRV_NAME));
  RCCHECK(rclc_service_init_default(&set_arm_lock_service, &node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, SetArmLock),
                                    SET_ARM_LOCK_SRV_NAME));
  RCCHECK(rclc_service_init_default(&set_smartglass_service, &node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, SetSmartglass),
                                    SET_SMARTGLASS_SRV_NAME));
  RCCHECK(rclc_service_init_default(&set_reward_service, &node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, SetReward),
                                    SET_REWARD_SRV_NAME));
  RCCHECK(rclc_service_init_default(&set_solenoid_service, &node,
                                    ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, SetSolenoid),
                                    SET_SOLENOID_SRV_NAME));
  LOG("Services initialized");

  // Timers
  RCCHECK(rclc_timer_init_default2(&sync_pulse_base_timer, &support, RCL_MS_TO_NS(SYNC_PULSE_BASE_PERIOD_MS),
                                   sync_pulse_base_timer_callback, true));
  RCCHECK(rclc_timer_init_default2(&sync_pulse_start_timer, &support, RCL_MS_TO_NS(SYNC_PULSE_DELAY_MIN_MS),
                                   sync_pulse_start_timer_callback, false));
  RCCHECK(rclc_timer_init_default2(&sync_pulse_end_timer, &support, RCL_MS_TO_NS(SYNC_PULSE_DURATION_MS),
                                   sync_pulse_end_timer_callback, false));
  RCCHECK(
      rclc_timer_init_default2(&sensor_timer, &support, RCL_MS_TO_NS(SENSOR_PERIOD_MS), sensor_timer_callback, true));
  RCCHECK(rclc_timer_init_default2(&reward_timer, &support, RCL_MS_TO_NS(1000), reward_timer_callback, false));
  RCCHECK(rclc_timer_init_default2(&arm_buzzer_timer, &support, RCL_MS_TO_NS(ARM_BUZZER_DURATION_MS),
                                   arm_buzzer_callback, false));
  LOG("Timers initialized");

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 11, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_base_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_start_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_pulse_end_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &reward_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &arm_buzzer_timer));
  RCCHECK(rclc_executor_add_service(&executor, &ping_service, &ping_request, &ping_response, ping_callback));
  RCCHECK(rclc_executor_add_service(&executor, &set_arm_lock_service, &set_arm_lock_request, &set_arm_lock_response,
                                    set_arm_lock_callback));
  RCCHECK(rclc_executor_add_service(&executor, &set_smartglass_service, &set_smartglass_request,
                                    &set_smartglass_response, set_smartglass_callback));
  RCCHECK(rclc_executor_add_service(&executor, &set_reward_service, &set_reward_request, &set_reward_response,
                                    set_reward_callback));
  RCCHECK(rclc_executor_add_service(&executor, &set_solenoid_service, &set_solenoid_request, &set_solenoid_response,
                                    set_solenoid_callback));
  LOG("Executor initialized");

  RCCHECK(rmw_uros_sync_session(1000));

  LOG("Session synced");

  // Attach interrupt service routines
  attachInterrupt(digitalPinToInterrupt(SAFETY_LASER_STATE_PIN), safety_laser_broken_isr, SAFETY_LASER_ISR_TRIGGER);
  attachInterrupt(digitalPinToInterrupt(LEFT_ARM_LOCK_STATE_PIN), left_arm_locked_isr, LEFT_ARM_ISR_TRIGGER);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ARM_LOCK_STATE_PIN), right_arm_locked_isr, RIGHT_ARM_ISR_TRIGGER);
  attachInterrupt(digitalPinToInterrupt(BUTTON_STATE_PIN), button_pressed_isr, BUTTON_ISR_TRIGGER);

  return true;
}

bool deinit_client()
{
  // Detach interrupt service routines
  detachInterrupt(digitalPinToInterrupt(SAFETY_LASER_STATE_PIN));
  detachInterrupt(digitalPinToInterrupt(LEFT_ARM_LOCK_STATE_PIN));
  detachInterrupt(digitalPinToInterrupt(RIGHT_ARM_LOCK_STATE_PIN));
  detachInterrupt(digitalPinToInterrupt(BUTTON_STATE_PIN));

  // Reset output pins
  reset_state();

  // Destroy session
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  RCCHECK(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  // Destroy entities
  RCCHECK(rcl_publisher_fini(&sensor_publisher, &node));
  RCCHECK(rcl_publisher_fini(&log_publisher, &node));
  RCCHECK(rcl_timer_fini(&sync_pulse_base_timer));
  RCCHECK(rcl_timer_fini(&sync_pulse_start_timer));
  RCCHECK(rcl_timer_fini(&sync_pulse_end_timer));
  RCCHECK(rcl_timer_fini(&sensor_timer));
  RCCHECK(rcl_timer_fini(&reward_timer));
  RCCHECK(rcl_timer_fini(&arm_buzzer_timer));
  RCCHECK(rcl_service_fini(&ping_service, &node));
  RCCHECK(rcl_service_fini(&set_arm_lock_service, &node));
  RCCHECK(rcl_service_fini(&set_smartglass_service, &node));
  RCCHECK(rcl_service_fini(&set_reward_service, &node));
  RCCHECK(rcl_service_fini(&set_solenoid_service, &node));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));

  printf("Client deinitialized\n");

  return true;
}

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  printf("Serial transport initialized\n");

  // Initialize output pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEFT_ARM_LOCK_CONTROL_PIN, OUTPUT);
  pinMode(RIGHT_ARM_LOCK_CONTROL_PIN, OUTPUT);
  pinMode(LEFT_ARM_BUZZER_CONTROL_PIN, OUTPUT);
  pinMode(RIGHT_ARM_BUZZER_CONTROL_PIN, OUTPUT);
  pinMode(SMARTGLASS_CONTROL_PIN, OUTPUT);
  pinMode(REWARD_CONTROL_PIN, OUTPUT);
  pinMode(SYNC_PULSE_CONTROL_PIN, OUTPUT);
  pinMode(SOLENOID_CONTROL_PIN, OUTPUT);

  // Initialize input pins
  pinMode(SAFETY_LASER_STATE_PIN, INPUT_PULLUP);
  pinMode(LEFT_ARM_LOCK_STATE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ARM_LOCK_STATE_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STATE_PIN, INPUT_PULLUP);
  for (size_t i = 0; i < 5; i++)
  {
    pinMode(LEFT_GLOVE_STATE_PINS[i], INPUT);
  }
  for (size_t i = 0; i < 5; i++)
  {
    pinMode(RIGHT_GLOVE_STATE_PINS[i], INPUT);
  }

  printf("Pins initialized\n");

  // Reset state
  reset_state();
  printf("State reset\n");

  // create message memories
  bool success = micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, srv, SetArmLock_Response), &set_arm_lock_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, srv,
                                                                                   SetSmartglass_Response),
                                                       &set_smartglass_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, srv, SetReward_Response), &set_reward_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(tabletop_interfaces, srv, SetSolenoid_Response), &set_solenoid_response, memory_conf);
  success &= micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), &log_msg,
                                                       memory_conf);

  agent_state = success ? WAITING_AGENT : UNCRECOVERABLE_ERROR;

  delay(1000);
}

void loop()
{
  switch (agent_state)
  {
    case WAITING_AGENT:
      digitalWrite(LED_BUILTIN, HIGH);
      EXECUTE_EVERY_N_MS(AGENT_RECONNECT_PERIOD_MS,
                         agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_RECONNECT_TIMEOUT_MS, 1)) ?
                                           AGENT_AVAILABLE :
                                           WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      digitalWrite(LED_BUILTIN, LOW);
      agent_state = init_client() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(BLINK_CONNECTED_PERIOD_MS, digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)););
      EXECUTE_EVERY_N_MS(AGENT_SYNC_PERIOD_MS,
                         agent_sync_retries = (RMW_RET_OK == rmw_uros_sync_session(AGENT_SYNC_TIMEOUT_MS)) ?
                                                  0 :
                                                  agent_sync_retries + 1;);
      if ((agent_sync_retries >= AGENT_SYNC_MAX_RETRIES) ||
          (RCL_RET_OK != rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_TIMEOUT_MS))))
      {
        agent_state = AGENT_DISCONNECTED;
      }
      break;
    case AGENT_DISCONNECTED:
      digitalWrite(LED_BUILTIN, LOW);
      agent_state = deinit_client() ? WAITING_AGENT : UNCRECOVERABLE_ERROR;
      break;
    case UNCRECOVERABLE_ERROR:
      EXECUTE_EVERY_N_MS(BLINK_ERROR_PERIOD_MS, digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)););
      break;
  }
}
