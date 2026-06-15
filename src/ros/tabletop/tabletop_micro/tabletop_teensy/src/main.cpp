// =============================================================================
// tabletop_teensy/src/main.cpp
// Teensy 4.1 micro-ROS firmware for the TableTop monkey electrophysiology rig.
//
// Built with PlatformIO (tt-microros-build).  Serial transport only
// (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL).
//
// ROS entities exposed (all under the "teensy" namespace):
//   Publisher : teensy/sensor  -- tabletop_interfaces/msg/TeensySensor, BEST_EFFORT, ~100 Hz
//   Publisher : teensy/log     -- std_msgs/msg/String, RELIABLE, human-readable diagnostics
//   Service   : teensy/ping              -- tabletop_interfaces/srv/Ping
//   Service   : teensy/set_arm_lock      -- tabletop_interfaces/srv/SetArmLock
//   Service   : teensy/set_smartglass    -- tabletop_interfaces/srv/SetSmartglass
//   Service   : teensy/set_reward        -- tabletop_interfaces/srv/SetReward
//   Service   : teensy/set_solenoid      -- tabletop_interfaces/srv/SetSolenoid
//
// Main-loop cadence:
//   loop() drives an rclc_executor handling timers and service callbacks.
//   sensor_timer fires every SENSOR_PERIOD_MS (10 ms) for 100 Hz sensor publishes.
//   sync_pulse_base_timer fires every SYNC_PULSE_BASE_PERIOD_MS (~1 s) and schedules
//   a randomly jittered sync pulse via two one-shot timers.
//   camera_trigger_timer is a hardware PIT IntervalTimer that toggles
//   CAMERA_TRIGGER_CONTROL_PIN at 2x120 Hz (120 fps square wave), running
//   independently of the micro-ROS session lifecycle.
// =============================================================================
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

// Guard: this firmware is only built with Arduino serial transport.
#ifndef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
#error This code only supports serial transport.
#endif

// #define DEBUG_LOGGING

// =============================================================================
// Pin definitions
// =============================================================================
// Define pin mappings
// NOTE: Pin 37 does not work
// --- Output control pins ---
// LEFT_ARM_LOCK_CONTROL_PIN: drives the left arm restraint solenoid (HIGH = engaged)
#define LEFT_ARM_LOCK_CONTROL_PIN 4
// RIGHT_ARM_LOCK_CONTROL_PIN: drives the right arm restraint solenoid (HIGH = engaged)
#define RIGHT_ARM_LOCK_CONTROL_PIN 5
// LEFT_ARM_BUZZER_CONTROL_PIN: buzzes left arm buzzer on unlock (HIGH = buzzing)
#define LEFT_ARM_BUZZER_CONTROL_PIN 41
// RIGHT_ARM_BUZZER_CONTROL_PIN: buzzes right arm buzzer on unlock (HIGH = buzzing)
#define RIGHT_ARM_BUZZER_CONTROL_PIN 40
// SMARTGLASS_CONTROL_PIN: controls LCD shutter goggles (HIGH = transparent/revealed)
#define SMARTGLASS_CONTROL_PIN 3
// REWARD_CONTROL_PIN: opens the juice reward solenoid (HIGH = dispensing)
#define REWARD_CONTROL_PIN 26
// SYNC_PULSE_CONTROL_PIN: sync pulse to recording equipment (HIGH during pulse)
#define SYNC_PULSE_CONTROL_PIN 9
// SOLENOID_CONTROL_PIN: auxiliary solenoid; mirrors sync pulse when is_solenoid_active
#define SOLENOID_CONTROL_PIN 12
// CAMERA_TRIGGER_CONTROL_PIN: 50% duty square wave at CAMERA_TRIGGER_FPS to FLIR cameras
#define CAMERA_TRIGGER_CONTROL_PIN 33
// --- Input sense pins ---
// SAFETY_LASER_STATE_PIN: safety laser photodetector (HIGH = beam broken, danger state)
#define SAFETY_LASER_STATE_PIN 25
// LEFT_ARM_LOCK_STATE_PIN: left arm restraint feedback (LOW = arm seated/locked)
#define LEFT_ARM_LOCK_STATE_PIN 38  // TODO: change back to 36
// RIGHT_ARM_LOCK_STATE_PIN: right arm restraint feedback (LOW = arm seated/locked)
#define RIGHT_ARM_LOCK_STATE_PIN 39
// BUTTON_STATE_PIN: subject response button (LOW = pressed, active-low with INPUT_PULLUP)
// Note: pin 36 currently used here because pin 38 is occupied by left arm lock (see TODO above)
#define BUTTON_STATE_PIN 36
// LEFT_GLOVE_STATE_PINS: 10-bit ADC inputs for left-hand glove pressure sensors (A0-A4)
static const uint8_t LEFT_GLOVE_STATE_PINS[] = { A0, A1, A2, A3, A4 };
// RIGHT_GLOVE_STATE_PINS: 10-bit ADC inputs for right-hand glove pressure sensors (A5-A9)
static const uint8_t RIGHT_GLOVE_STATE_PINS[] = { A5, A6, A7, A8, A9 };

// Define pin states
// The digital level that signals the active/triggered condition.
// SAFETY_LASER_BROKEN_STATE: HIGH = beam interrupted (open-collector / pull-up topology)
#define SAFETY_LASER_BROKEN_STATE HIGH
// LEFT_ARM_LOCKED_STATE: LOW = arm seated in restraint (active-low switch)
#define LEFT_ARM_LOCKED_STATE LOW
// RIGHT_ARM_LOCKED_STATE: LOW = arm seated in restraint (active-low switch)
#define RIGHT_ARM_LOCKED_STATE LOW
// BUTTON_PRESSED_STATE: LOW = button depressed (INPUT_PULLUP, active-low)
#define BUTTON_PRESSED_STATE LOW

// Define interrupt trigger conditions
// All ISRs fire on CHANGE so timestamps are captured for both edges
// (assertion and de-assertion), enabling edge-latched event timestamps.
#define SAFETY_LASER_ISR_TRIGGER CHANGE
#define LEFT_ARM_ISR_TRIGGER CHANGE
#define RIGHT_ARM_ISR_TRIGGER CHANGE
#define BUTTON_ISR_TRIGGER CHANGE

// Message memory configuration
// Used by micro_ros_utilities_create_message_memory.
// Allocates 100 bytes of string capacity and limits sequences to 5 elements.
static const micro_ros_utilities_memory_conf_t memory_conf = { 100, 5, 5, NULL, 0, NULL };

// =============================================================================
// ROS 2 node identity and entity names
// =============================================================================
// ROS2 node name
#define NODE_NAME "teensy"
#define NODE_NS ""

// ROS2 topics
// "~/" expands to "/<NODE_NS>/<NODE_NAME>/" at runtime.
// SENSOR_TOPIC: tabletop_interfaces/msg/TeensySensor, BEST_EFFORT QoS
#define SENSOR_TOPIC "~/sensor"
// LOG_TOPIC: std_msgs/msg/String, RELIABLE QoS; used for human-readable diagnostics
#define LOG_TOPIC "~/log"

// ROS2 services
// PING_SRV_NAME: connectivity check; responds with the current synchronized ROS time
#define PING_SRV_NAME "~/ping"
// SET_ARM_LOCK_SRV_NAME: lock/release arm restraints and trigger buzzer on unlock
#define SET_ARM_LOCK_SRV_NAME "~/set_arm_lock"
// SET_SMARTGLASS_SRV_NAME: reveal (transparent) or occlude LCD shutter goggles
#define SET_SMARTGLASS_SRV_NAME "~/set_smartglass"
// SET_REWARD_SRV_NAME: open juice solenoid for a caller-specified duration
#define SET_REWARD_SRV_NAME "~/set_reward"
// SET_SOLENOID_SRV_NAME: arm/disarm auxiliary solenoid (follows sync pulse when armed)
#define SET_SOLENOID_SRV_NAME "~/set_solenoid"

// =============================================================================
// Timing and execution parameters
// =============================================================================
// Execution parameters
// AGENT_RECONNECT_PERIOD_MS: how often loop() pings for the micro-ROS agent while disconnected
#define AGENT_RECONNECT_PERIOD_MS 100
// AGENT_RECONNECT_TIMEOUT_MS: per-ping timeout used by rmw_uros_ping_agent()
#define AGENT_RECONNECT_TIMEOUT_MS 20
// EXECUTOR_SPIN_TIMEOUT_MS: max time rclc_executor_spin_some() may block per loop iteration
#define EXECUTOR_SPIN_TIMEOUT_MS 50
// AGENT_SYNC_PERIOD_MS: how often ROS time is re-synchronized in AGENT_CONNECTED state
#define AGENT_SYNC_PERIOD_MS 200
// AGENT_SYNC_TIMEOUT_MS: per-call timeout for rmw_uros_sync_session()
#define AGENT_SYNC_TIMEOUT_MS 1
// AGENT_SYNC_MAX_RETRIES: consecutive sync failures before triggering disconnection
#define AGENT_SYNC_MAX_RETRIES 3
// SENSOR_PERIOD_MS: sensor_timer period; sets publish rate to ~100 Hz
#define SENSOR_PERIOD_MS 10
// SYNC_PULSE_BASE_PERIOD_MS: nominal inter-pulse interval (~1 s)
#define SYNC_PULSE_BASE_PERIOD_MS 1000
// SYNC_PULSE_DELAY_MIN_MS / MAX_MS: random jitter range added before each pulse onset
#define SYNC_PULSE_DELAY_MIN_MS 50
#define SYNC_PULSE_DELAY_MAX_MS 200
// SYNC_PULSE_DURATION_MS: how long SYNC_PULSE_CONTROL_PIN stays HIGH each cycle
#define SYNC_PULSE_DURATION_MS 100
// ARM_BUZZER_DURATION_MS: how long the unlock buzzer sounds after a release command
#define ARM_BUZZER_DURATION_MS 1000
// DEBOUNCE_DELAY_MS: minimum quiet time before a pin transition is considered stable
#define DEBOUNCE_DELAY_MS 1
// DEBOUNCE_DELAY_NS: DEBOUNCE_DELAY_MS expressed in nanoseconds for epoch comparisons
#define DEBOUNCE_DELAY_NS RCL_MS_TO_NS(DEBOUNCE_DELAY_MS)
// CAMERA_TRIGGER_FPS: target frame rate for the FLIR cameras
#define CAMERA_TRIGGER_FPS 120
// CAMERA_TRIGGER_TOGGLE_PERIOD_US: IntervalTimer period for a 50% duty square wave at CAMERA_TRIGGER_FPS
#define CAMERA_TRIGGER_TOGGLE_PERIOD_US (1000000.0f / (2 * CAMERA_TRIGGER_FPS))
// CAMERA_TRIGGER_ISR_PRIORITY: Teensy interrupt priority (lower = higher priority)
#define CAMERA_TRIGGER_ISR_PRIORITY 64

// Builtin LED agent state indicator
//    Steady on:    WAITING_AGENT
//    Steady off:   AGENT_AVAILABLE | AGENT_DISCONNECTED
//    Blink slow:   AGENT_CONNECTED
//    Blink fast:   UNRECOVERABLE_ERROR
// If UNRECOVERABLE_ERROR state is reached, the teensy needs to be rebooted
// This should not usually happen and indicates an error in the source code.
#define BLINK_CONNECTED_PERIOD_MS 500
#define BLINK_ERROR_PERIOD_MS 100

// =============================================================================
// Global micro-ROS entities and state variables
// =============================================================================
// Global variables
// --- Publishers ---
// sensor_publisher: publishes TeensySensor on ~/sensor (~100 Hz, BEST_EFFORT)
rcl_publisher_t sensor_publisher;
// log_publisher: publishes diagnostic strings on ~/log (RELIABLE)
rcl_publisher_t log_publisher;

// --- Services ---
rcl_service_t ping_service;
rcl_service_t set_arm_lock_service;
rcl_service_t set_smartglass_service;
rcl_service_t set_reward_service;
rcl_service_t set_solenoid_service;

// Hardware (PIT) timer for the camera exposure trigger. This is
// intentionally not an rcl timer: it must stay immune to micro-ROS
// session/time synchronization delays.
IntervalTimer camera_trigger_timer;

// --- rcl timers (managed by the executor) ---
// sync_pulse_base_timer: repeating ~1 s; triggers each sync-pulse cycle
rcl_timer_t sync_pulse_base_timer;
// sync_pulse_start_timer: one-shot; raises pulse after random jitter delay
rcl_timer_t sync_pulse_start_timer;
// sync_pulse_end_timer: one-shot; lowers pulse after SYNC_PULSE_DURATION_MS
rcl_timer_t sync_pulse_end_timer;
// sensor_timer: repeating 10 ms; drives the 100 Hz TeensySensor publish loop
rcl_timer_t sensor_timer;
// reward_timer: one-shot; closes the reward solenoid after the requested duration
rcl_timer_t reward_timer;
// arm_buzzer_timer: one-shot; silences the unlock buzzer after ARM_BUZZER_DURATION_MS
rcl_timer_t arm_buzzer_timer;

// --- micro-ROS runtime context ---
// allocator: default heap allocator used for all rcl/rclc init calls
rcl_allocator_t allocator;
// support: wraps rcl init options and context for rclc convenience API
rclc_support_t support;
// node: the "teensy" ROS 2 node
rcl_node_t node;
// executor: single-threaded executor spinning all timers and service callbacks
rclc_executor_t executor;

// --- Message buffers (statically allocated; reused on every publish/service call) ---
// sensor_msg: reused every sensor_timer tick (100 Hz)
tabletop_interfaces__msg__TeensySensor sensor_msg;
// log_msg: reused for every LOG() call; string capacity pre-allocated via memory_conf
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
// agent_sync_retries: counts consecutive failed rmw_uros_sync_session() calls
uint8_t agent_sync_retries;

// agent_state machine used by loop():
//   WAITING_AGENT      -- probing for micro-ROS agent; builtin LED steady ON
//   AGENT_AVAILABLE    -- agent found; about to call init_client()
//   AGENT_CONNECTED    -- session active, executor spinning; LED slow-blink
//   AGENT_DISCONNECTED -- session lost; about to call deinit_client()
//   UNCRECOVERABLE_ERROR -- fatal init failure; LED fast-blink; requires reboot
enum agent_states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  UNCRECOVERABLE_ERROR,
} agent_state;

// --- Output state mirrors (kept in sync by the set_* helpers) ---
// is_reward_active: true while REWARD_CONTROL_PIN is HIGH (solenoid open)
bool is_reward_active;
// is_smartglass_revealed: true while SMARTGLASS_CONTROL_PIN is HIGH (goggles transparent)
bool is_smartglass_revealed;
// is_solenoid_active: true when SOLENOID_CONTROL_PIN should follow the sync pulse
bool is_solenoid_active;

// --- Sync pulse bookkeeping ---
// sync_pulse_state: current output level of SYNC_PULSE_CONTROL_PIN
bool sync_pulse_state;
// sync_pulse_last_time_on/off: ROS timestamps of the most recent rising/falling edges
builtin_interfaces__msg__Time sync_pulse_last_time_on;
builtin_interfaces__msg__Time sync_pulse_last_time_off;

// --- ISR-shared debounce variables (volatile; access guarded by noInterrupts/interrupts) ---
// Safety laser (SAFETY_LASER_STATE_PIN)
// safety_laser_last_time_broken_ns: epoch ns when the beam was last newly broken
volatile int64_t safety_laser_last_time_broken_ns;
// safety_laser_last_time_bounced_ns: epoch ns of the most recent ISR call (any edge)
volatile int64_t safety_laser_last_time_bounced_ns;
// safety_laser_state_stable: debounced pin level after DEBOUNCE_DELAY_NS quiet time
volatile uint8_t safety_laser_state_stable;

// Left arm restraint (LEFT_ARM_LOCK_STATE_PIN)
// left_arm_last_time_locked_ns: epoch ns when arm was last newly seated
volatile int64_t left_arm_last_time_locked_ns;
volatile int64_t left_arm_last_time_bounced_ns;
volatile uint8_t left_arm_state_stable;

// Right arm restraint (RIGHT_ARM_LOCK_STATE_PIN)
// right_arm_last_time_locked_ns: epoch ns when arm was last newly seated
volatile int64_t right_arm_last_time_locked_ns;
volatile int64_t right_arm_last_time_bounced_ns;
volatile uint8_t right_arm_state_stable;

// Response button (BUTTON_STATE_PIN)
// button_last_time_pressed_ns: epoch ns when button was last newly pressed
volatile int64_t button_last_time_pressed_ns;
volatile int64_t button_last_time_bounced_ns;
volatile uint8_t button_state_stable;

// =============================================================================
// Utility macros
// =============================================================================
// Macro definitions
// RCCHECK: evaluates fn; on rcl failure prints the error string and returns false.
// Used inside bool-returning init/deinit functions.
#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      printf("Error: %s\n", rcl_get_error_string().str);                                                               \
      return false;                                                                                                    \
    }                                                                                                                  \
  }
// RCSOFTCHECK: like RCCHECK but does not return; logs the error and continues.
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      printf("Error: %s\n", rcl_get_error_string().str);                                                               \
    }                                                                                                                  \
  }
// STRING_SET: snprintf into a rosidl string struct, then updates its size field.
#define STRING_SET(str_ptr, fmt, ...)                                                                                  \
  {                                                                                                                    \
    snprintf((str_ptr)->data, (str_ptr)->capacity, fmt, ##__VA_ARGS__);                                                \
    (str_ptr)->size = strlen((str_ptr)->data);                                                                         \
  }
// LOG: format and publish a diagnostic string on ~/log. Always active.
#define LOG(fmt, ...)                                                                                                  \
  {                                                                                                                    \
    STRING_SET(&log_msg.data, fmt, ##__VA_ARGS__);                                                                     \
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));                                                          \
  }
// DEBUG: same as LOG when DEBUG_LOGGING is defined; otherwise a no-op.
#ifdef DEBUG_LOGGING
#define DEBUG LOG
#else
#define DEBUG(...)
#endif
// RCASSERT: evaluates fn; on failure publishes the rcl error and a custom message
// to ~/log. Does not halt execution -- use for non-fatal runtime assertions.
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
// ASSERT: like RCASSERT but for plain bool conditions (not rcl return codes).
#define ASSERT(fn, fmt, ...)                                                                                           \
  {                                                                                                                    \
    bool temp_success = fn;                                                                                            \
    if (!temp_success)                                                                                                 \
    {                                                                                                                  \
      LOG("Assertion failed!");                                                                                        \
      LOG(fmt, ##__VA_ARGS__);                                                                                         \
    }                                                                                                                  \
  }
// EXECUTE_EVERY_N_MS: runs statement X at most once per MS milliseconds.
// Uses a per-call-site static variable so each usage site has its own timer.
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
// Time conversion helpers between epoch nanoseconds and builtin_interfaces/Time.
#define RCL_S_TO_MS(sec) (sec * 1000LL)
#define ROS_TIME_TO_MS(time_msg) (RCL_S_TO_MS(time_msg.sec) + RCL_NS_TO_MS(time_msg.nanosec))
#define ROS_TIME_TO_NS(time_msg) (RCL_S_TO_NS(time_msg.sec) + time_msg.nanosec)
// NS_TO_ROS_TIME: splits a 64-bit nanosecond epoch value into (sec, nanosec) fields.
#define NS_TO_ROS_TIME(time_msg, ns)                                                                                   \
  {                                                                                                                    \
    time_msg.sec = ns / (1000LL * 1000LL * 1000LL);                                                                    \
    time_msg.nanosec = ns % (1000LL * 1000LL * 1000LL);                                                                \
  }
// GET_CURRENT_ROS_TIME: reads the synchronized ROS epoch via rmw_uros_epoch_nanos()
// and stores it into a builtin_interfaces/Time struct using NS_TO_ROS_TIME.
#define GET_CURRENT_ROS_TIME(time_msg)                                                                                 \
  {                                                                                                                    \
    int64_t now_ns = rmw_uros_epoch_nanos();                                                                           \
    NS_TO_ROS_TIME(time_msg, now_ns);                                                                                  \
  }

// =============================================================================
// Interrupt Service Routines (ISRs)
// =============================================================================
// camera_trigger_toggle_isr: hardware PIT ISR called every CAMERA_TRIGGER_TOGGLE_PERIOD_US.
// ISR toggling the camera trigger pin at 2 * CAMERA_TRIGGER_FPS, producing
// a 50% duty square wave with rising edges at CAMERA_TRIGGER_FPS
// Runs at CAMERA_TRIGGER_ISR_PRIORITY, independent of the micro-ROS session.
static void camera_trigger_toggle_isr()
{
  static bool state = false;
  state = !state;
  digitalWriteFast(CAMERA_TRIGGER_CONTROL_PIN, state ? HIGH : LOW);
}

// safety_laser_broken_isr: CHANGE ISR for SAFETY_LASER_STATE_PIN.
// Records the synchronized ROS epoch time of any edge; updates
// safety_laser_last_time_broken_ns only on the first transition into the broken state.
static void safety_laser_broken_isr()
{
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (safety_laser_state_stable != SAFETY_LASER_BROKEN_STATE)
  {
    safety_laser_last_time_broken_ns = now_ns;
  }
  safety_laser_last_time_bounced_ns = now_ns;
}
// left_arm_locked_isr: CHANGE ISR for LEFT_ARM_LOCK_STATE_PIN.
// Records the epoch time of every edge; also latches left_arm_last_time_locked_ns
// on the first transition into the locked (LOW) state.
static void left_arm_locked_isr()
{
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (left_arm_state_stable != LEFT_ARM_LOCKED_STATE)
  {
    left_arm_last_time_locked_ns = now_ns;
  }
  left_arm_last_time_bounced_ns = now_ns;
}
// right_arm_locked_isr: CHANGE ISR for RIGHT_ARM_LOCK_STATE_PIN. Same pattern as left_arm_locked_isr.
static void right_arm_locked_isr()
{
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (right_arm_state_stable != RIGHT_ARM_LOCKED_STATE)
  {
    right_arm_last_time_locked_ns = now_ns;
  }
  right_arm_last_time_bounced_ns = now_ns;
}
// button_pressed_isr: CHANGE ISR for BUTTON_STATE_PIN.
// Records the epoch time of every edge; also latches button_last_time_pressed_ns
// on the first transition into the pressed (LOW) state.
static void button_pressed_isr()
{
  int64_t now_ns = rmw_uros_epoch_nanos();
  if (button_state_stable != BUTTON_PRESSED_STATE)
  {
    button_last_time_pressed_ns = now_ns;
  }
  button_last_time_bounced_ns = now_ns;
}

// =============================================================================
// Output control helpers
// =============================================================================
// set_left_arm_lock: drive LEFT_ARM_LOCK_CONTROL_PIN (true = HIGH = restraint engaged).
static inline void set_left_arm_lock(bool lock)
{
  digitalWriteFast(LEFT_ARM_LOCK_CONTROL_PIN, lock ? HIGH : LOW);
}
// set_right_arm_lock: drive RIGHT_ARM_LOCK_CONTROL_PIN (true = HIGH = restraint engaged).
static inline void set_right_arm_lock(bool lock)
{
  digitalWriteFast(RIGHT_ARM_LOCK_CONTROL_PIN, lock ? HIGH : LOW);
}

// set_smartglass: drive SMARTGLASS_CONTROL_PIN and mirror state to is_smartglass_revealed.
// true (HIGH) = goggles transparent; false (LOW) = opaque.
static inline void set_smartglass(bool reveal)
{
  digitalWriteFast(SMARTGLASS_CONTROL_PIN, reveal ? HIGH : LOW);
  is_smartglass_revealed = reveal;
}
// set_reward: drive REWARD_CONTROL_PIN and mirror state to is_reward_active.
// true (HIGH) = juice solenoid open/dispensing; false (LOW) = closed.
static inline void set_reward(bool activate)
{
  digitalWriteFast(REWARD_CONTROL_PIN, activate ? HIGH : LOW);
  is_reward_active = activate;
}
// set_sync_pulse: drive SYNC_PULSE_CONTROL_PIN and, when is_solenoid_active, also
// SOLENOID_CONTROL_PIN. Updates sync_pulse_state mirror.
// Called by sync_pulse_start/end_timer_callback.
static inline void set_sync_pulse(bool activate)
{
  digitalWriteFast(SYNC_PULSE_CONTROL_PIN, activate ? HIGH : LOW);
  if (is_solenoid_active)
  {
    digitalWriteFast(SOLENOID_CONTROL_PIN, activate ? HIGH : LOW);
  }
  sync_pulse_state = activate;
}
// set_solenoid: arm or disarm the auxiliary solenoid.
// On deactivation, immediately drives SOLENOID_CONTROL_PIN LOW.
// On activation, the pin will follow the next sync pulse via set_sync_pulse().
static inline void set_solenoid(bool activate)
{
  if (!activate)
  {
    digitalWriteFast(SOLENOID_CONTROL_PIN, LOW);
  }
  is_solenoid_active = activate;
}

// =============================================================================
// rcl Timer callbacks
// =============================================================================
// sensor_timer_callback: fires every SENSOR_PERIOD_MS (10 ms); ~100 Hz.
// Timer callback for publishing the sensor message
// Reads and debounces all sensor inputs (with interrupts disabled for ISR-shared
// variables), populates sensor_msg, and publishes it on ~/sensor.
// last_call_time is used to widen the "is_X_broken/locked" window to catch
// transitions that resolve between timer ticks.
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

// sync_pulse_end_timer_callback: one-shot; fires SYNC_PULSE_DURATION_MS after pulse onset.
// Timer callback to stop the sync pulse
// Lowers SYNC_PULSE_CONTROL_PIN, records the falling-edge timestamp, and cancels itself.
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

// sync_pulse_start_timer_callback: one-shot; fires after the random jitter delay set by
// sync_pulse_base_timer_callback.
// Timer callback to start the sync pulse
// Raises SYNC_PULSE_CONTROL_PIN, records the rising-edge timestamp, cancels itself,
// and arms sync_pulse_end_timer.
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

// sync_pulse_base_timer_callback: repeating, fires every SYNC_PULSE_BASE_PERIOD_MS (~1 s).
// Timer callback to start the sync pulse delay timer
// Picks a random jitter in [DELAY_MIN, DELAY_MAX], sets it as sync_pulse_start_timer's
// period, and resets that timer (arms the one-shot).
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

// =============================================================================
// Service callbacks
// =============================================================================
// ping_callback: backs ~/ping (tabletop_interfaces/srv/Ping).
// Service callback for ping
// Returns the current synchronized ROS time as received_time.
// success is false only when time synchronization has not yet occurred (both fields zero).
void ping_callback(const void* req, void* res)
{
  RCLC_UNUSED(req);
  tabletop_interfaces__srv__Ping_Response* response = static_cast<tabletop_interfaces__srv__Ping_Response*>(res);

  GET_CURRENT_ROS_TIME(response->received_time);
  response->success = (response->received_time.sec != 0) || (response->received_time.nanosec != 0);
}

// reward_timer_callback: one-shot; fires after the duration requested in SetReward.
// Timer callback to stop the reward control
// Closes the reward solenoid (set_reward(false)) and cancels itself.
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

// set_reward_callback: backs ~/set_reward (tabletop_interfaces/srv/SetReward).
// Service callback for controlling the reward
// activate=true: opens the juice solenoid for request.duration, arms reward_timer.
// activate=false: closes solenoid immediately and cancels reward_timer.
// Returns success=false when duration is inconsistently provided or omitted.
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

// arm_buzzer_callback: one-shot; fires ARM_BUZZER_DURATION_MS after an unlock command.
// Timer callback to stop the arm buzzer control
// Drives both arm buzzer pins LOW and cancels itself.
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

// set_arm_lock_callback: backs ~/set_arm_lock (tabletop_interfaces/srv/SetArmLock).
// Service callback for controlling the arm lock
// Locks or releases the specified arm(s). On unlock, buzzes the corresponding
// buzzer(s) and arms arm_buzzer_timer for ARM_BUZZER_DURATION_MS.
// Returns success=false when neither arm is specified in the request.
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

// set_smartglass_callback: backs ~/set_smartglass (tabletop_interfaces/srv/SetSmartglass).
// Service callback for controlling the smartglass
// Reveals (transparent) or occludes the LCD shutter goggles via set_smartglass().
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

// set_solenoid_callback: backs ~/set_solenoid (tabletop_interfaces/srv/SetSolenoid).
// Service callback for controlling the solenoid
// Arms or disarms the auxiliary solenoid via set_solenoid().
// When armed, SOLENOID_CONTROL_PIN follows the next sync pulse; when disarmed, goes LOW.
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

// =============================================================================
// Session lifecycle helpers
// =============================================================================
// reset_state: drives all outputs to their safe defaults and clears all timestamp
// and debounce state variables. Called at session init, deinit, and from setup().
// Safe defaults: both arm locks engaged, smartglass revealed, reward/sync/solenoid off.
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

// init_client: creates all micro-ROS entities for one agent session.
// Order: reset_state -> allocator -> support -> node -> publishers -> services ->
//        timers -> executor (11 handles) -> time sync -> attach ISRs.
// Returns true on success; RCCHECK() returns false on any rcl failure.
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

// deinit_client: tears down all micro-ROS entities after a session ends.
// Order: detach ISRs -> reset_state -> set destroy timeout to 0 ->
//        fini publishers, timers, services, executor, node, support.
// Returns true on success; transition target is WAITING_AGENT or UNCRECOVERABLE_ERROR.
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

// =============================================================================
// Arduino entry points
// =============================================================================
// setup: runs once at power-on or reset.
// Initialises serial transport (115200 baud), configures all GPIO pin modes,
// pre-allocates micro-ROS message memory for string-bearing service responses
// and the log message, calls reset_state(), starts the free-running
// camera_trigger_timer, and sets initial agent_state.
// agent_state is WAITING_AGENT on success or UNCRECOVERABLE_ERROR if memory
// allocation fails for any message type.
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
  pinMode(CAMERA_TRIGGER_CONTROL_PIN, OUTPUT);
  digitalWriteFast(CAMERA_TRIGGER_CONTROL_PIN, LOW);

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

  // Start the free-running camera exposure trigger. This runs for the
  // lifetime of the program, independent of the agent connection state.
  camera_trigger_timer.begin(camera_trigger_toggle_isr, CAMERA_TRIGGER_TOGGLE_PERIOD_US);
  camera_trigger_timer.priority(CAMERA_TRIGGER_ISR_PRIORITY);
  printf("Camera trigger started at %d fps\n", CAMERA_TRIGGER_FPS);

  delay(1000);
}

// loop: runs continuously after setup(); drives the agent_state machine.
//   WAITING_AGENT      -- polls rmw_uros_ping_agent() every AGENT_RECONNECT_PERIOD_MS.
//   AGENT_AVAILABLE    -- calls init_client(); transitions to CONNECTED or DISCONNECTED.
//   AGENT_CONNECTED    -- slow-blinks LED, re-syncs ROS time every AGENT_SYNC_PERIOD_MS,
//                         spins executor; disconnects on sync failure or executor error.
//   AGENT_DISCONNECTED -- calls deinit_client(); transitions to WAITING_AGENT or ERROR.
//   UNCRECOVERABLE_ERROR -- fast-blinks LED; Teensy reboot required.
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
