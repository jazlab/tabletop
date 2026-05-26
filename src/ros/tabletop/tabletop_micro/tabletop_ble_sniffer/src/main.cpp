// ESP32 Flic BLE sniffer exposed over micro-ROS.
//
// Mirrors the press-detection logic from the original main.ino (NimBLE
// continuous scan, Flic-address prefix filter, per-button cooldown) and
// publishes each detected press as a std_msgs/Header on
// ~/button_pressed_time, matching the topic and payload that
// tabletop_rig.nodes.flic publishes from the Python flic node's
// spin_button_publisher_adv_packet method.
//
// Latency-comparison features carried over from the .ino:
//   - Hardware sync button on HW_BUTTON_PIN. Its press time is
//     published on ~/hw_sync_pressed_time so latency vs. the BLE
//     detection can be computed in ROS.
//   - The same average + stddev stats from the .ino are emitted on the
//     ~/log topic every MAX_MEASUREMENTS HW->BLE pairs.
//
// Design notes:
//   - NimBLE scanning starts once in setup() and runs forever; the
//     micro-ROS state machine (copied from tabletop_teensy) brings the
//     publishers up/down as the agent comes/goes. BLE events that
//     arrive while the agent is down are silently dropped because the
//     queue is bounded — we don't want to publish stale timestamps.
//   - The .ino's resetButtonAds() reconnect was intentionally dropped.
//     It blocks the main loop for >1 s, which stalls executor spin and
//     time-sync and causes the agent to disconnect. Instead we use a
//     longer per-button cooldown (~2 s) to absorb the Flic's ~3 s
//     post-press advertisement burst. If you need active silencing
//     later, run it from a dedicated FreeRTOS task fed by a separate
//     queue so the main loop stays responsive.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <builtin_interfaces/msg/time.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>

#include "rmw_microros/time_sync.h"

#ifndef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
#error This code only supports serial transport.
#endif

// ── Configuration ──────────────────────────────────────────────────────
// Hardware sync button (active-low). Pin 13 also drives LED_BUILTIN on
// the Feather ESP32 V1, so the on-board LED is unusable while this pin
// is wired to a button — fine for our use, just noting it.
static const int HW_BUTTON_PIN = 13;
static const uint32_t HW_BUTTON_DEBOUNCE_US = 500000;  // 500 ms

// Per-button cooldown. Sized to absorb a single Flic press' adv burst
// without firing multiple events. Shorter → risk of duplicate events
// per press; longer → rapid presses get coalesced.
static const uint32_t PER_BUTTON_COOLDOWN_MS = 2000;

// Bounded queue of press events from BLE callback → main loop.
static const size_t BLE_PRESS_QUEUE_LEN = 16;

// Latency stats batch size (matches the .ino).
static const int MAX_MEASUREMENTS = 10;

// micro-ROS topology
#define NODE_NAME "ble_sniffer"
#define NODE_NS ""
#define BLE_PRESS_TOPIC "~/button_pressed_time"
#define HW_PRESS_TOPIC "~/hw_sync_pressed_time"
#define LOG_TOPIC "~/log"

// Agent-supervision timings (mirrors tabletop_teensy).
#define AGENT_RECONNECT_PERIOD_MS 100
#define AGENT_RECONNECT_TIMEOUT_MS 20
#define EXECUTOR_SPIN_TIMEOUT_MS 5
#define AGENT_SYNC_PERIOD_MS 200
#define AGENT_SYNC_TIMEOUT_MS 1
#define AGENT_SYNC_MAX_RETRIES 3
#define BLINK_CONNECTED_PERIOD_MS 500

// Message memory configuration (used to back the Header.frame_id /
// String.data fields).
#define MAX_STRING_CAPACITY 100
static const micro_ros_utilities_memory_conf_t memory_conf = { MAX_STRING_CAPACITY, 5, 5, NULL, 0, NULL };

// ── BLE state ──────────────────────────────────────────────────────────
static NimBLEScan* pScan = nullptr;
// Indexed by the last octet of the bd_addr — a cheap 256-bucket hash
// that's collision-free in practice for the small number of buttons in
// the rig.
static uint32_t lastPressMs[256] = { 0 };

struct BlePressEvent
{
  char bd_addr_str[18];  // "aa:bb:cc:dd:ee:ff\0"
  int8_t rssi;
  uint32_t local_us;  // micros() at detection, used for epoch conversion
};
static QueueHandle_t blePressQueue = nullptr;

// ── HW latency tester state ────────────────────────────────────────────
volatile uint32_t hwPressTimeUs = 0;
volatile bool hwPressPending = false;

static long latencyMeasurements[MAX_MEASUREMENTS];
static int measurementCount = 0;

// ── micro-ROS entities ────────────────────────────────────────────────
rcl_publisher_t ble_press_publisher;
rcl_publisher_t hw_press_publisher;
rcl_publisher_t log_publisher;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

std_msgs__msg__Header ble_press_msg;
std_msgs__msg__Header hw_press_msg;
std_msgs__msg__String log_msg;

uint8_t agent_sync_retries;

enum agent_states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
  UNRECOVERABLE_ERROR,
} agent_state;

// ── Macros (adapted from tabletop_teensy) ──────────────────────────────
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
#define NS_TO_ROS_TIME(time_msg, ns)                                                                                   \
  {                                                                                                                    \
    (time_msg).sec = (int32_t)((ns) / 1000000000LL);                                                                   \
    (time_msg).nanosec = (uint32_t)((ns) % 1000000000LL);                                                              \
  }
#define EXECUTE_EVERY_N_MS(MS, X)                                                                                      \
  {                                                                                                                    \
    static int64_t init = -1;                                                                                          \
    if (init == -1)                                                                                                    \
    {                                                                                                                  \
      init = millis();                                                                                                 \
    }                                                                                                                  \
    if ((int64_t)millis() - init > (MS))                                                                               \
    {                                                                                                                  \
      X;                                                                                                               \
      init = millis();                                                                                                 \
    }                                                                                                                  \
  }

// ── BLE helpers (carried over from main.ino) ───────────────────────────
static bool isFlicAddress(const NimBLEAddress& addr)
{
  std::string s = addr.toString();
  return (s.find("80:e4:da") == 0 || s.find("90:88:a9") == 0);
}

static uint8_t lastOctet(const NimBLEAddress& addr)
{
  std::string s = addr.toString();
  return (uint8_t)strtoul(s.substr(15, 2).c_str(), NULL, 16);
}

// ── Hardware sync ISR ──────────────────────────────────────────────────
// IRAM_ATTR: ESP32 ISRs must be in IRAM. Keep this lean — no calls into
// micro-ROS or anything that isn't IRAM-safe.
void IRAM_ATTR onHardwareButtonPress()
{
  uint32_t now = micros();
  if (now - hwPressTimeUs > HW_BUTTON_DEBOUNCE_US)
  {
    hwPressTimeUs = now;
    hwPressPending = true;
  }
}

// ── BLE scan callback (runs on the NimBLE host task) ───────────────────
class ScanCallbacks : public NimBLEScanCallbacks
{
  void onResult(const NimBLEAdvertisedDevice* device) override
  {
    NimBLEAddress addr = device->getAddress();
    if (!isFlicAddress(addr))
      return;

    uint8_t key = lastOctet(addr);
    uint32_t now_ms = millis();
    if ((now_ms - lastPressMs[key]) < PER_BUTTON_COOLDOWN_MS)
      return;
    lastPressMs[key] = now_ms;

    // Capture the detection time before doing any string work so the
    // measurement reflects the BLE arrival, not the queue push.
    uint32_t local_us = micros();

    BlePressEvent ev = {};
    std::string s = addr.toString();
    strncpy(ev.bd_addr_str, s.c_str(), sizeof(ev.bd_addr_str) - 1);
    ev.rssi = device->getRSSI();
    ev.local_us = local_us;

    // Non-blocking send. If the queue is full (agent down or main loop
    // wedged), drop the event rather than back-pressuring the BLE task.
    xQueueSend(blePressQueue, &ev, 0);
  }
};

// ── Publishing (runs on the main loop) ────────────────────────────────
// Convert a captured micros() timestamp to ROS epoch ns by anchoring
// against a fresh rmw_uros_epoch_nanos() reading. This avoids reading
// the 64-bit time offset from the BLE task (where a concurrent sync
// could tear it on this 32-bit MCU).
static int64_t local_us_to_epoch_ns(uint32_t local_us)
{
  uint32_t now_us = micros();
  int64_t now_ns = rmw_uros_epoch_nanos();
  // (now_us - local_us) is correct under unsigned wrap as long as the
  // gap is < ~70 minutes; queue drains every loop pass so this holds.
  int64_t age_ns = (int64_t)(now_us - local_us) * 1000LL;
  return now_ns - age_ns;
}

static void publish_ble_press(const BlePressEvent& ev)
{
  int64_t epoch_ns = local_us_to_epoch_ns(ev.local_us);
  NS_TO_ROS_TIME(ble_press_msg.stamp, epoch_ns);
  STRING_SET(&ble_press_msg.frame_id, "%s", ev.bd_addr_str);
  RCSOFTCHECK(rcl_publish(&ble_press_publisher, &ble_press_msg, NULL));
}

static void publish_hw_press(uint32_t hw_us)
{
  int64_t epoch_ns = local_us_to_epoch_ns(hw_us);
  NS_TO_ROS_TIME(hw_press_msg.stamp, epoch_ns);
  STRING_SET(&hw_press_msg.frame_id, "hw_sync");
  RCSOFTCHECK(rcl_publish(&hw_press_publisher, &hw_press_msg, NULL));
}

static void record_latency_stats(long latency_us)
{
  latencyMeasurements[measurementCount++] = latency_us;
  if (measurementCount < MAX_MEASUREMENTS)
    return;

  long sum = 0;
  for (int i = 0; i < MAX_MEASUREMENTS; i++)
    sum += latencyMeasurements[i];
  float avg = (float)sum / MAX_MEASUREMENTS;

  float var = 0;
  for (int i = 0; i < MAX_MEASUREMENTS; i++)
  {
    float d = (float)latencyMeasurements[i] - avg;
    var += d * d;
  }
  float stddev = sqrtf(var / MAX_MEASUREMENTS);

  LOG("HW->BLE stats (n=%d): avg=%.2f ms, stddev=%.2f ms", MAX_MEASUREMENTS, avg / 1000.0f, stddev / 1000.0f);
  measurementCount = 0;
}

static void pump_publish_queue()
{
  // Drain BLE events. For each one, also check if there's a pending HW
  // press whose latency we should record. The "match the next BLE press
  // to the most recent HW press" semantics are preserved from the .ino.
  BlePressEvent ev;
  while (xQueueReceive(blePressQueue, &ev, 0) == pdTRUE)
  {
    publish_ble_press(ev);

    if (hwPressPending)
    {
      uint32_t hw_us = hwPressTimeUs;
      hwPressPending = false;
      long latency_us = (long)(ev.local_us - hw_us);
      LOG("HW->BLE latency: %ld us  (bd_addr=%s)", latency_us, ev.bd_addr_str);
      record_latency_stats(latency_us);
    }
  }

  // Publish any HW press that didn't get paired with a BLE event (e.g.
  // the user only pressed the hardware sync). The Python side can still
  // correlate it against advertisements seen by the scapy client.
  if (hwPressPending)
  {
    uint32_t hw_us = hwPressTimeUs;
    hwPressPending = false;
    publish_hw_press(hw_us);
  }
}

// ── BLE init (called once at boot) ─────────────────────────────────────
static void init_ble()
{
  NimBLEDevice::init("FlicSniffer");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks(), true);
  pScan->setActiveScan(false);
  pScan->setInterval(16);  // 0.625 ms units → ~10 ms
  pScan->setWindow(16);    // 100% duty cycle
  pScan->setDuplicateFilter(false);
  pScan->setMaxResults(0);
  pScan->start(0, false);
}

// ── micro-ROS client lifecycle ────────────────────────────────────────
bool init_client()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NS, &support));

  RCCHECK(rclc_publisher_init_best_effort(&ble_press_publisher, &node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), BLE_PRESS_TOPIC));
  RCCHECK(rclc_publisher_init_best_effort(&hw_press_publisher, &node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), HW_PRESS_TOPIC));
  RCCHECK(rclc_publisher_init_default(&log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                      LOG_TOPIC));

  // No timers, services, or subscriptions on this node — but rclc still
  // wants a non-zero handle count for spin_some to be useful, so size 1.
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK(rmw_uros_sync_session(1000));
  LOG("BLE sniffer ready (cooldown=%lu ms, queue=%u)", (unsigned long)PER_BUTTON_COOLDOWN_MS,
      (unsigned)BLE_PRESS_QUEUE_LEN);
  return true;
}

bool deinit_client()
{
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  RCCHECK(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  RCCHECK(rcl_publisher_fini(&ble_press_publisher, &node));
  RCCHECK(rcl_publisher_fini(&hw_press_publisher, &node));
  RCCHECK(rcl_publisher_fini(&log_publisher, &node));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));

  printf("Client deinitialized\n");
  return true;
}

// ── Arduino entry points ──────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  // No prints to Serial after this point — it's owned by the transport.

  WiFi.mode(WIFI_OFF);

  pinMode(HW_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HW_BUTTON_PIN), onHardwareButtonPress, FALLING);

  // Pre-allocate string memory for the three messages we publish.
  bool ok = micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
                                                      &ble_press_msg, memory_conf);
  ok &= micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), &hw_press_msg,
                                                  memory_conf);
  ok &= micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), &log_msg,
                                                  memory_conf);

  blePressQueue = xQueueCreate(BLE_PRESS_QUEUE_LEN, sizeof(BlePressEvent));
  ok &= (blePressQueue != nullptr);

  // BLE runs independently of the agent; bring it up once here so we're
  // collecting events even before the first micro-ROS connection.
  init_ble();

  agent_state = ok ? WAITING_AGENT : UNRECOVERABLE_ERROR;
  delay(500);
}

void loop()
{
  switch (agent_state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(AGENT_RECONNECT_PERIOD_MS,
                         agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_RECONNECT_TIMEOUT_MS, 1)) ?
                                           AGENT_AVAILABLE :
                                           WAITING_AGENT;);
      // Drop queued events while disconnected so we don't republish
      // stale timestamps on reconnect.
      xQueueReset(blePressQueue);
      hwPressPending = false;
      break;

    case AGENT_AVAILABLE:
      agent_state = init_client() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(AGENT_SYNC_PERIOD_MS,
                         agent_sync_retries = (RMW_RET_OK == rmw_uros_sync_session(AGENT_SYNC_TIMEOUT_MS)) ?
                                                  0 :
                                                  agent_sync_retries + 1;);
      if ((agent_sync_retries >= AGENT_SYNC_MAX_RETRIES) ||
          (RCL_RET_OK != rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_TIMEOUT_MS))))
      {
        agent_state = AGENT_DISCONNECTED;
        break;
      }
      pump_publish_queue();
      break;

    case AGENT_DISCONNECTED:
      agent_state = deinit_client() ? WAITING_AGENT : UNRECOVERABLE_ERROR;
      break;

    case UNRECOVERABLE_ERROR:
      delay(100);
      break;
  }
}
