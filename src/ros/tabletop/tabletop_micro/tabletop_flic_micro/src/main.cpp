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
//   - To absorb the Flic's ~3 s post-press advertisement burst, two
//     mechanisms are used: a per-button cooldown (lastOctet hash key)
//     and resetButtonAds(), which pump_publish_queue() calls after each
//     press to connect-then-immediately-disconnect the button.
//     WARNING: that sync runs on the main loop and can block for up to
//     AGENT_SYNC_TIMEOUT_MS, which may stall executor spin and time-sync
//     and cause the micro-ROS agent to disconnect (see docs/known-issues.md).
//     Moving resetButtonAds() to a dedicated FreeRTOS task fed by a separate
//     queue would keep the main loop responsive.

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
#include <tabletop_interfaces/srv/ping.h>

#include "rmw_microros/time_sync.h"

#ifndef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
#error This code only supports serial transport.
#endif

// ── Configuration ──────────────────────────────────────────────────────
// Hardware sync button (active-low).
#define HW_BUTTON_PIN 12
#define HW_BUTTON_DEBOUNCE_US 500000  // 500 ms

// Per-button cooldown. Sized to absorb a single Flic press' adv burst
// without firing multiple events. Shorter → risk of duplicate events
// per press; longer → rapid presses get coalesced.
#define PER_BUTTON_COOLDOWN_MS 1000

// Bounded queue of press events from BLE callback → main loop.
#define BLE_PRESS_QUEUE_LEN 16

// Latency stats batch size (matches the .ino).
#define MAX_MEASUREMENTS 10

// micro-ROS topology
#define NODE_NAME "flic"
#define NODE_NS ""
#define BLE_PRESS_TOPIC "~/button_pressed_time"
#define HW_PRESS_TOPIC "~/hw_sync_pressed_time"
#define LOG_TOPIC "~/log"
#define PING_SRV_NAME "~/ping"

// Agent-supervision timings (mirrors tabletop_teensy).
#define AGENT_RECONNECT_PERIOD_MS 100
#define AGENT_RECONNECT_TIMEOUT_MS 20
#define EXECUTOR_SPIN_TIMEOUT_MS 1
#define AGENT_SYNC_PERIOD_MS 10000
#define AGENT_SYNC_TIMEOUT_MS 10
#define AGENT_SYNC_MAX_RETRIES 3
#define BLINK_CONNECTED_PERIOD_MS 500

// Message memory configuration (used to back the Header.frame_id /
// String.data fields).
static const micro_ros_utilities_memory_conf_t memory_conf = { 100, 5, 5, NULL, 0, NULL };

// ── BLE state ──────────────────────────────────────────────────────────
static NimBLEScan* pScan = nullptr;
static NimBLEClient* pClient = nullptr;
// Indexed by the last octet of the bd_addr — a cheap 256-bucket hash
// that's collision-free in practice for the small number of buttons in
// the rig.
static uint32_t lastPressMs[256] = { 0 };

struct BlePressEvent
{
  // char bd_addr_str[18];  // "aa:bb:cc:dd:ee:ff\0"
  NimBLEAddress addr;
  int8_t rssi;
  uint32_t local_us;  // micros() at detection, used for epoch conversion
};
static QueueHandle_t blePressQueue = nullptr;

// ── HW latency tester state ────────────────────────────────────────────
// volatile uint32_t hwPressTimeUs = 0;
// volatile bool hwPressPending = false;
//
// static long latencyMeasurements[MAX_MEASUREMENTS];
// static int measurementCount = 0;

// ── micro-ROS entities ────────────────────────────────────────────────
rcl_publisher_t ble_press_publisher;
rcl_publisher_t hw_press_publisher;
rcl_publisher_t log_publisher;
rcl_service_t ping_service;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

std_msgs__msg__Header ble_press_msg;
std_msgs__msg__Header hw_press_msg;
std_msgs__msg__String log_msg;
tabletop_interfaces__srv__Ping_Request ping_request;
tabletop_interfaces__srv__Ping_Response ping_response;

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
#define BOOLCHECK(fn)                                                                                                  \
  {                                                                                                                    \
    bool temp_ret = fn;                                                                                                \
    if (!temp_ret)                                                                                                     \
    {                                                                                                                  \
      return false;                                                                                                    \
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
#define GET_CURRENT_ROS_TIME(time_msg)                                                                                 \
  {                                                                                                                    \
    int64_t now_ns = rmw_uros_epoch_nanos();                                                                           \
    NS_TO_ROS_TIME(time_msg, now_ns);                                                                                  \
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
// Returns true if addr matches a known Flic button OUI prefix
// (80:e4:da for older hardware, 90:88:a9 for newer).
static bool isFlicAddress(const NimBLEAddress& addr)
{
  std::string s = addr.toString();
  return (s.find("80:e4:da") == 0 || s.find("90:88:a9") == 0);
}

// Extracts the final byte of a BLE MAC address as an 8-bit index into
// lastPressMs[]. Acts as a cheap per-button hash key.
static uint8_t lastOctet(const NimBLEAddress& addr)
{
  std::string s = addr.toString();
  return (uint8_t)strtoul(s.substr(15, 2).c_str(), NULL, 16);
}

// ── Hardware sync ISR ──────────────────────────────────────────────────
// IRAM_ATTR: ESP32 ISRs must be in IRAM. Keep this lean — no calls into
// micro-ROS or anything that isn't IRAM-safe.
// void ARDUINO_ISR_ATTR onHardwareButtonPress()
// {
//   uint32_t now = micros();
//   if (now - hwPressTimeUs > HW_BUTTON_DEBOUNCE_US)
//   {
//     hwPressTimeUs = now;
//     hwPressPending = true;
//   }
// }

// ── BLE scan callback (runs on the NimBLE host task) ───────────────────
// NimBLE calls onResult() for every advertisement packet seen during the
// continuous passive scan. The callback filters to Flic addresses, enforces
// the per-button cooldown, and pushes qualifying events to blePressQueue for
// the main loop to publish. All heavy work (micro-ROS, string formatting)
// is deferred to the main loop to keep this callback fast and ISR-safe.
class ScanCallbacks : public NimBLEScanCallbacks
{
  void onResult(const NimBLEAdvertisedDevice* device) override
  {
    // Capture the detection time before doing any work so the
    // measurement reflects the BLE arrival, not the queue push.
    uint32_t local_us = micros();

    NimBLEAddress addr = device->getAddress();
    if (!isFlicAddress(addr))
      return;

    uint8_t key = lastOctet(addr);
    uint32_t now_ms = millis();
    if ((now_ms - lastPressMs[key]) < PER_BUTTON_COOLDOWN_MS)
      return;
    lastPressMs[key] = now_ms;

    BlePressEvent ev = {};
    ev.addr = addr;
    ev.rssi = device->getRSSI();
    ev.local_us = local_us;

    // Non-blocking send. If the queue is full (agent down or main loop
    // wedged), drop the event rather than back-pressuring the BLE task.
    xQueueSend(blePressQueue, &ev, 0);
  }
} scan_callbacks;

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

// Converts a queued BLE press event to a std_msgs/Header and publishes it.
// The Header stamp is computed via local_us_to_epoch_ns() so it reflects the
// BLE detection time rather than the publish time.
static void publish_ble_press(const BlePressEvent& ev)
{
  int64_t epoch_ns = local_us_to_epoch_ns(ev.local_us);
  NS_TO_ROS_TIME(ble_press_msg.stamp, epoch_ns);
  STRING_SET(&ble_press_msg.frame_id, "%s", ev.addr.toString().c_str());
  RCSOFTCHECK(rcl_publish(&ble_press_publisher, &ble_press_msg, NULL));
}

// micro-ROS service handler for ~/ping. Returns the current ROS epoch time
// and sets success=true if the time sync is live (non-zero timestamp).
void ping_callback(const void* req, void* res)
{
  RCLC_UNUSED(req);
  tabletop_interfaces__srv__Ping_Response* response = static_cast<tabletop_interfaces__srv__Ping_Response*>(res);

  GET_CURRENT_ROS_TIME(response->received_time);
  response->success = (response->received_time.sec != 0) || (response->received_time.nanosec != 0);
}

// static void publish_hw_press(uint32_t hw_us)
// {
//   int64_t epoch_ns = local_us_to_epoch_ns(hw_us);
//   NS_TO_ROS_TIME(hw_press_msg.stamp, epoch_ns);
//   STRING_SET(&hw_press_msg.frame_id, "hw_sync");
//   RCSOFTCHECK(rcl_publish(&hw_press_publisher, &hw_press_msg, NULL));
// }

// static void record_latency_stats(long latency_us)
// {
//   latencyMeasurements[measurementCount++] = latency_us;
//   if (measurementCount < MAX_MEASUREMENTS)
//     return;
//
//   long sum = 0;
//   for (int i = 0; i < MAX_MEASUREMENTS; i++)
//     sum += latencyMeasurements[i];
//   float avg = (float)sum / MAX_MEASUREMENTS;
//
//   float var = 0;
//   for (int i = 0; i < MAX_MEASUREMENTS; i++)
//   {
//     float d = (float)latencyMeasurements[i] - avg;
//     var += d * d;
//   }
//   float stddev = sqrtf(var / MAX_MEASUREMENTS);
//
//   LOG("HW->BLE stats (n=%d): avg=%.2f ms, stddev=%.2f ms", MAX_MEASUREMENTS, avg / 1000.0f, stddev / 1000.0f);
//   measurementCount = 0;
// }

// ── Connect & disconnect ───────────────────────────────────────────────
// Briefly connects to a Flic button and immediately disconnects to silence
// its post-press advertisement burst. Called from pump_publish_queue() after
// each BLE press event.
// NOTE: this runs on the main loop (not a timer), so it stalls executor
// spin while it connects — this may cause timing jitter or agent
// disconnects if the connect takes >~1 s (see docs/known-issues.md).
static void resetButtonAds(const NimBLEAddress& addr)
{
  // delay(CONNECT_DELAY_MS);

  LOG("Connecting to %s to reset ads...", addr.toString().c_str());

  bool connected = pClient->connect(addr, false);

  if (connected)
  {
    LOG("Connected. Immediately disconnecting...");
    pClient->disconnect();
  }
  else
  {
    LOG("Connect failed (button may have stopped advertising).");
  }
}

// Drains blePressQueue and publishes each event. For each event, stops the
// scan, calls resetButtonAds() to silence the button's advertisement burst,
// then restarts the scan after draining all pending events.
static void pump_publish_queue()
{
  // Drain BLE events. For each one, also check if there's a pending HW
  // press whose latency we should record. The "match the next BLE press
  // to the most recent HW press" semantics are preserved from the .ino.
  BlePressEvent ev;
  bool scan_stopped = false;
  while (xQueueReceive(blePressQueue, &ev, 0) == pdTRUE)
  {
    publish_ble_press(ev);
    if (!scan_stopped)
    {
      pScan->stop();
      scan_stopped = true;
    }
    resetButtonAds(ev.addr);

    // if (hwPressPending)
    // {
    //   uint32_t hw_us = hwPressTimeUs;
    //   hwPressPending = false;
    //   long latency_us = (long)(ev.local_us - hw_us);
    //   LOG("HW->BLE latency: %ld us  (bd_addr=%s)", latency_us, ev.bd_addr_str);
    //   record_latency_stats(latency_us);
    // }
  }
  if (scan_stopped)
  {
    pScan->start(0, false);
  }

  // Publish any HW press that didn't get paired with a BLE event (e.g.
  // the user only pressed the hardware sync). The Python side can still
  // correlate it against advertisements seen by the scapy client.
  // if (hwPressPending)
  // {
  //   uint32_t hw_us = hwPressTimeUs;
  //   hwPressPending = false;
  //   publish_hw_press(hw_us);
  // }
}

// ── BLE init (called once at boot) ─────────────────────────────────────
// Initializes NimBLE with device name "FlicSniffer", configures a passive
// continuous scan at 100% duty cycle (interval==window==16 * 0.625 ms = 10 ms),
// disables duplicate filtering so every advertisement is delivered to the
// callback, and creates a NimBLEClient used by resetButtonAds(). Returns false
// on any NimBLE initialization failure.
static bool init_ble()
{
  BOOLCHECK(NimBLEDevice::init("FlicSniffer"));
  BOOLCHECK(NimBLEDevice::setPower(ESP_PWR_LVL_P9));

  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scan_callbacks, true);
  pScan->setActiveScan(false);
  pScan->setInterval(16);  // 0.625 ms units → ~10 ms
  pScan->setWindow(16);    // 100% duty cycle
  pScan->setDuplicateFilter(false);
  pScan->setMaxResults(0);

  pClient = NimBLEDevice::createClient();

  return true;
}

// ── micro-ROS client lifecycle ────────────────────────────────────────
// Creates the micro-ROS node, publishers (BLE press, HW sync, log), the ping
// service, and the rclc executor. Performs an initial time sync, resets the
// BLE press queue to discard any stale events accumulated while disconnected,
// and starts the BLE scan. Returns false on any rcl/rclc failure.
bool init_client()
{
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NS, &support));

  RCCHECK(rclc_publisher_init_default(&ble_press_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
                                      BLE_PRESS_TOPIC));
  RCCHECK(rclc_publisher_init_default(&hw_press_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
                                      HW_PRESS_TOPIC));
  RCCHECK(rclc_publisher_init_default(&log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                      LOG_TOPIC));
  RCCHECK(rclc_service_init_default(&ping_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(tabletop_interfaces, srv, Ping),
                                    PING_SRV_NAME));

  // No timers, services, or subscriptions on this node — but rclc still
  // wants a non-zero handle count for spin_some to be useful, so size 1.
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &ping_service, &ping_request, &ping_response, ping_callback));

  RCCHECK(rmw_uros_sync_session(1000));

  xQueueReset(blePressQueue);

  BOOLCHECK(pScan->start(0, false));

  LOG("BLE sniffer ready (cooldown=%lu ms, queue=%u)", (unsigned long)PER_BUTTON_COOLDOWN_MS,
      (unsigned)BLE_PRESS_QUEUE_LEN);
  return true;
}

// Tears down the micro-ROS node, publishers, service, executor, and support.
// Stops the BLE scan first; sets the rmw destroy timeout to 0 for a fast
// teardown. Returns false on any rcl failure. Called on agent disconnect
// before re-entering WAITING_AGENT.
bool deinit_client()
{
  pScan->stop();

  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  RCCHECK(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  RCCHECK(rcl_publisher_fini(&ble_press_publisher, &node));
  RCCHECK(rcl_publisher_fini(&hw_press_publisher, &node));
  RCCHECK(rcl_publisher_fini(&log_publisher, &node));
  RCCHECK(rcl_service_fini(&ping_service, &node));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));

  printf("Client deinitialized\n");
  return true;
}

// ── Arduino entry points ──────────────────────────────────────────────
// Initializes serial (115 200 baud, handed to micro-ROS transport), disables
// Wi-Fi to avoid RF interference with BLE, allocates message memory for the
// three published message types, creates blePressQueue, and brings up BLE.
// Sets agent_state to WAITING_AGENT on success or UNRECOVERABLE_ERROR if
// any allocation/initialization step fails.
void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  // No prints to Serial after this point — it's owned by the transport.

  WiFi.mode(WIFI_OFF);

  // pinMode(HW_BUTTON_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(HW_BUTTON_PIN), onHardwareButtonPress, FALLING);

  // Pre-allocate string memory for the three messages we publish.
  bool ok = micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
                                                      &ble_press_msg, memory_conf);
  ok &= micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), &hw_press_msg,
                                                  memory_conf);
  ok &= micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), &log_msg,
                                                  memory_conf);

  blePressQueue = xQueueCreate(BLE_PRESS_QUEUE_LEN, sizeof(BlePressEvent));
  // bleConnectQueue = xQueueCreate(BLE_PRESS_QUEUE_LEN, sizeof(BlePressEvent));
  ok &= (blePressQueue != nullptr);

  // BLE runs independently of the agent; bring it up once here so we're
  // collecting events even before the first micro-ROS connection.
  ok &= init_ble();

  agent_sync_retries = 0;
  agent_state = ok ? WAITING_AGENT : UNRECOVERABLE_ERROR;
  delay(500);
}

// Main state machine. States:
//   WAITING_AGENT      — polls for the micro-ROS agent every AGENT_RECONNECT_PERIOD_MS.
//   AGENT_AVAILABLE    — agent detected; calls init_client() to create ROS entities.
//   AGENT_CONNECTED    — normal operation: re-syncs clock every AGENT_SYNC_PERIOD_MS,
//                        spins the rclc executor, and drains the BLE press queue via
//                        pump_publish_queue(). Falls to AGENT_DISCONNECTED if time-sync
//                        fails AGENT_SYNC_MAX_RETRIES consecutive times.
//   AGENT_DISCONNECTED — tears down ROS entities via deinit_client(), then re-enters
//                        WAITING_AGENT.
//   UNRECOVERABLE_ERROR — spins indefinitely with a 100 ms delay; requires reboot.
void loop()
{
  switch (agent_state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(AGENT_RECONNECT_PERIOD_MS,
                         agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_RECONNECT_TIMEOUT_MS, 1)) ?
                                           AGENT_AVAILABLE :
                                           WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      agent_state = init_client() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(AGENT_SYNC_PERIOD_MS,
                         agent_sync_retries = (RMW_RET_OK == rmw_uros_sync_session(AGENT_SYNC_TIMEOUT_MS)) ?
                                                  0 :
                                                  agent_sync_retries + 1;);
      if (agent_sync_retries >= AGENT_SYNC_MAX_RETRIES)
      {
        agent_sync_retries = 0;
        agent_state = AGENT_DISCONNECTED;
        break;
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_SPIN_TIMEOUT_MS));
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
