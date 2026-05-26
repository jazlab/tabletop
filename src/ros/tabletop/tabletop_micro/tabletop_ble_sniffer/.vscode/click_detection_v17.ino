/*
 * Flic 2 Button Press Detector (Ultra-Low Latency Mode)
 * Includes Hardware Sync Tester on Pin 13
 * Calculates Average & Standard Deviation every 10 presses
 */

#include <NimBLEDevice.h>
#include <WiFi.h>
#include <math.h>  // Required for sqrt() and pow() functions

// ── Configuration ──────────────────────────────────────────────────────
static const uint32_t CONNECT_DELAY_MS = 1000;
static const uint32_t CONNECT_TIMEOUT_MS = 500;
static const uint32_t PER_BUTTON_COOLDOWN_MS = 500;

// Hardware Latency Tester Pin
const int HW_BUTTON_PIN = 13;

// ── State ──────────────────────────────────────────────────────────────
static NimBLEScan* pScan = nullptr;
static uint32_t lastPressTime[256] = { 0 };

static NimBLEAddress connectTarget;
static volatile bool connectRequested = false;
static volatile uint8_t connectTargetKey = 0;

// Tracking the latency test
volatile unsigned long hwPressTimeUs = 0;
volatile bool hwPressPending = false;

// Variables to hold data so we can print it AFTER the radio is done
std::string printMac;
int printRSSI;
uint32_t printStampMs;
uint32_t printStampUs;
long printLatency;
bool printHasLatencyInfo = false;

// ── Statistical Tracking ───────────────────────────────────────────────
const int MAX_MEASUREMENTS = 10;
long latencyMeasurements[MAX_MEASUREMENTS];
int measurementCount = 0;

// ── Hardware Interrupt ─────────────────────────────────────────────────
void IRAM_ATTR onHardwareButtonPress()
{
  unsigned long now = micros();
  if (now - hwPressTimeUs > 500000)
  {
    hwPressTimeUs = now;
    hwPressPending = true;
  }
}

// ── Helpers ────────────────────────────────────────────────────────────
bool isFlicAddress(const NimBLEAddress& addr)
{
  std::string s = addr.toString();
  return (s.find("80:e4:da") == 0 || s.find("90:88:a9") == 0);
}

uint8_t lastOctet(const NimBLEAddress& addr)
{
  std::string s = addr.toString();
  return (uint8_t)strtoul(s.substr(15, 2).c_str(), NULL, 16);
}

// ── Scan callback (STRIPPED FOR MAXIMUM SPEED) ─────────────────────────
class ScanCallbacks : public NimBLEScanCallbacks
{
  void onResult(const NimBLEAdvertisedDevice* device) override
  {
    NimBLEAddress addr = device->getAddress();
    if (!isFlicAddress(addr))
      return;

    uint8_t key = lastOctet(addr);
    uint32_t now = millis();

    if ((now - lastPressTime[key]) < PER_BUTTON_COOLDOWN_MS)
      return;

    // ── 1. GRAB TIME INSTANTLY ──
    lastPressTime[key] = now;
    uint32_t stampUs = micros();
    uint32_t stampMs = now;

    // ── 2. SAVE DATA TO MEMORY ──
    printMac = addr.toString();
    printRSSI = device->getRSSI();
    printStampMs = stampMs;
    printStampUs = stampUs;

    // ── 3. CALCULATE LATENCY INSTANTLY ──
    if (hwPressPending)
    {
      printLatency = stampUs - hwPressTimeUs;  // microseconds
      printHasLatencyInfo = true;
      hwPressPending = false;
    }
    else
    {
      printHasLatencyInfo = false;
    }

    // ── 4. FLAG MAIN LOOP AND EXIT CALLBACK ──
    connectTarget = addr;
    connectTargetKey = key;
    connectRequested = true;
  }
};

// ── Connect & disconnect ───────────────────────────────────────────────
void resetButtonAds(const NimBLEAddress& addr)
{
  pScan->stop();
  delay(CONNECT_DELAY_MS);

  Serial.printf("  -> Connecting to %s to reset ads...\n", addr.toString().c_str());

  NimBLEClient* pClient = NimBLEDevice::createClient();
  bool connected = pClient->connect(addr, false);

  if (connected)
  {
    Serial.println("  -> Connected. Holding briefly to silence button ads...");
    delay(200);
    Serial.println("  -> Disconnecting...");
    pClient->disconnect();
  }
  else
  {
    Serial.println("  -> Connect failed (button may have stopped advertising).");
  }

  NimBLEDevice::deleteClient(pClient);
  lastPressTime[connectTargetKey] = millis();
  pScan->start(0, false);
  Serial.println("  -> Scanning resumed.\n");
}

// ── Arduino setup & loop ───────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  WiFi.mode(WIFI_OFF);

  pinMode(HW_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HW_BUTTON_PIN), onHardwareButtonPress, FALLING);

  Serial.println("\n=== Flic 2 BLE Press Detector (Statistical Mode) ===");
  Serial.println("Smash both buttons at the same time to measure delay!");
  Serial.println("Initializing BLE...");

  NimBLEDevice::init("FlicDetector");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks(), true);

  pScan->setActiveScan(false);

  pScan->setInterval(16);  // ~10ms scan interval
  pScan->setWindow(16);    // 100% duty cycle at shorter granularity

  pScan->setDuplicateFilter(false);
  pScan->setMaxResults(0);  // Don't buffer results, deliver immediately

  Serial.println("Starting continuous BLE scan...");
  Serial.println("Waiting for Flic button presses...\n");

  pScan->start(0, false);
}

void loop()
{
  if (connectRequested)
  {
    Serial.printf("[PRESS] t=%lu ms  (%lu us)  addr=%s  RSSI=%d\n", printStampMs, printStampUs, printMac.c_str(),
                  printRSSI);

    if (printHasLatencyInfo)
    {
      Serial.printf("        ---> LATENCY: %.2f ms \n", printLatency / 1000.0);

      // Add the valid latency measurement to our array
      latencyMeasurements[measurementCount] = printLatency;
      measurementCount++;

      // Once we hit 10 valid measurements, crunch the numbers
      if (measurementCount >= MAX_MEASUREMENTS)
      {
        // 1. Calculate the Average (Mean)
        long sum = 0;
        for (int i = 0; i < MAX_MEASUREMENTS; i++)
        {
          sum += latencyMeasurements[i];
        }
        float average = (float)sum / MAX_MEASUREMENTS;

        // 2. Calculate the Standard Deviation
        float varianceSum = 0;
        for (int i = 0; i < MAX_MEASUREMENTS; i++)
        {
          varianceSum += pow(latencyMeasurements[i] - average, 2);
        }
        float stdDev = sqrt(varianceSum / MAX_MEASUREMENTS);

        // 3. Print the results (convert from us to ms)
        Serial.println("\n=================================================");
        Serial.printf("  STATISTICS (Last %d presses):\n", MAX_MEASUREMENTS);
        Serial.printf("  Average Latency: %.2f ms\n", average / 1000.0);
        Serial.printf("  Standard Deviation: %.2f ms\n", stdDev / 1000.0);
        Serial.println("=================================================\n");

        // 4. Reset the counter for the next batch
        measurementCount = 0;
      }
    }
    else
    {
      Serial.println("        ---> (No physical sync click detected prior to BLE packet)");
    }

    connectRequested = false;
    NimBLEAddress target = connectTarget;
    resetButtonAds(target);
  }

  delay(1);
}
