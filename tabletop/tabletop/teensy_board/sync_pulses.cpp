/*
 * Sync Pulse Generator
 *
 * This program generates TTL pulses that are sent to a variety of data streams
 * for synchronization. The pulses are sent from a specified pin (PULSE_PIN)
 * and occur on a regular interval (BASE_INTERVAL). The pulses last for a set
 * duration (PULSE_DURATION). Jitter (NOISE_RANGE) is added to the base
 * interval so that the data streams can be easily aligned after the
 * experiment.
 *
 * The program sends the state of the pulse pin ("ON" or "OFF") to the serial
 * monitor with a timestamp every time the pin changes state.
 *
 * This code can be run on a Teensy board from the Arduino IDE.
 */

#define PULSE_PIN         2      // Pin connected to sync cable
#define BASE_INTERVAL   1000     // Base interval between sync pulses in ms
#define NOISE_RANGE     200      // Range of jitter in the base interval in ms
#define PULSE_DURATION  100      // Duration of each sync pulse in ms

// Set up the board and serial communication
void setup() {
  pinMode(PULSE_PIN, OUTPUT);    // Set pulse pin as output
  Serial.begin(115200);          // Start serial communication at 115200 bps
}

// Main program loop: send sync pulses at random intervals
void loop() {
  // Calculate total interval with added noise
  long interval = BASE_INTERVAL + random(-NOISE_RANGE / 2, NOISE_RANGE / 2);

  // Turn pulse pin ON and send state to serial
  digitalWrite(PULSE_PIN, HIGH);
  sendStateToSerial("ON");

  // Keep pulse pin ON for the pulse duration
  delay(PULSE_DURATION);

  // Turn pulse pin OFF and send state to serial
  digitalWrite(PULSE_PIN, LOW);
  sendStateToSerial("OFF");

  // Wait for the remainder of the interval before the next pulse
  delay(interval - PULSE_DURATION);
}

// Send pulse pin state and timestamp to serial monitor
void sendStateToSerial(const char* state) {
  unsigned long timeStamp = millis();  // Time in ms since program start
  Serial.print(timeStamp);             // Send timestamp
  Serial.print(", ");
  Serial.println(state);               // Send pulse pin state
}
