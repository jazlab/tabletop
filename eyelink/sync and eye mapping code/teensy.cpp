// Definitions for LED control
#define LED_PIN         2     // The pin where the LED is connected
#define BASE_INTERVAL   1000  // Base interval for LED blinking in milliseconds
#define NOISE_RANGE     200   // Range of random variation in the blink interval (+/- 200 ms)
#define BLINK_DURATION  100   // Duration the LED stays on during a blink in milliseconds

// Set up the board and serial communication
void setup() {
  pinMode(LED_PIN, OUTPUT);  // Configure the LED pin as an output
  Serial.begin(115200);      // Initialize serial communication at 115200 bps for debugging
}

// Main program loop: blink the LED at random intervals
void loop() {
  // Calculate the total interval for this loop iteration with added randomness
  long interval = BASE_INTERVAL + random(-NOISE_RANGE, NOISE_RANGE);

  // Turn the LED on and send its state over serial
  digitalWrite(LED_PIN, HIGH);
  sendStateToSerial("ON");

  // Keep the LED on for the duration of BLINK_DURATION
  delay(BLINK_DURATION);

  // Turn the LED off and send its state over serial
  digitalWrite(LED_PIN, LOW);
  sendStateToSerial("OFF");

  // Wait for the remainder of the interval before the next loop iteration
  delay(interval - BLINK_DURATION);
}

// Send the current state of the LED to the serial monitor with a timestamp
void sendStateToSerial(const char* state) {
  unsigned long timeStamp = millis();  // Current time in milliseconds since the program started
  Serial.print(timeStamp);             // Send the timestamp
  Serial.print(", ");
  Serial.println(state);               // Send the state of the LED
}
