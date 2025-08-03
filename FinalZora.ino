#include <VoiceRecognitionV3.h>
#include <SoftwareSerial.h> // Keep SoftwareSerial for the VoiceRecognitionV3 module

// --- Voice Command Setup ---
/**
 * Connection:
 * Arduino Pin 2 ---> VoiceRecognitionModule TX
 * Arduino Pin 3 ---> VoiceRecognitionModule RX
 */
VR myVR(2,3);    // 2:RX 3:TX

// Buffer for voice recognition data
uint8_t buf[64];

// Voice command IDs //Use vr_sample_train to set
const uint8_t VOICE_HEY = 0; //Zora
const uint8_t VOICE_DANCE = 1; //Boogie
const uint8_t VOICE_STOP = 2; //Freeze
const uint8_t VOICE_TEST = 3; //Ping (Emergency Stop)
const uint8_t VOICE_CHECK = 10; // Check (Emergency Stop)


// --- Motor Driver Pin Definitions  ---
#define AIN1 13   // Motor 1 Input 1 (Direction)
#define AIN2 12   // Motor 1 Input 2 (Direction)
#define BIN1 8    // Motor 2 Input 1 (Direction)
#define BIN2 9    // Motor 2 Input 2 (Direction)

const int PWMA = 11;             // Speed control pin on the motor driver for Motor 1 (Right)
const int PWMB = 10;             // Speed control pin on the motor driver for Motor 2 (Left)


// --- 8M Recordable Sound Play Module Control ---
// This pin controls the 5V Relay Module, which in turn shorts the music module's bare wires.
#define RELAY_CONTROL_PIN A0


// --- LED Pin ---
#define LED_PIN 4 // Pin for the LED (Anode via 220 Ohm resistor, Cathode to GND)


// --- Sonar Sensor Pins (HC-SR04) ---
#define SONAR_TRIG_PIN 5
#define SONAR_ECHO_PIN 6


// --- Motor Control Functions ---
void setMotorSpeed(int motor, int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255); // Ensure speed is within valid range (-255 to 255)

  if (motor == 1)  // Motor 1 (Right)
  {
    if (motorSpeed > 0) // Forward
    { 
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    } else if (motorSpeed < 0) // Backward
    { 
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    } else // Stop
    { 
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
    }
    analogWrite(PWMA, abs(motorSpeed)); // Apply PWM to Motor 1
  } else if (motor == 2) // Motor 2 (Left)
  { 
    if (motorSpeed > 0) // Forward
    { 
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    } else if (motorSpeed < 0) // Backward
    { 
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    } else // Stop
    { 
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    }
    analogWrite(PWMB, abs(motorSpeed)); // Apply PWM to Motor 2
  }
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0); // Ensure PWM is 0 to truly stop
  analogWrite(PWMB, 0); // Ensure PWM is 0 to truly stop
}

// --- Dance Move Helper Functions ---
// 

// Twerk – Quick alternating turns.
void doTwerk(int speed = 100, int duration = 150, int repetitions = 5) 
{
  Serial.println("Wiggling!");
  for (int i = 0; i < repetitions; i++) 
  {
    // Turn left
    setMotorSpeed(1, speed);
    setMotorSpeed(2, -speed);
    delay(duration);
    stopMotors();
    delay(50); // Short pause

    // Turn right
    setMotorSpeed(1, -speed);
    setMotorSpeed(2, speed);
    delay(duration);
    stopMotors();
    delay(50); // Short pause
  }
}

// Spin – Turn in place.
void doSpin(int speed = 180, int duration = 1000) 
{
  Serial.println("Spinning!");
  // One motor forward, one motor backward
  setMotorSpeed(1, speed);
  setMotorSpeed(2, -speed);
  delay(duration);
  stopMotors();
}

// Forward-Back Pulse – Short pulses forward and back.
void doForwardBackPulse(int speed = 120, int duration = 200, int repetitions = 3) 
{
  Serial.println("Pulsing forward and back!");
  for (int i = 0; i < repetitions; i++) 
  {
    // Pulse forward
    setMotorSpeed(1, speed);
    setMotorSpeed(2, speed);
    delay(duration);
    stopMotors();
    delay(100); // Short pause

    // Pulse backward
    setMotorSpeed(1, -speed);
    setMotorSpeed(2, -speed);
    delay(duration);
    stopMotors();
    delay(100); // Short pause
  }
}


// --- Sonar Function ---
// Measures distance in centimeters
long readDistanceCM() 
{
  // Clears the SONAR_TRIG_PIN
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the SONAR_TRIG_PIN on HIGH state for 10 micro seconds
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);

  // Reads the SONAR_ECHO_PIN, returns the sound wave travel time in microseconds
  long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 10000); // 10ms timeout

  // Calculate the distance: Speed of sound is 0.034 cm/microsecond
  // Divide by 2 because the sound travels to the object and back.
  long distance = duration * 0.034 / 2;

  // If no echo, distance will be 0 or very large, indicating no object within range
  if (duration == 0) 
  {
      return -1; // Indicate no object detected within timeout
  }
  return distance;
}


// --- Function to pulse the music module using the relay ---
// This simulates touching/shorting the two bare wires on your music module
void pulseMusicToggle() 
{
  Serial.println("Activating relay to pulse music module...");
  // Most common 5V relay modules are active-LOW, meaning LOW turns them ON
  digitalWrite(RELAY_CONTROL_PIN, LOW);   // Activate the relay (close the switch)
  delay(100);                             // Hold the "short" for 100ms
  digitalWrite(RELAY_CONTROL_PIN, HIGH);  // Deactivate the relay (open the switch)
  Serial.println("Relay deactivated.");
  delay(200);                             // A small delay to ensure the module registers the release
}


// --- Lilybot Dance Routine ---
void lilybotDance() 
{
  Serial.println("Lilybot is dancing!");
  digitalWrite(LED_PIN, HIGH);           // Turn LED on for dancing!

  // Toggle music ON. This will activate the relay to "short" the wires.
  pulseMusicToggle();
  //musicIsOn = !musicIsOn; // Update our assumed state

//  if (musicIsOn) {
//      Serial.println("Music assumed ON.");
//  } else {
//      Serial.println("Music assumed OFF (unexpected for dance start, re-check manual music state).");
//  }
  delay(200); // Give music module a moment to start playing (longer because of relay activation time)

  // Add a quick check for obstacle before starting main movement
  long initialDistance = readDistanceCM();
  if (initialDistance != -1 && initialDistance < 20) {
    Serial.print("Obstacle detected at "); Serial.print(initialDistance); Serial.println(" cm. Adjusting dance start.");
    doSpin(150, 500); // Spin slightly to avoid immediate collision
  }

  // Example Dance Choreography - Customize this sequence!
  doForwardBackPulse(120, 250, 3); // Do a short forward-back pulse
  delay(300);

  doTwerk(150, 100, 5); // Do some wiggling
  delay(300);

  doSpin(200, 1000); // Spin in place for 1 second
  delay(300);

  // Check for obstacles during dance and react
  long midDanceDistance = readDistanceCM();
  if (midDanceDistance != -1 && midDanceDistance < 35) { // If obstacle within 35cm during dance
    Serial.print("Obstacle during dance at ");
    Serial.print(midDanceDistance);
    Serial.println(" cm. Performing evasive maneuver.");
    // Evasive maneuver: back up and turn sharply
    setMotorSpeed(1, -100); setMotorSpeed(2, -100); delay(500); stopMotors();
    doSpin(220, 700); // Spin quickly
    stopMotors(); delay(200);
  }

  // Continue with other moves
  setMotorSpeed(1, 180); setMotorSpeed(2, 180); delay(1500); stopMotors(); delay(200); // Move forward faster
  setMotorSpeed(1, 100); setMotorSpeed(2, -50); delay(1200); stopMotors(); delay(200); // Gentle turn left
  setMotorSpeed(1, -180); setMotorSpeed(2, -180); delay(1500); stopMotors(); delay(200); // Move backward faster
  setMotorSpeed(1, -50); setMotorSpeed(2, 100); delay(1200); stopMotors(); delay(200); // Gentle turn right

  doSpin(255, 2000); // Fast, longer spin
  delay(300);

  doTwerk(180, 80, 8); // Fast wiggles
  delay(300);

  // Stop motors, music, and turn off LED
  stopMotors();
  digitalWrite(LED_PIN, LOW);          // Turn LED off

  // Toggle music OFF. This will activate the relay again.
  pulseMusicToggle();
  //musicIsOn = !musicIsOn; // Update our assumed state
//
//  if (!musicIsOn) {
//      Serial.println("Music assumed OFF.");
//  } else {
//      Serial.println("Music assumed ON (unexpected for dance end, re-check manual music state).");
//  }
  Serial.println("Lilybot finished dancing.");
}


// --- Lilybot Stop Routine ---
// This function can be called by voice command to stop motors and music immediately.
void lilybotStop() {
  Serial.println("Dance Stopped!");
  stopMotors();

  Serial.println("Stopping music due to 'STOP' command.");
  pulseMusicToggle(); // This will turn the music OFF
      
  // If music is currently assumed ON, toggle it OFF.
//  if (musicIsOn) {
//      Serial.println("Stopping music due to 'STOP' command.");
//      pulseMusicToggle(); // This will turn the music OFF
//      musicIsOn = false;   // Update our assumed state
//  } else {
//      Serial.println("Music was already assumed OFF. No toggle needed for stop.");
//  }

  digitalWrite(LED_PIN, LOW); // Ensure LED is off if stopping
  delay(500); // Short delay after stopping actions
  Serial.println("Lilybot is halted.");
}


void setup() {
  Serial.begin(9600); // Standard serial for debugging output to computer
  Serial.println("Lilybot System Initializing...");

  // Initialize motor pins as OUTPUTs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); // PWM pins must be set as OUTPUT
  pinMode(PWMB, OUTPUT); // PWM pins must be set as OUTPUT
  stopMotors(); // Ensure motors are off initially

  // Initialize RELAY_CONTROL_PIN as OUTPUT and set to non-triggering state (HIGH for active-LOW relays)
  pinMode(RELAY_CONTROL_PIN, OUTPUT);
  digitalWrite(RELAY_CONTROL_PIN, HIGH); // Ensure relay is off/open initially
  Serial.println("Relay control pin initialized.");

  // Initialize LED pin as OUTPUT
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially

  // Initialize Sonar pins
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  digitalWrite(SONAR_TRIG_PIN, LOW); // Ensure trigger is low initially
  Serial.println("Sonar initialized.");

  // Initialize SoftwareSerial for voice module
  myVR.begin(9600); // This is for communication with the VoiceRecognitionV3 module
  Serial.println("Voice module serial initialized. Loading commands...");
  
  // Load voice commands 
  // These commands must be trained into your VR module at these specific indices.
  myVR.load(VOICE_HEY);
  myVR.load(VOICE_DANCE);
  myVR.load(VOICE_STOP);
  myVR.load(VOICE_TEST);
  myVR.load(VOICE_CHECK); 

  Serial.println("Voice commands loaded.");
  Serial.println("Say 'Zora' to Lilybot to wake up or 'Boogie' to move, 'Freeze' to halt, 'Ping'/'Check' for stopping.");
  Serial.println("Initial music state assumed OFF. Ensure music module is OFF before starting Arduino.");
}

void loop() {
  // Check for voice commands
  // myVR.recognize returns the ID of the recognized command (or 0 if none/error)
  // The buf array contains information about the recognition (buf[1] is the ID)
  if (myVR.recognize(buf, 50) > 0) { // Timeout 50ms for recognition
    Serial.print("Recognized ID: ");
    Serial.println(buf[1], DEC); // Print the recognized ID

    // Clear the incoming serial buffer for the voice module's debug output
    // Note: This is for the VR module's debug, not VR data. It might interfere with VR data if not careful.
    // If you experience missed commands, try commenting out this while loop.
    while (myVR.available() > 0) 
    { // Use myVR.available() and myVR.read() for the VR module's serial data
      myVR.read();
    }

    // Process the recognized command
    switch (buf[1]) {
      case VOICE_HEY:
        Serial.print("Command: Zora (Wake Up)\n");
        setMotorSpeed(1, 150); // Move forward (motor 1)
        setMotorSpeed(2, 150); // Move forward (motor 2)
        delay(1000);
        stopMotors(); // Stop after a short move
        break;
      case VOICE_DANCE:
        Serial.print("Command: Boogie (Dance)\n");
        lilybotDance(); // Start the full dance routine
        break;
      case VOICE_STOP:
        Serial.print("Command: Freeze (stop_\n");
        lilybotStop(); // Stop motors and music immediately
        break;
      case VOICE_TEST:
        Serial.print("Command: Ping (Stop)\n");
        lilybotStop(); // Use 'Ping' as an alternative stop command
        break;
      case VOICE_CHECK: // CHECK also triggers a stop
        Serial.print("Command: Check (Stop)\n");
        lilybotStop(); // Use 'Check' as an alternative stop command
        break;
      default:
        Serial.print("Command: Unrecognized (ID: ");
        Serial.print(buf[1], DEC);
        Serial.println(")");
        break;
    }
  }

  // --- Continuous sonar monitoring when Lilybot is idle (music is OFF) ---
  // The LED can act as a proximity indicator when not dancing.
    long currentDistance = readDistanceCM();
    if (currentDistance != -1 && currentDistance < 15) { // If an object is closer than 15cm
      digitalWrite(LED_PIN, HIGH); // Turn LED on for close object when idle
    } else {
      digitalWrite(LED_PIN, LOW); // Turn LED off if no close object and idle
    }

  
//  if (!musicIsOn) 
//  { // Only check sonar for LED if music is not currently assumed ON
//    long currentDistance = readDistanceCM();
//    if (currentDistance != -1 && currentDistance < 15) 
//    { // If an object is closer than 15cm
//      digitalWrite(LED_PIN, HIGH); // Turn LED on for close object when idle
//    } else 
//    {
//      digitalWrite(LED_PIN, LOW); // Turn LED off if no close object and idle
//    }
//  }

  // Short delay to prevent excessive loop iterations or sonar pings when idle
  delay(10);
}
