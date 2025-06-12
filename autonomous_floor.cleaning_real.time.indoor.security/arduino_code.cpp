// ESP32 requires explicit LEDC setup for PWM
// No Servo library needed for this version

// --- Pin Definitions (ESP32 GPIO Numbers - CHOOSE YOURS CAREFULLY!) ---

// L298N Motor Driver Pins
// IMPORTANT: These signals to L298N might need 3.3V to 5V logic level shifting
const int MOTOR_A_IN1_PIN = 26; // Example GPIO
const int MOTOR_A_IN2_PIN = 27; // Example GPIO
const int MOTOR_A_ENA_PIN = 14; // Example GPIO for PWM (LEDC channel will be assigned)

const int MOTOR_B_IN3_PIN = 32; // Example GPIO
const int MOTOR_B_IN4_PIN = 33; // Example GPIO
const int MOTOR_B_ENB_PIN = 12; // Example GPIO for PWM (LEDC channel will be assigned)

// HC-SR04 Ultrasonic Sensor Pins
// IMPORTANT: ECHO pins from sensor (5V) to ESP32 (3.3V) need a voltage divider or level shifter
// Sensor 1 (Center)
const int TRIG_PIN_CENTER = 23; // Example GPIO
const int ECHO_PIN_CENTER = 22; // Example GPIO (NEEDS VOLTAGE DIVIDER/SHIFTER)
// Sensor 2 (Left)
const int TRIG_PIN_LEFT = 19;   // Example GPIO
const int ECHO_PIN_LEFT = 18;   // Example GPIO (NEEDS VOLTAGE DIVIDER/SHIFTER)
// Sensor 3 (Right)
const int TRIG_PIN_RIGHT = 5;   // Example GPIO
const int ECHO_PIN_RIGHT = 17;  // Example GPIO (NEEDS VOLTAGE DIVIDER/SHIFTER)

// --- LEDC PWM Configuration ---
const int LEDC_PWM_FREQ = 5000;   // PWM frequency in Hz (e.g., 5kHz)
const int LEDC_PWM_RESOLUTION = 8; // PWM resolution (8-bit means 0-255 duty cycle)
const int LEDC_CHANNEL_A = 0;     // LEDC channel for Motor A speed
const int LEDC_CHANNEL_B = 1;     // LEDC channel for Motor B speed

// --- Constants ---
const int OBSTACLE_THRESHOLD_CENTER = 20;
const int OBSTACLE_THRESHOLD_SIDE = 25;
const int MOTOR_SPEED_FORWARD = 150; // Duty cycle (0-255 for 8-bit resolution)
const int MOTOR_SPEED_TURN = 130;
const int TURN_DURATION = 600;
const int BACKUP_DURATION = 500;
const int SENSOR_READ_DELAY = 60;

void setup() {
  Serial.begin(115200); // ESP32 can handle higher baud rates

  // --- Initialize Motor Pins ---
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN3_PIN, OUTPUT);
  pinMode(MOTOR_B_IN4_PIN, OUTPUT);

  // --- Configure LEDC for Motor PWM ---
  ledcSetup(LEDC_CHANNEL_A, LEDC_PWM_FREQ, LEDC_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_ENA_PIN, LEDC_CHANNEL_A);

  ledcSetup(LEDC_CHANNEL_B, LEDC_PWM_FREQ, LEDC_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_B_ENB_PIN, LEDC_CHANNEL_B);

  // --- Initialize Ultrasonic Sensor Pins ---
  pinMode(TRIG_PIN_CENTER, OUTPUT);
  pinMode(ECHO_PIN_CENTER, INPUT); // Remember voltage divider!
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);   // Remember voltage divider!
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);  // Remember voltage divider!

  Serial.println("ESP32 Robot Initialized with 3 Ultrasonic Sensors. Ready to go!");
  stopMotors();
  delay(1000);
}

void loop() {
  int distanceCenter = getDistance(TRIG_PIN_CENTER, ECHO_PIN_CENTER);
  delay(SENSOR_READ_DELAY);
  int distanceLeft = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  delay(SENSOR_READ_DELAY);
  int distanceRight = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

  Serial.print("Center: "); Serial.print(distanceCenter); Serial.print(" cm | ");
  Serial.print("Left: ");   Serial.print(distanceLeft);   Serial.print(" cm | ");
  Serial.print("Right: ");  Serial.print(distanceRight);  Serial.println(" cm");

  // --- Obstacle Avoidance Logic (Same as before) ---
  if (distanceCenter <= OBSTACLE_THRESHOLD_CENTER && distanceCenter > 0) {
    Serial.println("Obstacle detected by CENTER sensor!");
    stopMotors();
    moveBackward();
    delay(BACKUP_DURATION);
    stopMotors();
    delay(200);

    // Decision making based on side sensors
    // Re-read sensors after backup for more accuracy if needed
    // int updatedDistanceLeft = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    // delay(SENSOR_READ_DELAY);
    // int updatedDistanceRight = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

    if (distanceLeft > distanceRight && distanceLeft > OBSTACLE_THRESHOLD_SIDE) {
      Serial.println("Turning Left");
      turnLeft();
      delay(TURN_DURATION);
    } else if (distanceRight > distanceLeft && distanceRight > OBSTACLE_THRESHOLD_SIDE) {
      Serial.println("Turning Right");
      turnRight();
      delay(TURN_DURATION);
    } else {
      Serial.println("Path significantly blocked, performing default turn (Right).");
      turnRight();
      delay(TURN_DURATION + 200);
    }
    stopMotors();
    delay(100);

  } else if (distanceLeft <= OBSTACLE_THRESHOLD_SIDE && distanceLeft > 0 && distanceCenter > OBSTACLE_THRESHOLD_CENTER) {
    Serial.println("Obstacle detected by LEFT sensor. Veering Right.");
    stopMotors();
    veerRight(); // You would implement veer functions similarly
    delay(TURN_DURATION / 2);
    stopMotors();
    delay(100);

  } else if (distanceRight <= OBSTACLE_THRESHOLD_SIDE && distanceRight > 0 && distanceCenter > OBSTACLE_THRESHOLD_CENTER) {
    Serial.println("Obstacle detected by RIGHT sensor. Veering Left.");
    stopMotors();
    veerLeft(); // You would implement veer functions similarly
    delay(TURN_DURATION / 2);
    stopMotors();
    delay(100);

  } else {
    moveForward();
  }
  delay(50);
}

// --- Function to get distance from a given HC-SR04 sensor ---
int getDistance(int trigPin, int echoPin) {
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms (approx 5m, more than enough)
  
  int dist = duration * 0.0343 / 2;
  if (dist == 0 || dist > 400) { // If timeout or out of practical range
      return 400; 
  }
  return dist;
}

// --- Motor Control Functions ---
void moveForward() {
  digitalWrite(MOTOR_A_IN1_PIN, HIGH); digitalWrite(MOTOR_A_IN2_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_A, MOTOR_SPEED_FORWARD);
  digitalWrite(MOTOR_B_IN3_PIN, HIGH); digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_B, MOTOR_SPEED_FORWARD);
}

void moveBackward() {
  Serial.println("Moving Backward");
  digitalWrite(MOTOR_A_IN1_PIN, LOW); digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  ledcWrite(LEDC_CHANNEL_A, MOTOR_SPEED_FORWARD);
  digitalWrite(MOTOR_B_IN3_PIN, LOW); digitalWrite(MOTOR_B_IN4_PIN, HIGH);
  ledcWrite(LEDC_CHANNEL_B, MOTOR_SPEED_FORWARD);
}

void turnRight() {
  Serial.println("Turning Right (Pivot)");
  digitalWrite(MOTOR_A_IN1_PIN, HIGH); digitalWrite(MOTOR_A_IN2_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_A, MOTOR_SPEED_TURN);
  digitalWrite(MOTOR_B_IN3_PIN, LOW);  digitalWrite(MOTOR_B_IN4_PIN, HIGH);
  ledcWrite(LEDC_CHANNEL_B, MOTOR_SPEED_TURN);
}

void turnLeft() {
  Serial.println("Turning Left (Pivot)");
  digitalWrite(MOTOR_A_IN1_PIN, LOW);  digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  ledcWrite(LEDC_CHANNEL_A, MOTOR_SPEED_TURN);
  digitalWrite(MOTOR_B_IN3_PIN, HIGH); digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_B, MOTOR_SPEED_TURN);
}

// Implement veerRight and veerLeft similarly, adjusting ledcWrite values
void veerRight() {
  Serial.println("Veering Right");
  digitalWrite(MOTOR_A_IN1_PIN, HIGH); digitalWrite(MOTOR_A_IN2_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_A, MOTOR_SPEED_TURN - 30); // Slower left
  digitalWrite(MOTOR_B_IN3_PIN, HIGH); digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_B, MOTOR_SPEED_TURN);     // Normal right
}

void veerLeft() {
  Serial.println("Veering Left");
  digitalWrite(MOTOR_A_IN1_PIN, HIGH); digitalWrite(MOTOR_A_IN2_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_A, MOTOR_SPEED_TURN);     // Normal left
  digitalWrite(MOTOR_B_IN3_PIN, HIGH); digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_B, MOTOR_SPEED_TURN - 30); // Slower right
}

void stopMotors() {
  Serial.println("Stopping Motors");
  digitalWrite(MOTOR_A_IN1_PIN, LOW); digitalWrite(MOTOR_A_IN2_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_A, 0);
  digitalWrite(MOTOR_B_IN3_PIN, LOW); digitalWrite(MOTOR_B_IN4_PIN, LOW);
  ledcWrite(LEDC_CHANNEL_B, 0);
}