#include <ESP32Servo.h>

#define TRIG_PIN_OBSTACLE 12    // Obstacle HC-SR04 Trigger
#define ECHO_PIN_OBSTACLE 14    // Obstacle HC-SR04 Echo
#define TRIG_PIN_LID 23         // Smart Lid HC-SR04 Trigger
#define ECHO_PIN_LID 22         // Smart Lid HC-SR04 Echo
#define SERVO_PIN_OBSTACLE 13   // Servo for panning obstacle HC-SR04
#define SERVO_PIN_LID 5         // Servo for smart lid
#define IR_LEFT 19              // Left IR sensor
#define IR_RIGHT 18             // Right IR sensor
#define MAX_DISTANCE 400        // Max distance in cm
#define OBSTACLE_DISTANCE_THRESHOLD 10  // Obstacle avoidance (cm)
#define LID_DISTANCE_THRESHOLD 15       // Lid opening (cm)

// Motor pins for L298N
#define LEFT_IN1 33       // Left motors (IN3)
#define LEFT_IN2 32       // Left motors (IN4)
#define RIGHT_IN1 26      // Right motors (IN1)
#define RIGHT_IN2 27      // Right motors (IN2)
#define LEFT_PWM 25       // Left motors PWM (ENA)
#define RIGHT_PWM 15      // Right motors PWM (ENB)

// Motor speed
#define MOTOR_SPEED 200

Servo servoObstacle;
Servo servoLid;

int distanceObstacle = 0;
int leftDistance = 0;
int rightDistance = 0;
boolean object;
int distanceLid = 0;

void setup() {
  Serial.begin(115200);
  
  // Ultrasonic sensor pins
  pinMode(TRIG_PIN_OBSTACLE, OUTPUT);
  pinMode(ECHO_PIN_OBSTACLE, INPUT);
  
  // Smart Lid HC-SR04 pins
  pinMode(TRIG_PIN_LID, OUTPUT);
  pinMode(ECHO_PIN_LID, INPUT);
  
  // IR sensor pins
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  // Servo setup
  servoObstacle.attach(SERVO_PIN_OBSTACLE, 500, 2400); // Min/max pulse width
  servoObstacle.write(90);  // Center
  Serial.println("Obstacle servo initialized at 90 degrees");
  servoLid.attach(SERVO_PIN_LID, 500, 2400); // Min/max pulse width
  servoLid.write(0);  // Lid closed
  Serial.println("Lid servo initialized at 0 degrees (closed)");
  delay(1000);
  
  // Motor pins
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  
  // Initial stop
  stopMotors();
}

void loop() {
  // Smart lid control (independent)
  controlSmartLid();
  
  // Line following and obstacle avoidance
  int leftIR = digitalRead(IR_LEFT);   // LOW = on line, HIGH = off
  int rightIR = digitalRead(IR_RIGHT); // LOW = on line, HIGH = off
  Serial.print("Left IR: "); Serial.print(leftIR);
  Serial.print(", Right IR: "); Serial.println(rightIR);
  
  if (leftIR == 0 && rightIR == 0) {
    objectAvoid();
  } else if (leftIR == 0 && rightIR == 1) {
    objectAvoid();
    moveLeft();
  } else if (leftIR == 1 && rightIR == 0) {
    objectAvoid();
    moveRight();
  } else if (leftIR == 1 && rightIR == 1) {
    stopMotors();
  }
}

void controlSmartLid() {
  distanceLid = getDistanceLid();
  Serial.print("Lid Distance: ");
  Serial.print(distanceLid);
  Serial.println(" cm");
  
  if (distanceLid > 0 && distanceLid <= LID_DISTANCE_THRESHOLD) {
    servoLid.write(90);  // Open lid
    Serial.println("Object detected within 15 cm, lid opened to 90 degrees");
  } else {
    servoLid.write(0);  // Close lid
    Serial.println("No object within 15 cm, lid closed at 0 degrees");
  }
}

void objectAvoid() {
  distanceObstacle = getDistanceObstacle();
  Serial.print("Obstacle Distance: ");
  Serial.print(distanceObstacle);
  Serial.println(" cm");
  
  if (distanceObstacle > 0 && distanceObstacle <= OBSTACLE_DISTANCE_THRESHOLD) {
    Serial.println("Obstacle detected within 10 cm, avoiding...");
    stopMotors();
    moveBackward();
    delay(500);  // Backward for 0.5 seconds
    stopMotors();
    
    // Look left
    servoObstacle.write(150);  // Left
    delay(500);
    leftDistance = getDistanceObstacle();
    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.println(" cm");
    
    // Look right
    servoObstacle.write(30);  // Right
    delay(500);
    rightDistance = getDistanceObstacle();
    Serial.print("Right Distance: ");
    Serial.print(rightDistance);
    Serial.println(" cm");
    
    // Return to center
    servoObstacle.write(90);  // Center
    delay(500);
    
    if (rightDistance <= leftDistance) {
      object = true;  // Turn left
      turn();
    } else {
      object = false; // Turn right
      turn();
    }
  } else {
    moveForward();
  }
}

int getDistanceObstacle() {
  digitalWrite(TRIG_PIN_OBSTACLE, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_OBSTACLE, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_OBSTACLE, LOW);
  
  long duration = pulseIn(ECHO_PIN_OBSTACLE, HIGH, 30000); // Timeout 30ms
  int cm = (duration * 0.034) / 2;
  
  if (cm <= 0 || cm > MAX_DISTANCE) {
    cm = MAX_DISTANCE;
  }
  return cm;
}

int getDistanceLid() {
  digitalWrite(TRIG_PIN_LID, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_LID, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_LID, LOW);
  
  long duration = pulseIn(ECHO_PIN_LID, HIGH, 30000); // Timeout 30ms
  int cm = (duration * 0.034) / 2;
  
  if (cm <= 0 || cm > MAX_DISTANCE) {
    cm = MAX_DISTANCE;
  }
  return cm;
}

void stopMotors() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  Serial.println("Motors stopped");
}

void moveForward() {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_PWM, MOTOR_SPEED);
  analogWrite(RIGHT_PWM, MOTOR_SPEED);
  Serial.println("Moving forward");
}

void moveBackward() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  analogWrite(LEFT_PWM, MOTOR_SPEED);
  analogWrite(RIGHT_PWM, MOTOR_SPEED);
  Serial.println("Moving backward");
}

void turn() {
  if (object == false) {
    Serial.println("Turning right to avoid obstacle");
    moveRight();
    delay(700);
    moveForward();
    delay(800);
    moveLeft();
    delay(900);
    if (digitalRead(IR_RIGHT) == 1) {
      Serial.println("Right IR off line, continuing search");
    } else {
      Serial.println("Right IR on line, moving forward");
      moveForward();
    }
  } else {
    Serial.println("Turning left to avoid obstacle");
    moveLeft();
    delay(700);
    moveForward();
    delay(800);
    moveRight();
    delay(900);
    if (digitalRead(IR_LEFT) == 1) {
      Serial.println("Left IR off line, continuing search");
    } else {
      Serial.println("Left IR on line, moving forward");
      moveForward();
    }
  }
}

void moveRight() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_PWM, MOTOR_SPEED);
  analogWrite(RIGHT_PWM, MOTOR_SPEED);
  Serial.println("Turning right");
}

void moveLeft() {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  analogWrite(LEFT_PWM, MOTOR_SPEED);
  analogWrite(RIGHT_PWM, MOTOR_SPEED);
  Serial.println("Turning left");
}