#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>

#define LEFT_IN1 33 // Left motors (IN3)
#define LEFT_IN2 32 // Left motors (IN4)
#define RIGHT_IN1 27 // Right motors (IN1)
#define RIGHT_IN2 26 // Right motors (IN2)
#define LEFT_PWM 15 // Left motors PWM (ENB)
#define RIGHT_PWM 25 // Right motors PWM (ENA)
#define MOTOR_SPEED 120 // PWM speed (adjust to 60 if too fast, 100 if stalls)
#define TRIG_PIN_OBSTACLE 4 // Object Avoidance HC-SR04 Trigger
#define ECHO_PIN_OBSTACLE 21 // Object Avoidance HC-SR04 Echo (moved from GPIO 2)
#define SERVO_PIN_OBSTACLE 13 // Object avoidance servo
#define OBSTACLE_DISTANCE_THRESHOLD 30 // Object avoidance (cm, reduced from 45)
#define SERVO_MIN_PULSE 500 // Min pulse width (µs)
#define SERVO_MAX_PULSE 2400 // Max pulse width (µs)

// Servo object
Servo servoObstacle;

// WiFi and WebSocket
const char* ssid = "Input SSID";
const char* password = "Input Password";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Modes
enum RobotMode { STOPPED, OBJECT_AVOIDANCE, MANUAL };
RobotMode currentMode = STOPPED;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>MediMotor Bot 🩺🚗</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: 'Segoe UI', Arial, sans-serif;
      text-align: center;
      background: linear-gradient(135deg, #007bff 0%, #28a745 100%);
      color: #ffffff;
      margin: 0;
      padding: 20px;
    }
    h1 { font-size: 2.5rem; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }
    .container { max-width: 400px; margin: auto; padding: 20px; background: rgba(255,255,255,0.1); border-radius: 15px; }
    .button {
      padding: 15px 30px;
      margin: 10px;
      font-size: 1.5rem;
      border: none;
      border-radius: 10px;
      cursor: pointer;
      color: white;
      transition: transform 0.2s;
    }
    .button:hover { transform: scale(1.1); }
    .obstacle-button { background-color: #ffc107; }
    .stop-button { background-color: #dc3545; }
    .dir-button { background-color: #007bff; width: 100px; height: 100px; font-size: 2rem; }
    .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; margin-top: 20px; }
    .forward { grid-column: 2; }
    .left { grid-column: 1; grid-row: 2; }
    .right { grid-column: 3; grid-row: 2; }
    .backward { grid-column: 2; grid-row: 3; }
    .status { margin-top: 10px; font-size: 1rem; color: #ffffff; font-style: italic; }
  </style>
</head>
<body>
  <div class="container">
    <h1>MediMotor Bot 🩺🚗</h1>
    <p>Control your medical transport robot! 🚀</p>
    <p>Connection: <span id="connection-status">Disconnected</span></p>
    <p>Status: <span id="status">Stopped</span></p>
    <button class="button obstacle-button" onclick="sendMessage('obstacleavoid')">Obstacle Avoidance 🚧</button>
    <button class="button stop-button" onclick="sendMessage('autostop')">Stop 🛑</button>
    <div class="controls">
      <button class="dir-button forward" ontouchstart="sendMessage('forward')" ontouchend="sendMessage('stop')" onmousedown="sendMessage('forward')" onmouseup="sendMessage('stop')" onmouseleave="sendMessage('stop')">↑</button>
      <button class="dir-button left" ontouchstart="sendMessage('left')" ontouchend="sendMessage('stop')" onmousedown="sendMessage('left')" onmouseup="sendMessage('stop')" onmouseleave="sendMessage('stop')">←</button>
      <button class="dir-button right" ontouchstart="sendMessage('right')" ontouchend="sendMessage('stop')" onmousedown="sendMessage('right')" onmouseup="sendMessage('stop')" onmouseleave="sendMessage('stop')">→</button>
      <button class="dir-button backward" ontouchstart="sendMessage('backward')" ontouchend="sendMessage('stop')" onmousedown="sendMessage('backward')" onmouseup="sendMessage('stop')" onmouseleave="sendMessage('stop')">↓</button>
    </div>
  </div>
  <script>
    var gateway = 'ws://192.168.4.1/ws';
    var websocket;
    window.addEventListener('load', initWebSocket);
    function initWebSocket() {
      console.log('Attempting WebSocket connection to ' + gateway);
      websocket = new WebSocket(gateway);
      websocket.onopen = function() {
        console.log('WebSocket connected');
        document.getElementById('connection-status').innerText = 'WebSocket: Connected';
        document.getElementById('status').innerText = 'Stopped';
      };
      websocket.onclose = function() {
        console.log('WebSocket closed, retrying in 2s...');
        document.getElementById('connection-status').innerText = 'WebSocket: Disconnected';
        setTimeout(initWebSocket, 2000);
      };
      websocket.onerror = function(error) {
        console.error('WebSocket error: ', error);
        document.getElementById('connection-status').innerText = 'WebSocket: Error';
      };
      websocket.onmessage = function(event) {
        console.log('Received: ' + event.data);
        document.getElementById('status').innerText = event.data;
      };
    }
    function sendMessage(message) {
      if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(message);
        console.log('Sent: ' + message);
      } else {
        console.log('Cannot send: ' + message + ', WebSocket not connected');
        document.getElementById('connection-status').innerText = 'WebSocket: Disconnected';
      }
    }
  </script>
</body>
</html>)rawliteral";

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");

  // Setup motors
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  stopMotors();
  Serial.println("Motor pins configured");

  // Object avoidance setup
  pinMode(TRIG_PIN_OBSTACLE, OUTPUT);
  pinMode(ECHO_PIN_OBSTACLE, INPUT);
  servoObstacle.attach(SERVO_PIN_OBSTACLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servoObstacle.write(90);
  Serial.println("Object avoidance servo initialized at 90 degrees");

  // WiFi AP setup
  Serial.println("Setting up WiFi AP...");
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("WiFi AP failed to start, retrying...");
    delay(1000);
    WiFi.softAP(ssid, password);
  }
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("WiFi AP started");

  // Web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Serving web page");
    request->send_P(200, "text/html", index_html);
  });
  Serial.println("Web server route configured");

  // WebSocket events
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("Web server started");

  Serial.println("Setup complete");
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected, ID: " + String(client->id()));
    client->text("Connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected, ID: " + String(client->id()));
    currentMode = STOPPED;
    stopMotors();
    servoObstacle.write(90);
    Serial.println("Stopped, obstacle servo centered due to WebSocket disconnect");
    client->text("Disconnected");
  } else if (type == WS_EVT_ERROR) {
    Serial.println("WebSocket error occurred");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo info = (AwsFrameInfo)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String message = (char*)data;
      message.trim();
      message.toLowerCase();
      Serial.print("Received WS message: ");
      Serial.println(message);

      if (message == "obstacleavoid") {
        currentMode = OBJECT_AVOIDANCE;
        moveForward();
        Serial.println("Object avoidance mode started, moving forward");
        client->text("Object Avoidance Started");
      } else if (message == "autostop") {
        currentMode = STOPPED;
        stopMotors();
        Serial.println("Stopped");
        client->text("Stopped");
      } else {
        currentMode = MANUAL;
        if (message == "forward") {
          moveForward();
          Serial.println("Manual: Moving forward");
          client->text("Moving Forward");
        } else if (message == "backward") {
          moveBackward();
          Serial.println("Manual: Moving backward");
          client->text("Moving Backward");
        } else if (message == "left") {
          moveLeft();
          Serial.println("Manual: Turning left");
          client->text("Turning Left");
        } else if (message == "right") {
          moveRight();
          Serial.println("Manual: Turning right");
          client->text("Turning Right");
        } else if (message == "stop") {
          stopMotors();
          Serial.println("Manual: Stopped");
          client->text("Stopped");
        } else {
          Serial.println("Unknown command: " + message);
        }
      }
    }
  }
}

void loop() {
  ws.cleanupClients();
  Serial.print("WebSocket clients: ");
  Serial.println(ws.count());

  // Object avoidance mode
  if (currentMode == OBJECT_AVOIDANCE) {
    static int pos = 0;
    static bool increasing = true;
    static int maxDistance = 0;
    static int bestAngle = 90;
    static int lastDecisionAngle = 0;

    // Continuous servo sweep
    servoObstacle.write(pos);
    delay(100); // Slower sweep for reliable readings
    int distance = getDistanceObstacle();
    Serial.print("Angle: "); Serial.print(pos);
    Serial.print(" degrees, Distance: "); Serial.print(distance);
    Serial.println(" cm");

    // Update best direction
    if (distance > maxDistance) {
      maxDistance = distance;
      bestAngle = pos;
    }

    // Decide movement every 30 degrees
    if (pos % 30 == 0 || pos == 0 || pos == 180) {
      Serial.print("Decision point, Angle: "); Serial.print(pos);
      Serial.print(" degrees, Best distance: "); Serial.print(maxDistance);
      Serial.print(" cm, Best angle: "); Serial.print(bestAngle);
      Serial.println(" degrees");
      if (maxDistance <= OBSTACLE_DISTANCE_THRESHOLD) {
        stopMotors();
        Serial.println("All directions blocked (<30 cm), motors stopped");
        ws.textAll("Obstacle Detected");
        delay(150); // Ensure stop completes
      } else {
        if (bestAngle >= 80 && bestAngle <= 100) {
          moveForward();
          Serial.println("Moving forward (best angle ~90°)");
          ws.textAll("Moving Forward");
          delay(150); // Ensure movement completes
        } else if (bestAngle < 80) {
          moveLeft();
          Serial.println("Turning left (best angle " + String(bestAngle) + "°)");
          ws.textAll("Turning Left");
          delay(150); // Ensure movement completes
        } else {
          moveRight();
          Serial.println("Turning right (best angle " + String(bestAngle) + "°)");
          ws.textAll("Turning Right");
          delay(150); // Ensure movement completes
        }
      }
      maxDistance = 0;
      bestAngle = 90;
      lastDecisionAngle = pos;
    }

    // Update position for continuous sweep
    if (increasing) {
      pos += 5;
      if (pos >= 180) increasing = false;
    } else {
      pos -= 5;
      if (pos <= 0) increasing = true;
    }
  } else {
    stopMotors();
    servoObstacle.write(90);
    Serial.println("Action: Stopped (mode = STOPPED)");
    ws.textAll("Stopped");
  }

  delay(100); // Stabilize sensor readings
}

int getDistanceObstacle() {
  digitalWrite(TRIG_PIN_OBSTACLE, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_OBSTACLE, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_OBSTACLE, LOW);
  long duration = pulseIn(ECHO_PIN_OBSTACLE, HIGH, 30000);
  Serial.print("Obstacle raw duration: ");
  Serial.print(duration);
  Serial.println(" us");
  int cm = (duration * 0.034) / 2;
  if (cm <= 0 || cm > 400) cm = 400;
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
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  analogWrite(LEFT_PWM, MOTOR_SPEED);
  analogWrite(RIGHT_PWM, MOTOR_SPEED);
  Serial.println("Moving forward");
}

void moveBackward() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_PWM, MOTOR_SPEED);
  analogWrite(RIGHT_PWM, MOTOR_SPEED);
  Serial.println("Moving backward");
}

void moveLeft() {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_PWM, MOTOR_SPEED);
  analogWrite(RIGHT_PWM, MOTOR_SPEED);
  Serial.println("Turning left");
}

void moveRight() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  analogWrite(LEFT_PWM, MOTOR_SPEED);
  analogWrite(RIGHT_PWM, MOTOR_SPEED);
  Serial.println("Turning right");
}
