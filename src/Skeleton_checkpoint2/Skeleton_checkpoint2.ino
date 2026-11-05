
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <atomic>
#include <Adafruit_TCS34725.h>

Adafruit_PWMServoDriver pwm(0x40);

// pca9685 channels
#define PITCH_A 0
#define YAW 1
#define PITCH_B 2
#define GRIPPER 3

// buttons - change pins if needed
#define START_BUTTON 27
#define STOP_BUTTON 26
#define EMERGENCY_BUTTON 25

// yaw positions depending on bin location
const int YAW_HOME = 100;
const int YAW_RED = 180;
const int YAW_GREEN = 250;
const int YAW_BASE = 320;

// gripper positions
const int GRIP_OPEN = 100;
const int GRIP_CLOSED = 165;

// pitch positions - still need calibration
const int PITCH_EXTEND = 90;
const int PITCH_RETRACT = 180;

// different extension depending on bin
// temporary values until we calibrate
const int PITCH_RED_BIN = 70;
const int PITCH_GREEN_BIN = 90;
const int PITCH_BLUE_BIN = 80;

// pitch direction
const bool INVERT_PITCH_A = false;
const bool INVERT_PITCH_B = false;

// servo pwm limits
const int SERVO_MIN_US = 1100;
const int SERVO_MAX_US = 1900;

const int GRIP_MIN_US = 1000;
const int GRIP_MAX_US = 2000;

// yaw uses the previous 0-360 scale
const int YAW_SCALE_MIN = 0;
const int YAW_SCALE_MAX = 360;

// timing
const int MOVE_STEP_MS = 20;
const int SENSOR_POLL_MS = 50;
const int BUTTON_POLL_MS = 10;

// motor types
enum MotorType {
  MOVE_YAW,
  MOVE_PITCH,
  MOVE_GRIPPER
};

// color types - jake will use these
enum ObjectColor {
  COLOR_UNKNOWN,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE
};

// robot states
enum RobotState {
  IDLE,
  WAITING_FOR_OBJECT,
  READING_COLOR,
  PICKING_UP,
  DEPOSITING,
  CHECKING_NEXT_OBJECT,
  RETURNING_HOME,
  PAUSED,
  ERROR_STATE
};

// motor command sent through the queue
struct MotorCommand {
  MotorType motor;
  int position;
  TaskHandle_t requester;
};

// freertos queues
QueueHandle_t motorQueue;
QueueHandle_t objectQueue;

// keep track of the robot
RobotState currentState = IDLE;

// shared safety flags
std::atomic<bool> emergencyStop(false);
std::atomic<bool> pauseRequested(true);

// last commanded positions
// these must match the actual arm position
int currentYaw = YAW_HOME;
int currentPitch = PITCH_RETRACT;
int currentGrip = GRIP_CLOSED;

//sensor pinouts
const int TRIG = 5; //both these values are arbitrary - change to actual values
const int ECHO = 18;

//color sensor initializiation
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); //arguments to be tuned

// ------------------------------------------
// pwm conversion
// ------------------------------------------

int angleToPulse(int angle, int minUs, int maxUs) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, minUs, maxUs);
}

int yawToPulse(int value) {
  value = constrain(value, YAW_SCALE_MIN, YAW_SCALE_MAX);

  return map(
    value,
    YAW_SCALE_MIN,
    YAW_SCALE_MAX,
    SERVO_MIN_US,
    SERVO_MAX_US
  );
}

// ------------------------------------------
// servo control
// ------------------------------------------

void writeYaw(int value) {
  pwm.writeMicroseconds(YAW, yawToPulse(value));
}

void writeGripper(int angle) {
  pwm.writeMicroseconds(
    GRIPPER,
    angleToPulse(angle, GRIP_MIN_US, GRIP_MAX_US)
  );
}

void writePitch(int angle) {
  int angleA = INVERT_PITCH_A ? 180 - angle : angle;
  int angleB = INVERT_PITCH_B ? 180 - angle : angle;

  pwm.writeMicroseconds(
    PITCH_A,
    angleToPulse(angleA, SERVO_MIN_US, SERVO_MAX_US)
  );

  pwm.writeMicroseconds(
    PITCH_B,
    angleToPulse(angleB, SERVO_MIN_US, SERVO_MAX_US)
  );
}

// stop sending servo pulses
// this does not disconnect servo power
void disableServoOutputs() {
  pwm.setPin(PITCH_A, 0);
  pwm.setPin(PITCH_B, 0);
  pwm.setPin(YAW, 0);
  pwm.setPin(GRIPPER, 0);
}

// check if the target is within the code limits
// still need to calibrate the real mechanical limits
bool validTarget(MotorType motor, int target) {
  if (motor == MOVE_YAW) {
    return target >= YAW_SCALE_MIN &&
           target <= YAW_SCALE_MAX;
  }

  return target >= 0 && target <= 180;
}

// move smoothly without blocking other rtos tasks
// both pitch motors move together
bool moveSmooth(MotorType motor, int target) {
  int *current;

  if (motor == MOVE_YAW) {
    current = &currentYaw;
  } else if (motor == MOVE_PITCH) {
    current = &currentPitch;
  } else {
    current = &currentGrip;
  }

  int start = *current;
  int step = (target >= start) ? 1 : -1;

  for (int pos = start; ; pos += step) {

    // emergency stop interrupts normal movement
    if (emergencyStop.load()) {
      return false;
    }

    if (motor == MOVE_YAW) {
      writeYaw(pos);
    } else if (motor == MOVE_PITCH) {
      writePitch(pos);
    } else {
      writeGripper(pos);
    }

    *current = pos;

    if (pos == target) break;

    vTaskDelay(pdMS_TO_TICKS(MOVE_STEP_MS));
  }

  return !emergencyStop.load();
}

// ------------------------------------------
// freertos servo task - priority 2
// ------------------------------------------

void ServoTask(void *parameter) {
  MotorCommand command;
  bool outputsDisabled = false;

  for (;;) {

    // stop all pwm outputs when emergency is active
    if (emergencyStop.load()) {
      if (!outputsDisabled) {
        disableServoOutputs();
        outputsDisabled = true;
      }

      // emergency stays latched until reboot
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // wait for motor commands
    if (xQueueReceive(
          motorQueue,
          &command,
          pdMS_TO_TICKS(20)
        ) == pdTRUE) {

      moveSmooth(command.motor, command.position);

      // tell sorting task the movement ended
      // it may have ended because of emergency stop
      if (command.requester != nullptr) {
        xTaskNotifyGive(command.requester);
      }
    }
  }
}

// ------------------------------------------
// send motor command and wait
// ------------------------------------------

bool moveAndWait(MotorType motor, int position) {
  if (emergencyStop.load()) return false;

  if (!validTarget(motor, position)) {
    Serial.println("invalid motor position");
    return false;
  }

  MotorCommand command = {
    motor,
    position,
    xTaskGetCurrentTaskHandle()
  };

  // send command to servo task
  if (xQueueSend(
        motorQueue,
        &command,
        pdMS_TO_TICKS(100)
      ) != pdTRUE) {
    return false;
  }

  // wait until the motor task finishes
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  return !emergencyStop.load();
}

// ------------------------------------------
// sensor functions - jake
// ------------------------------------------

// jake will add ultrasonic detection here
// return true when an object is at the pickup base
// keep this function short so sensor task can keep running
bool detectObject() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long us = pulseIn(ECHO, HIGH, 30000); //pulseIn is NOT REAL TIME. To be implemented: Interrupt on ECHO change, timestamp interrupts, and measure difference.
  float cm = us * .0343f / 2.0f

  if(cm >= 2 && cm <= 400) { //max specd distance - adjust if readings are faulty
    return true;
  }

  return false;
}

// jake will add color sensor code here
// return the color of the object at the base
ObjectColor detectColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (c < 100) { //tune "100" to match up with value seen when there's nothing there
    return COLOR_UNKNOWN;
  }                          
  else if (r > g * 1.5f && r > b * 1.5f) { //multipliers to ensure color is strongly red, green, or blue. to be tuned based on actual readings
      return COLOR_RED;
  }
  else if (g > r * 1.2f && g > b * 1.2f) {
      return COLOR_GREEN;
  }
  else if (b > r * 1.2f && b > g * 1.2f) {
    return COLOR_BLUE;
  }
    
  return COLOR_UNKNOWN;
}

// ------------------------------------------
// freertos sensor task - priority 3
// ------------------------------------------

void SensorTask(void *parameter) {
  bool previousDetection = false;
  bool detected;

  for (;;) {

    if (emergencyStop.load()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // check ultrasonic sensor
    detected = detectObject();

    // only send an event when a new object appears
    // avoids sending the same object over and over
    if (detected && !previousDetection) {
      uint8_t objectEvent = 1;

      // save the event even if arm is still sorting
      // queue holds one pending object for now
      xQueueSend(objectQueue, &objectEvent, 0);

      Serial.println("object detected");
    }

    // rearm after object is no longer detected
    previousDetection = detected;

    vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
  }
}

// ------------------------------------------
// wait for start/resume
// ------------------------------------------

bool waitUntilRunning() {
  while (pauseRequested.load()) {

    if (emergencyStop.load()) return false;

    vTaskDelay(pdMS_TO_TICKS(20));
  }

  return !emergencyStop.load();
}

// ------------------------------------------
// pickup sequence
// ------------------------------------------

bool pickupObject() {
  currentState = PICKING_UP;

  Serial.println("moving to base");

  // move to base
  if (!moveAndWait(MOVE_YAW, YAW_BASE)) return false;

  // open gripper
  if (!moveAndWait(MOVE_GRIPPER, GRIP_OPEN)) return false;

  // extend pitch toward object
  if (!moveAndWait(MOVE_PITCH, PITCH_EXTEND)) return false;

  // grab object
  if (!moveAndWait(MOVE_GRIPPER, GRIP_CLOSED)) return false;

  // retract arm with object
  if (!moveAndWait(MOVE_PITCH, PITCH_RETRACT)) return false;

  Serial.println("object picked up");
  return true;
}

// ------------------------------------------
// select bin depending on color
// ------------------------------------------
// select bin depending on color
bool getBinPosition(
  ObjectColor color,
  int &yawPosition,
  int &pitchPosition
) {
  if (color == COLOR_RED) {
    yawPosition = YAW_RED;
    pitchPosition = PITCH_RED_BIN;
    return true;
  }

  if (color == COLOR_GREEN) {
    yawPosition = YAW_GREEN;
    pitchPosition = PITCH_GREEN_BIN;
    return true;
  }

  if (color == COLOR_BLUE) {
    // blue bin is at the same yaw as home
    yawPosition = YAW_HOME;
    pitchPosition = PITCH_BLUE_BIN;
    return true;
  }

  return false;
}

// ------------------------------------------
// deposit sequence
// ------------------------------------------

bool depositObject(ObjectColor color) {
  currentState = DEPOSITING;

  int binYaw;
  int binPitch;

  if (!getBinPosition(color, binYaw, binPitch)) {
    Serial.println("unknown bin");
    return false;
  }

  Serial.println("moving to bin");

  // rotate to correct bin
  if (!moveAndWait(MOVE_YAW, binYaw)) return false;

  // extend depending on bin position
  if (!moveAndWait(MOVE_PITCH, binPitch)) return false;

  // release object
  if (!moveAndWait(MOVE_GRIPPER, GRIP_OPEN)) return false;

  // retract arm
  if (!moveAndWait(MOVE_PITCH, PITCH_RETRACT)) return false;

  // close gripper
  if (!moveAndWait(MOVE_GRIPPER, GRIP_CLOSED)) return false;

  Serial.println("object deposited");
  return true;
}

// ------------------------------------------
// return home when no new objects
// ------------------------------------------

bool returnHome() {
  currentState = RETURNING_HOME;

  Serial.println("returning home");

  if (!moveAndWait(MOVE_PITCH, PITCH_RETRACT)) return false;
  if (!moveAndWait(MOVE_GRIPPER, GRIP_CLOSED)) return false;
  if (!moveAndWait(MOVE_YAW, YAW_HOME)) return false;

  return true;
}

// ------------------------------------------
// freertos sorting task - priority 1
// ------------------------------------------

void SortingTask(void *parameter) {
  uint8_t objectEvent;
  bool objectPending = false;

  for (;;) {

    // wait for start button if paused
    if (pauseRequested.load()) {
      currentState = PAUSED;
    }

    if (!waitUntilRunning()) break;

    currentState = WAITING_FOR_OBJECT;

    // wait for object if none is pending
    if (!objectPending) {
      if (xQueueReceive(
            objectQueue,
            &objectEvent,
            pdMS_TO_TICKS(50)
          ) != pdTRUE) {
        continue;
      }
    }

    // clear pending event since we are processing it
    objectPending = false;

    if (!waitUntilRunning()) break;

    // read color before moving arm
    currentState = READING_COLOR;

    ObjectColor color = detectColor();

    if (color == COLOR_UNKNOWN) {
      Serial.println("unknown color, skipping");
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // pick up current object
    if (!pickupObject()) {
      break;
    }

    // finish depositing before taking next object
    if (!depositObject(color)) {
      break;
    }

    currentState = CHECKING_NEXT_OBJECT;

    // normal stop pauses after finishing deposit
    if (pauseRequested.load()) {
      currentState = PAUSED;

      if (!waitUntilRunning()) break;
    }

    // check if another object arrived while sorting
    if (xQueueReceive(
          objectQueue,
          &objectEvent,
          0
        ) == pdTRUE) {

      Serial.println("new object waiting");

      // skip home, next loop goes to base
      objectPending = true;
      continue;
    }

    // no new object, return home
    if (!returnHome()) {
      break;
    }

    Serial.println("sorting cycle completed");
  }

  // no automatic restart after an error
  currentState = ERROR_STATE;
  Serial.println("sorting task stopped");

  vTaskSuspend(NULL);
}

// ------------------------------------------
// freertos safety task - priority 4
// ------------------------------------------

void SafetyTask(void *parameter) {
  bool lastStart = false;
  bool lastStop = false;
  TickType_t lastStartTime = 0;
  TickType_t lastStopTime = 0;

  for (;;) {

    // buttons use input pullup
    // pressed means gpio reads low
    bool startPressed =
      digitalRead(START_BUTTON) == LOW;

    bool stopPressed =
      digitalRead(STOP_BUTTON) == LOW;

    bool emergencyPressed =
      digitalRead(EMERGENCY_BUTTON) == LOW;

    // emergency stop overrides everything
    if (emergencyPressed) {
      emergencyStop.store(true);
      pauseRequested.store(true);
    }

    if (emergencyStop.load()) {
      // keep the system stopped until reboot
      vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
      continue;
    }

    TickType_t now = xTaskGetTickCount();

    // start/resume button
    if (startPressed && !lastStart &&
        (now - lastStartTime) >= pdMS_TO_TICKS(150)) {
      pauseRequested.store(false);
      lastStartTime = now;

      Serial.println("start pressed");
    }

    // normal stop button
    // current object will finish depositing
    if (stopPressed && !lastStop &&
        (now - lastStopTime) >= pdMS_TO_TICKS(150)) {
      pauseRequested.store(true);
      lastStopTime = now;

      Serial.println("stop requested");
    }

    lastStart = startPressed;
    lastStop = stopPressed;

    vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
  }
}

// ------------------------------------------
// setup
// ------------------------------------------

void setup() {
  Serial.begin(115200);

  // initialize buttons
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  pinMode(EMERGENCY_BUTTON, INPUT_PULLUP);

  // do not arm if emergency is active at startup
  if (digitalRead(EMERGENCY_BUTTON) == LOW) {
    emergencyStop.store(true);
    Serial.println("emergency stop active");
  }

  //initialize ultrasonic sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //initialize color sensor
  while(!tcs.begin) {
    Serial.println("Color sensor not found");
    delay(1000);
  }

  // initialize i2c
  Wire.begin(21, 22);

  // initialize servo driver
  if (!pwm.begin()) {
    Serial.println("pca9685 not found");
    while (true) {
      delay(1000);
    }
  }

  pwm.setPWMFreq(50);
  delay(100);

  // do not command any initial servo movement
  disableServoOutputs();

  // create motor command queue
  motorQueue = xQueueCreate(
    16,
    sizeof(MotorCommand)
  );

  // one pending object event
  objectQueue = xQueueCreate(
    1,
    sizeof(uint8_t)
  );

  if (motorQueue == NULL || objectQueue == NULL) {
    Serial.println("queue creation failed");
    return;
  }

  // servo control task
  BaseType_t servoOK = xTaskCreatePinnedToCore(
    ServoTask,
    "ServoControl",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  // sensor task
  BaseType_t sensorOK = xTaskCreatePinnedToCore(
    SensorTask,
    "Sensor",
    4096,
    NULL,
    3,
    NULL,
    1
  );

  // sorting controller task
  BaseType_t sortingOK = xTaskCreatePinnedToCore(
    SortingTask,
    "Sorting",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  // safety task
  BaseType_t safetyOK = xTaskCreatePinnedToCore(
    SafetyTask,
    "Safety",
    4096,
    NULL,
    4,
    NULL,
    1
  );

  if (servoOK != pdPASS ||
      sensorOK != pdPASS ||
      sortingOK != pdPASS ||
      safetyOK != pdPASS) {
    Serial.println("task creation failed");
    emergencyStop.store(true);
  }

  Serial.println("rtos initialized");
  Serial.println("waiting for start button");
}

// ------------------------------------------
// main loop
// ------------------------------------------

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
