//July 16th
//run motor at constant speed
//pid working!! acceleration
// cleaned up from serial debugs
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <HardwareSerial.h>


enum JointType { STEPPER, SERVO };

struct Joint {
  const char* name;
  JointType type;
  float minAngle;
  float maxAngle;

  union {
    struct {
      uint8_t motorID;
      float gearRatio;
      float rpm;
      uint8_t index;  // new field for mapping t1–t3 to array index
    } stepper;

    struct {
      uint8_t channel;
      float homeAngle;
    } servo;
  };
};

Joint joints[] = {
  { .name = "t1", .type = STEPPER, .minAngle = -100.0, .maxAngle = 100.0, .stepper = { .motorID = 0xE0, .gearRatio = 3.778, .rpm = 30.0, .index = 0 } },
  { .name = "t2", .type = STEPPER, .minAngle = -90.0, .maxAngle = 50.0, .stepper = { .motorID = 0xE1, .gearRatio = 15.0,  .rpm = 20.0, .index = 1 } },
  { .name = "t3", .type = STEPPER, .minAngle = -45.0, .maxAngle = 90.0, .stepper = { .motorID = 0xE2, .gearRatio = 10.0,  .rpm = 40.0, .index = 2 } },
  { .name = "t4", .type = SERVO,   .minAngle = 0.0,   .maxAngle = 270.0, .servo = { .channel = 0, .homeAngle = 180.0 } },
  { .name = "t5", .type = SERVO,   .minAngle = 0.0,   .maxAngle = 270.0, .servo = { .channel = 1, .homeAngle = 0.0 } },
  { .name = "t6", .type = SERVO,   .minAngle = 0.0,   .maxAngle = 130.0, .servo = { .channel = 2, .homeAngle = 0.0  } }
};
const int NUM_JOINTS = sizeof(joints) / sizeof(joints[0]);


float lastServoAngles[3] = {0};  // tracks last commanded angle per channel
float lastStepperAngles[3] = {0};  // t1, t2, t3

#define T1 joints[0]
#define T2 joints[1]
#define T3 joints[2]
#define T4 joints[3]
#define T5 joints[4]
#define T6 joints[5]

//////////////////////// pid
struct PID {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float lastError;
};
PID jointPID = { 1.5, 0.0, 0.2, 0.0, 0.0 };  // Tune this!


// ========================== COMMAND LIST SETUP ==========================
const char* COMMAND_LIST[] = {
  "  set <joint> <angle>",        // Replaces: goto
  "  move <joint> <delta>",       // Replaces: moveby
  "  rpm <joint> <val>",          // Replaces: set rpm
  "  arm <a1> <a2> <a3> <a4> <a5> <a6>",  // Full arm movement
  "  read <joint>",               // Unchanged
  "  home",                       // Unchanged
  "  grip up/down/home/open/close", // Replaces: gripper
  "  status"                      // Unchanged
};

const int NUM_COMMANDS = sizeof(COMMAND_LIST) / sizeof(COMMAND_LIST[0]);

// ========================== MKS SERVO42C SETUP ==========================
// Used to control closed-loop stepper motors over UART (Stepper t1–t3)

HardwareSerial SerialMKS(1); // UART1
const int PULSES_PER_REV = 3200;
bool ccw = false;

// ========================== PCA9685 SERVO SETUP ==========================
// Used to control hobby servo motors over I2C (Servo t4–t6)

#define SDA_PIN 12
#define SCL_PIN 13
#define SERVO_FREQ 50
#define MIN_ANGLE 0
#define MAX_ANGLE 270
#define MIN_PULSE_US 400
#define MAX_PULSE_US 2351

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ====================== Function Prototypes =======================
// === Core Motor Control ===
bool rotateMotorBy(uint8_t motorID, float angle_deg, bool ccw, float rpm);
bool runMotorAtSpeed(uint8_t motorID, float rpm, bool ccw);
bool stopMotor(uint8_t motorID);
bool moveJointWithF6UntilTarget(Joint& joint, float targetAngle, float rpm);
void goToAngle(const Joint& joint, float targetJointAngle);
float readCurrentAngle(const Joint& joint);
void moveServo(const Joint& joint, float angle);
void goJoint(Joint* joint, float angle);
void goArm(float a1, float a2, float a3, float a4, float a5, float a6);
float computePID(PID& pid, float error, float dt);
bool moveJointWithF6_PID(Joint& joint, float targetAngle);


// === Utilities ===
Joint* getJointByName(const String& name);
void refreshStepperAngles();

// === Serial Command Handlers ===
void handleSetCommand(const String& input);     // replaces old handleGotoCommand()
void handleMoveCommand(const String& input);    // replaces old handleMoveByCommand()
void handleRPMCommand(const String& input);     // replaces old handleSetRPM()
void handleReadCommand(const String& input);
void handleArmCommand(const String& input);
void handleGripCommand(const String& input);    // replaces old handleGripperCommand()
void handleMoveF6Command(const String& input);  // new: run to angle using F6

// === Status and UI ===
void printStatus();
void printHelp();
void printWelcomeMessage();

// === Misc ===
void homeAllJoints();
void moveToPickupPosition();
////////////////////////////////////////////////////////////////////



void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  SerialMKS.begin(38400, SERIAL_8N1,6, 5);
  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  //refreshStepperAngles();
  //homeAllJoints();
  printWelcomeMessage();

}

// ======================= AVAILABLE SERIAL COMMANDS =======================
// Format: Send these commands via Serial Monitor or from Raspberry Pi
//
// ┌───────────────────────────────────────────────────────────────────────┐
// │ Command Format            │ Description                               │
// ├───────────────────────────┼───────────────────────────────────────────┤
// │ SET <joint> <angle>       │ Move joint t1–t6 to specified angle       │
// │ MOVE <joint> <delta>      │ Rotate joint t1–t3 by ±degrees            │
// │ RPM <joint> <val>         │ Set RPM of specified stepper (t1–t3)      │
// │ ARM <a1> <a2> ... <a6>    │ Set all 6 joints (t1–t6) to angles        │
// │ READ <joint>              │ Read current angle of joint               │
// │ HOME                      │ Move all joints to their home positions   │
// │ GRIP up/down/home         │ Set wrist pose (t4/t5)                    │
// │ GRIP open/close           │ Control gripper (t6)                      │
// │ STATUS                    │ Print all current joint angles            │
// │ REFRESH                   │ Update all current joint angles           │
// └───────────────────────────┴───────────────────────────────────────────┘
//
// Notes:
// - t1–t3 are stepper motors (MKS SERVO42C over UART)
// - t4–t6 are servo motors (PCA9685 over I2C)
// - Joint limits and RPM safety are enforced automatically
// - spacing matters
// ==========================================================================

void loop() {

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if      (input.startsWith("set "))      handleSetCommand(input);
    else if (input.startsWith("move "))     handleMoveCommand(input);
    else if (input.startsWith("arm "))      handleArmCommand(input);
    else if (input.startsWith("rpm "))      handleRPMCommand(input);
    else if (input.equals("home"))          homeAllJoints();
    else if (input.startsWith("grip "))     handleGripCommand(input);
    else if (input.startsWith("read "))     handleReadCommand(input);
    else if (input.equals("status"))        printStatus();
    else if (input.equals("refresh"))       refreshStepperAngles();
    else if (input.startsWith("movef6 "))   handleMoveF6Command(input);
    else if (input.startsWith("pid "))      handlePIDCommand(input);

    // === Custom pose commands ===
    else if (input.equals("center"))      moveToCenterPosition();
    else if (input.equals("left"))        moveToLeftPosition();
    else if (input.equals("far_left"))    moveToFarLeftPosition();
    else if (input.equals("right"))       moveToRightPosition();
    else if (input.equals("far_right"))   moveToFarRightPosition();

    else if (input.equals("pickup"))             moveToPickupPosition();
    else if (input.equals("move_to_spectrometer")) moveToSpectrometer();
    else if (input.startsWith("bin:")) {
      String type = input.substring(4);
      moveToBinPosition(type);
    }

    else printHelp();
  }
}

//////////////////////////////////////////////////////////////////
////       ========== MKS Servo42C FUNCTIONS ==========       ////
//////////////////////////////////////////////////////////////////
bool runMotorAtSpeed(uint8_t motorID, float rpm, bool ccw) {
  uint8_t speedByte = round(rpm / 9.375);
  if (speedByte > 127) speedByte = 127;

  uint8_t dirSpeed = ccw ? (speedByte & 0x7F) : (0x80 | (speedByte & 0x7F));  // bit 7 is direction
  uint8_t cmd[3] = { motorID, 0xF6, dirSpeed };

  // Calculate checksum
  uint8_t tchk = (cmd[0] + cmd[1] + cmd[2]) & 0xFF;

  // Flush leftover bytes
  while (SerialMKS.available()) SerialMKS.read();

  // Send command
  for (int i = 0; i < 3; i++) SerialMKS.write(cmd[i]);
  SerialMKS.write(tchk);

  // Debug print removed for cleaner runtime
  // Serial.printf("📤 Sent F6 command: 0x%02X 0x%02X 0x%02X 0x%02X\n", cmd[0], cmd[1], cmd[2], tchk);

  return true;
}
//////////////////////////////////////////////////////////////////
bool stopMotor(uint8_t motorID) {
  uint8_t cmd[2] = { motorID, 0xF7 };
  uint8_t tchk = (cmd[0] + cmd[1]) & 0xFF;

  // Flush junk before sending
  while (SerialMKS.available()) SerialMKS.read();

  SerialMKS.write(cmd[0]);
  SerialMKS.write(cmd[1]);
  SerialMKS.write(tchk);
  //Serial.printf("🛑 Sent STOP command: 0x%02X 0x%02X 0x%02X\n", cmd[0], cmd[1], tchk);

  uint8_t buf[3] = {0};
  unsigned long start = millis();

  while (millis() - start < 900) {
    if (SerialMKS.available()) {
      // Slide window
      buf[0] = buf[1];
      buf[1] = buf[2];
      buf[2] = SerialMKS.read();

      //Serial.printf("📥 Sliding: 0x%02X 0x%02X 0x%02X\n", buf[0], buf[1], buf[2]);

      // Check for valid STOP response
      if (buf[0] == motorID) {
        uint8_t status = buf[1];
        uint8_t chk = (buf[0] + buf[1]) & 0xFF;

        if (buf[2] == chk) {
          if (status == 0x01) {
            Serial.println("✅ Stop success");
            return true;
          } else {
            //Serial.println("❌ Stop failed (status = 0)");
            return false;
          }
        } else {
          //Serial.printf("❌ Bad checksum — got 0x%02X, expected 0x%02X\n", buf[2], chk);
        }
      }
    }
  }

  //Serial.println("❌ Stop timeout — no valid response");
  return false;
}

//////////////////////////////////////////////////////////////////
//moveJointWithF6UntilTarget(joints[0], 45.0, 20.0);  // move t1 to 45° at 20 RPM
bool moveJointWithF6UntilTarget(Joint& joint, float targetAngle, float rpm) {
  if (joint.type != STEPPER) return false;

  float gearRatio = joint.stepper.gearRatio;
  uint8_t motorID = joint.stepper.motorID;
  float minLimit = joint.minAngle;
  float maxLimit = joint.maxAngle;

  targetAngle = constrain(targetAngle, minLimit, maxLimit);

  delay(10);  // small pause before reading
  float currentAngle = readCurrentAngle(joint);
  if (currentAngle < -3000) return false;

  bool directionCCW = targetAngle < currentAngle;
  float error = targetAngle - currentAngle;

  // 🛠️ Send motor run command
runMotorAtSpeed(motorID, rpm, directionCCW);

// Ensure transmission complete + let driver spin up
SerialMKS.flush();
delay(150);
while (SerialMKS.available()) SerialMKS.read();  // Flush junk

// 🆕 Recalculate actual error after delay
currentAngle = readCurrentAngle(joint);
if (currentAngle < -900) return false;
error = targetAngle - currentAngle;
directionCCW = targetAngle < currentAngle;  // 🛠️ Recalculate direction

  // Loop until close to target
  const float tolerance = 0.5f;
  unsigned long timeout = millis() + 7000;

  while (millis() < timeout) {
    delay(20);
    currentAngle = readCurrentAngle(joint);
    if (currentAngle < -900) break;

    error = targetAngle - currentAngle;
    if (abs(error) <= tolerance) break;
  }

  stopMotor(motorID);
  delay(30);  // allow motor to fully stop
  delay(10);  // allow UART settle
  readCurrentAngle(joint);  // refresh state

  if (abs(error) <= tolerance) {
    //Serial.printf("✅ Moved %s to %.2f° using F6 (%.1f RPM)\n", joint.name, targetAngle, rpm);
    Serial.printf("finished\n");
    return true;
  } else {
    Serial.printf("❌ Failed to reach %.2f° — final err = %.2f°\n", targetAngle, error);
    return false;
  }
}
//////////////////////////////////////////////////////////////////
bool rotateMotorBy(uint8_t motorID, float angle_deg, bool ccw, float rpm) {
  uint32_t pulses = (angle_deg / 360.0f) * PULSES_PER_REV;
  uint8_t speed = round(rpm / 9.375f);
  if (speed > 127) speed = 127;

  uint8_t dir_speed = (ccw ? 0x00 : 0x80) | (speed & 0x7F);

  uint8_t cmd[7] = {
    motorID, 0xFD, dir_speed,
    (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
    (uint8_t)(pulses >> 8), (uint8_t)pulses
  };

  uint8_t checksum = 0;
  for (int i = 0; i < 7; i++) checksum += cmd[i];
  checksum &= 0xFF;

  // Flush serial buffer
  while (SerialMKS.available()) SerialMKS.read();

  // Send command
  for (int i = 0; i < 7; i++) SerialMKS.write(cmd[i]);
  SerialMKS.write(checksum);

  // Wait for ACKs: 0x01 = Start, 0x02 = Done
  uint8_t buf[3] = {0};
  bool gotStart = false, gotDone = false;
  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {
    if (SerialMKS.available()) {
      buf[0] = buf[1];
      buf[1] = buf[2];
      buf[2] = SerialMKS.read();

      //Serial.printf("📥 Received: 0x%02X 0x%02X 0x%02X\n", buf[0], buf[1], buf[2]);

      // Expect format: motorID, ACK, checksum
      if (buf[0] == motorID) {
        uint8_t ack = buf[1];
        uint8_t chk = (motorID + ack) & 0xFF;
        if (buf[2] == chk) {
          if (ack == 0x01) {
            gotStart = true;
            //Serial.println("✅ Got START ACK");
          } else if (ack == 0x02) {
            gotDone = true;
            //Serial.println("✅ Got DONE ACK");
          }
        }
      }

      if (gotStart && gotDone) {
        //Serial.printf("✅ Movement command for motorID 0x%02X successful.\n", motorID);
        return true;
      }
    }

    delay(1); // prevent watchdog reset
  }

  Serial.printf("❌ Movement command for motorID 0x%02X timed out.\n", motorID);
  return false;
}
//////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void goToAngle(const Joint& joint, float targetJointAngle) {
  float gearRatio = joint.stepper.gearRatio;
  uint8_t motorID = joint.stepper.motorID;
  float minLimit  = joint.minAngle;
  float maxLimit  = joint.maxAngle;
  float rpm       = joint.stepper.rpm;

  targetJointAngle = constrain(targetJointAngle, minLimit, maxLimit);
  float currentJointAngle = lastStepperAngles[joint.stepper.index];

  const float safetyMargin = 0.5f;

  // Check if current angle is out of bounds
  if (currentJointAngle < minLimit - safetyMargin || currentJointAngle > maxLimit + safetyMargin) {
    Serial.printf("⚠️ Current joint angle %.2f° is outside safe zone!\n", currentJointAngle);
    return;
  }

  // Skip movement if already within 0.5° of the target
  float delta = targetJointAngle - currentJointAngle;
  if (abs(delta) < 0.5f) {
    //Serial.printf("✅ %s already at %.2f° (Δ%.2f°) — no move needed.\n", joint.name, currentJointAngle, delta);
    return;
  }

  // Normalize delta to [-180, 180]
  if (delta > 180.0f) delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;

  float predicted = currentJointAngle + delta;
  if (predicted < minLimit - safetyMargin || predicted > maxLimit + safetyMargin) {
    Serial.printf("⛔ Movement to %.2f° exceeds safe limits. Aborted.\n", predicted);
    return;
  }

  bool direction = delta < 0;
  float moveAmount = abs(delta) * gearRatio;

  bool success = rotateMotorBy(motorID, moveAmount, direction, rpm);
  if (!success) {
    Serial.printf("❌ Movement command for %s timed out.\n", joint.name);
    return;
  }

  //Serial.printf("✅ Moved %s to %.2f° (%s %.2f°) at %.1f RPM\n", joint.name, targetJointAngle, direction ? "CCW" : "CW", abs(delta), rpm);
}
//////////////////////////////////////////////////////////////////
float readCurrentAngle(const Joint& joint)
{
  float gearRatio = joint.stepper.gearRatio;
  uint8_t motorID = joint.stepper.motorID;

  // ------- flush any leftovers -------
  while (SerialMKS.available()) {
    SerialMKS.read();
  }
  delay(10);  // Let UART settle

  // ------- send request -------
  uint8_t cmd1 = motorID, cmd2 = 0x36;
  uint8_t checksum = (cmd1 + cmd2) & 0xFF;
  SerialMKS.write(cmd1); SerialMKS.write(cmd2); SerialMKS.write(checksum);

  // ------- sliding buffer read -------
  uint8_t buf[6] = {0};
  unsigned long startTime = millis();

  while (millis() - startTime < 1000) {
    if (SerialMKS.available()) {
      // Shift buffer left
      for (int i = 0; i < 5; i++) {
        buf[i] = buf[i + 1];
      }
      buf[5] = SerialMKS.read();

      // Check for valid frame
      if (buf[0] == motorID) {
        uint8_t expectedChk = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) & 0xFF;
        if (expectedChk == buf[5]) {
          
          int16_t carry = (int16_t)((buf[1] << 8) | buf[2]);
          uint16_t value = (buf[3] << 8) | buf[4];

          float motorDeg = carry * 360.0f + (value / 65535.0f) * 360.0f;
          float jointDeg = motorDeg / gearRatio;

          lastStepperAngles[joint.stepper.index] = jointDeg;
          //Serial.printf("✅ Joint angle (%s): %.2f° (motor %.2f°)\n", joint.name, jointDeg, motorDeg);
          return jointDeg;
        } 
      }
    }

    delay(1);  // prevent busy wait
  }

  // ------- failure case -------
  Serial.println("❌ No valid response from MKS.");
  return -9999.0f;
}
//////////////////////////////////////////////////////////////////
////         ========== Servo PWM FUNCTIONS ==========        ////
//////////////////////////////////////////////////////////////////
void moveServo(const Joint& joint, float angle) {
  angle = constrain(angle, joint.minAngle, joint.maxAngle);
  lastServoAngles[joint.servo.channel] = angle;

  int pulse_us = map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE_US, MAX_PULSE_US);
  int pulse_steps = (int)((float)pulse_us / (1000000.0 / SERVO_FREQ / 4096.0));
  pwm.setPWM(joint.servo.channel, 0, pulse_steps);

  //Serial.printf("🔧 %s → %.1f° (%d µs = %d steps)\n",
    //            joint.name, angle, pulse_us, pulse_steps);
}
//////////////////////////////////////////////////////////////////
////      ========== STRUCT HELPER FUNCTIONS ==========       ////
//////////////////////////////////////////////////////////////////
Joint* getJointByName(const String& name) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (name.equalsIgnoreCase(joints[i].name)) return &joints[i];
  }
  return nullptr;
}
//////////////////////////////////////////////////////////////////
void printHelp() {
  Serial.println("Unrecognized command. Try:");
  for (int i = 0; i < NUM_COMMANDS; i++) {
    Serial.println(COMMAND_LIST[i]);
  }
}
//////////////////////////////////////////////////////////////////  
//Modify ESP32 firmware to return something structured like:
//STATUS t1:45 t2:90 t3:135 t4:90 t5:90 t6:0
void printStatus() {
  Serial.println("📊 Joint Status:");

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].type == STEPPER) {
      float angle = lastStepperAngles[i];
      Serial.printf("  %s: %.2f°\n", joints[i].name, angle); }
    else {
    uint8_t ch = joints[i].servo.channel;
    Serial.printf("  %s: %.1f° (servo)\n", joints[i].name, lastServoAngles[ch]);
    }
  }
}
//////////////////////////////////////////////////////////////////
void goJoint(Joint* joint, float angle) {
  if (!joint) {
    Serial.println("❌ Unknown joint. Use t1–t6.");
    return;
  }

  if (joint->type == STEPPER) {
    goToAngle(*joint, angle);
    delay(300);
    readCurrentAngle(*joint);  // Updates angle array
    delay(300);
  } else {
    moveServo(*joint, angle);
  }
}
////////////////////////////////////////////////////////////////// for testing only!!
void handleMoveCommand(const String& input) {
  int space1 = input.indexOf(' ', 5);
  if (space1 != -1) {
    String jointName = input.substring(5, space1);
    float delta = input.substring(space1 + 1).toFloat();

    Joint* joint = getJointByName(jointName);
    if (!joint || joint->type != STEPPER) {
      Serial.println("❌ Invalid joint or not a stepper.");
      return;
    }

    float moveAmount = abs(delta) * joint->stepper.gearRatio;
    bool dirCCW = delta < 0;
    rotateMotorBy(joint->stepper.motorID, moveAmount, dirCCW, joint->stepper.rpm);
    readCurrentAngle(*joint);  // Updates angle array
  } else {
    Serial.println("❌ Invalid format. Use: move <joint> <delta>");
  }
}
//////////////////////////////////////////////////////////////////
void goArm(float a1, float a2, float a3, float a4, float a5, float a6) {
  float angles[] = { a1, a2, a3, a4, a5, a6 };
  for (int i = 0; i < 6; i++) {
    goJoint(&joints[i], angles[i]);
  }
}
//////////////////////////////////////////////////////////////////
void handleRPMCommand(const String& input) {
  int space1 = input.indexOf(' ', 4);
  if (space1 != -1) {
    String jointName = input.substring(4, space1);
    float rpm = input.substring(space1 + 1).toFloat();

    Joint* joint = getJointByName(jointName);
    if (joint && joint->type == STEPPER) {
      joint->stepper.rpm = constrain(rpm, 1, 100);
      Serial.printf("✅ RPM for %s set to %.1f\n", jointName.c_str(), rpm);
    } else {
      Serial.println("❌ Invalid joint or not a stepper.");
    }
  } else {
    Serial.println("❌ Invalid format. Use: rpm <joint> <val>");
  }
}
//////////////////////////////////////////////////////////////////
void homeAllJoints() {
  goArm(0.0, 0.0, 0.0,
        joints[3].servo.homeAngle,
        joints[4].servo.homeAngle,
        joints[5].servo.homeAngle);
  
  Serial.println("Finished ✅ All motors homed.");
}
//////////////////////////////////////////////////////////////////
void handleReadCommand(const String& input) {
  String jointName = input.substring(5);
  jointName.trim();
  Joint* joint = getJointByName(jointName);

  if (!joint) {
    Serial.println("❌ Unknown joint. Use t1–t6.");
    return;
  }

  if (joint->type == STEPPER) {
    float angle = readCurrentAngle(*joint);
    Serial.printf("✅ Joint angle (%s): %.2f°\n", joint->name, angle);
  } else {
    float angle = lastServoAngles[joint->servo.channel];
    Serial.printf("🧭 %s servo angle: %.1f° (last commanded)\n", joint->name, angle);
  }
}
//////////////////////////////////////////////////////////////////
void handleArmCommand(const String& input) {
  float angles[6];
  int lastIndex = input.indexOf(' ');  // Start from first space

  for (int i = 0; i < 6; i++) {
    int spaceIndex = input.indexOf(' ', lastIndex + 1);
    String val = input.substring(lastIndex + 1, spaceIndex);
    angles[i] = val.toFloat();
    lastIndex = spaceIndex;
  }

  // Debug print
  //Serial.print("📐 Parsed angles: ");
  //for (int i = 0; i < 6; i++) {
  //  Serial.print(angles[i], 2);
  //  if (i < 5) Serial.print(", ");
  //}
  //Serial.println();

  goArm(angles[0], angles[1], angles[2],
        angles[3], angles[4], angles[5]);
  
  Serial.println("Finished\n");
}
//////////////////////////////////////////////////////////////////
void refreshStepperAngles() {
  Serial.println("\n🔄 Refreshing all stepper joint angles...");

  for (int i = 0; i < 3; i++) {
    float angle = readCurrentAngle(joints[i]);
    Serial.printf("📌 %s updated to %.2f°\n", joints[i].name, angle);
    delay(10);  // Delay between commands
  }

  Serial.println("✅ Stepper angle refresh complete.\n");
}

//////////////////////////////////////////////////////////////////
void handleSetCommand(const String& input) {
  int space1 = input.indexOf(' ', 4);
  if (space1 != -1) {
    String jointName = input.substring(4, space1);
    float angle = input.substring(space1 + 1).toFloat();

    Joint* joint = getJointByName(jointName);
    if (joint) {
      goJoint(joint, angle);
      Serial.println("Finished\n");
    }
    else Serial.println("❌ Unknown joint.");
  } else {
    Serial.println("Failed ❌ Invalid format. Use: set <joint> <angle>");
  }
}
////////////////////////////////////////////////////////////////
void handleGripCommand(const String& input) {
  String action = input.substring(5);
  action.trim();

  if (action == "up")         { moveServo(T4, 180);  moveServo(T5, 0); }
  else if (action == "down")  { moveServo(T4, 0); moveServo(T5, 180);  }
  else if (action == "home")  { moveServo(T4, joints[3].servo.homeAngle); moveServo(T5, joints[4].servo.homeAngle); } // fix to use struct home
  else if (action == "open")  { moveServo(T6, 0); }
  else if (action == "straight")  { moveServo(T4,90);  moveServo(T5, 90); }
  else if (action == "close") { moveServo(T6, 90); }
  else Serial.println("❌ Unknown grip command.");

  Serial.printf("Finished\n");
}
////////////////////////////////////////////////////////////////
void printWelcomeMessage() {
  Serial.println("ESP32-S3 Ready.");
  Serial.println("Available commands:");
  for (int i = 0; i < NUM_COMMANDS; i++) {
    Serial.println(COMMAND_LIST[i]);
  }
}
////////////////////////////////////////////////////////////////
//movef6 t1 30
void handleMoveF6Command(const String& input) {
  int space1 = input.indexOf(' ', 7);
  if (space1 != -1) {
    String jointName = input.substring(7, space1);
    float target = input.substring(space1 + 1).toFloat();

    Joint* joint = getJointByName(jointName);
    if (!joint || joint->type != STEPPER) {
      Serial.println("❌ Invalid joint or not a stepper.");
      return;
    }

    moveJointWithF6UntilTarget(*joint, target, joint->stepper.rpm);
  } else {
    Serial.println("❌ Invalid format. Use: movef6 <joint> <angle>");
  }
}

//////////////////////////////////////////////////////////
float computePID(PID& pid, float error, float dt) {
  pid.integral += error * dt;
  float derivative = (dt > 0) ? (error - pid.lastError) / dt : 0;
  pid.lastError = error;

  float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
  return abs(output);  // use magnitude for RPM
}
///////////////////////////////////////////////////////////
bool moveJointWithF6_PID(Joint& joint, float targetAngle) {
  if (joint.type != STEPPER) return false;

  uint8_t motorID = joint.stepper.motorID;
  float gearRatio = joint.stepper.gearRatio;
  float minLimit = joint.minAngle;
  float maxLimit = joint.maxAngle;

  targetAngle = constrain(targetAngle, minLimit, maxLimit);

  jointPID.integral = 0;
  jointPID.lastError = 0;

  delay(10);
  float currentAngle = readCurrentAngle(joint);
  if (currentAngle < -3000) return false;

  const float tolerance = 0.5f;
  unsigned long lastTime = millis();
  unsigned long timeout = millis() + 12000;

  while (millis() < timeout) {
    float error = targetAngle - currentAngle;

    if (abs(error) <= tolerance) break;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    float rpm = computePID(jointPID, error, dt);
    rpm = constrain(rpm, 5, 50);  // clamp RPM
    bool ccw = error < 0;

    runMotorAtSpeed(motorID, rpm, ccw);

    //Serial.printf("⚙️ PID move %s: curr=%.2f° targ=%.2f° err=%.2f° rpm=%.1f\n", joint.name, currentAngle, targetAngle, error, rpm);

    delay(100);
    currentAngle = readCurrentAngle(joint);
    if (currentAngle < -3000) break;
  }

  stopMotor(motorID);
  delay(100);
  currentAngle = readCurrentAngle(joint);

  float finalError = targetAngle - currentAngle;
  if (abs(finalError) <= tolerance) {
    Serial.printf("✅ PID Moved %s to %.2f°\n", joint.name, targetAngle);
    return true;
  } else {
    Serial.printf("❌ PID move failed — final err = %.2f°\n", finalError);
    return false;
  }
}
////////////////////////
void handlePIDCommand(String input) {
  String args = input.substring(4);
  int spaceIndex = args.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("⚠️ Usage: pid <joint> <angle>");
    return;
  }

  String jointStr = args.substring(0, spaceIndex);
  String angleStr = args.substring(spaceIndex + 1);
  float target = angleStr.toFloat();

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (jointStr.equalsIgnoreCase(joints[i].name)) {
      Serial.printf("🎯 PID moving %s to %.2f°...\n", joints[i].name, target);
      moveJointWithF6_PID(joints[i], target);
      return;
    }
  }

  Serial.printf("❌ Unknown joint name '%s'. Use t1–t3.\n", jointStr.c_str());
}

////////////////////////////////////////////////////////////////////////////////////
void moveToCenterPosition() {
  Serial.println("🟢 Moving to CENTER position...");
  goJoint(&T4, 180);
  goJoint(&T5, 0);
  goJoint(&T6, 0);  // gripper open
  goJoint(&T3, 39);
  goJoint(&T2, -75);
  goJoint(&T1, 0);
  goJoint(&T6, 90);  // gripper open

  Serial.printf("Finished\n");
}

void moveToLeftPosition() {
  Serial.println("🟢 Moving to LEFT position...");
  goJoint(&T4, 170);
  goJoint(&T5, 10);
  goJoint(&T6, 0);  // gripper open
  goJoint(&T3, 42);
  goJoint(&T2, -75);
  goJoint(&T1, -10);  // Adjust this value later
  goJoint(&T6, 90);  // gripper open
  Serial.printf("Finished\n");
}

void moveToFarLeftPosition() {
  Serial.println("🟢 Moving to FAR LEFT position...");
  goJoint(&T4, 160);
  goJoint(&T5, 20);
  goJoint(&T6, 0);  // gripper open
  goJoint(&T3, 50);
  goJoint(&T2, -75);
  goJoint(&T1, -20);  // Adjust this value later
  goJoint(&T6, 90);  // gripper open
  Serial.printf("Finished\n");
}

void moveToRightPosition() {
  Serial.println("🟢 Moving to RIGHT position...");
  goJoint(&T4, 170);
  goJoint(&T5, 10);
  goJoint(&T6, 0);  // gripper open
  goJoint(&T3, 42);
  goJoint(&T2, -75);
  goJoint(&T1, 10);  // Adjust this value later
  goJoint(&T6, 90);  // gripper open
  Serial.printf("Finished\n");
}

void moveToFarRightPosition() {
  Serial.println("🟢 Moving to FAR RIGHT position...");
  goJoint(&T4, 160);
  goJoint(&T5, 20);
  goJoint(&T6, 0);  // gripper open
  goJoint(&T3, 50);
  goJoint(&T2, -75);
  goJoint(&T1, 20);  // Adjust this value later
  goJoint(&T6, 90);  // gripper open
  Serial.printf("Finished\n");
}

void moveToPickupPosition() {
  Serial.println("🟢 Moving to ready position...");
  goJoint(&T4, 125);
  goJoint(&T5, 35);
  goJoint(&T6, 0);  // gripper open
  goJoint(&T3, 87);
  goJoint(&T2, -85);
  goJoint(&T1, 0);
  
  Serial.printf("Finished");

  //goJoint(&T6, 90);  // gripper close
}


void moveToSpectrometer() {
  Serial.println("🔬 Moving to spectrometer...");
  goJoint(&T2, -30); //move up
  goArm(90, -72, 50, 160, 30, 90);
}

void moveToBinPosition(const String& type) {
  Serial.printf("🗑️ Moving to %s bin...\n", type.c_str());
    goJoint(&T2, -30);  
  if (type == "metal" || type == "plastic") {
    goJoint(&T1, -75);  
  } else if (type == "paper") {
    goJoint(&T1, -90);   // t1
  } else {
    Serial.println("❌ Unknown bin type. Use bin:metal or bin:paper");
    return;
  }

  //goJoint(&T2, -40);    
  //goJoint(&T3, 100);   
  goJoint(&T2, -57);
  delay(500);
  goJoint(&T6, 0);  // gripper open

  goJoint(&T3, 30);    
  goJoint(&T2, -30);   
  homeAllJoints();
}

