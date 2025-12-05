#include <Adafruit_PWMServoDriver.h>
#include "../include/commandLine.h"
#include "../include/servoConfig.h"

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

// Application state variables
uint16_t servo_LF2_pulselength = 335;
uint16_t pulselength = 335;
int angleStart = 20;
int angleEnd = 45;
int angleLoop;
bool angleLoopInc;
bool doLoop;

double restrictServoAngle(const double angleServo, const int jointType) {
  double restrictedAngle = angleServo;

  //Serial.print("Checking restrictions for joint type ");
  //Serial.println(jointType);

  switch (jointType) {
    case JOINT_TYPE_L2:
      //Serial.println("Checking restrictions for L2 servo.");
      restrictedAngle = constrain(angleServo, -45, 45);
      break;
    case JOINT_TYPE_L3:
      //Serial.println("Checking restrictions for L3 servo.");
      restrictedAngle = constrain(angleServo, -30, 45);
      break;
    default:
      //Serial.println("No restrictions applied.");
      break;
  }

  return restrictedAngle;
}

int transformServoAngleIntoPulseWidth(const double angleServo, const int joint) {
  int pulseWidth;

  switch (joint) {
    //case JOINT_LF1:
    //  break;
    case JOINT_LF2:
      if (angleServo == 0) {
        pulseWidth = SERVO_LF2_PULSEWIDTH_HORIZONTAL;
      } else {
        pulseWidth = SERVO_LF2_PULSEWIDTH_HORIZONTAL + angleServo * SERVO_LF2_PULSEWIDTH_PER_DEGREE;
      }
      break;
    case JOINT_LF3:
      if (angleServo == 0) {
        pulseWidth = SERVO_LF3_PULSEWIDTH_HORIZONTAL;
      } else {
        pulseWidth = SERVO_LF3_PULSEWIDTH_HORIZONTAL + angleServo * SERVO_LF3_PULSEWIDTH_PER_DEGREE;
      }
      break;
    default:
      Serial.println("Unknown joint.");
      pulseWidth = 333; // ~ 0°
      break;
  }

  return pulseWidth;
}

int calculatePulseWidth(const double angle, const int joint) {

  // We define a servo angle of 0° when the two screws form a line which is perpendicular to the servo casing.
  // This is true at a pulse width = SERVO_LEFT_FRONT_THREE_HORIZONTAL

  double angleServo = 0;  // Initialize to prevent undefined behavior

  int jointType = getJointType(joint);

  // Transform joint angle into servo angle
  switch (jointType) {
    case JOINT_TYPE_L2:
      // joint angle 45° -> servo angle 0°
      // joint angle inc -> servo angle inc
      // servo angle = joint angle - 45°
      angleServo = angle - 45;
      break;
    case JOINT_TYPE_L3:
      // joint angle 45° -> servo angle 0°
      // joint angle inc -> servo angle inc
      // servo angle = joint angle - 45°
      angleServo = angle - 45;
      break;
    default:
      // TODO: Implement angle calculations for other joint types (R1, R2, R3, L1)
      Serial.print("Warning: No angle calculation implemented for joint type ");
      Serial.println(jointType);
      angleServo = 0;  // Safe default
      break;
  }

  // Security measures
  angleServo = restrictServoAngle(angleServo, jointType);
  //Serial.print("Restricted servo angle to ");
  //Serial.println(angleServo);

  // Transform angleServo into pulseWidth
  const int pulseWidth = transformServoAngleIntoPulseWidth(angleServo, joint);

  return pulseWidth;
}

void setAngle(const double angle, const int joint) {
  const int servoNumber = getServoNumber(joint);

  // Check if getServoNumber returned an error
  if (servoNumber < 0) {
    Serial.println("Error: Cannot set angle for invalid joint");
    return;
  }

  // Check if servo is mapped (99 means unmapped)
  if (servoNumber == 99) {
    Serial.print("Warning: Joint ");
    Serial.print(joint);
    Serial.println(" is not mapped to a servo");
    return;
  }

  const int pulseWidth = calculatePulseWidth(angle, joint);
  servoDriver.setPWM(servoNumber, 0, pulseWidth);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  servoDriver.begin();
  servoDriver.setPWMFreq(50);
  delay(10);

  currentCommand[0] = ACTION_STOP;
  currentCommand[1] = 0;
  currentCommand[2] = 0;

  angleLoop = angleStart;
  angleLoopInc = true;
  doLoop = false;

  // Position servo somewhere in the middle
  setAngle(angleLoop, JOINT_LF2);
  setAngle(angleLoop, JOINT_LF3);

  Serial.println("Hexapod Servo Control Via Serial Monitor Version 2");
  Serial.println("Send commands via Serial Monitor: loop(x,y) or stop.");
}

void loop() {
  readInputLine();

  if (doLoop == true) {
    if (angleLoopInc == true) {
      if (angleLoop < angleEnd) {
        angleLoop++;
      }
      if (angleLoop >= angleEnd) {
        angleLoop = angleEnd;
        angleLoopInc = false;
      }
    } else {
      if (angleLoop > angleStart) {
        angleLoop--;
      }
      if (angleLoop <= angleStart) {
        angleLoop = angleStart;
        angleLoopInc = true;
      }
    }

    setAngle(angleLoop, JOINT_LF2);
    setAngle(angleLoop, JOINT_LF3);
  }

  if (hasNewInput() == true) {
    const String inputLine = getInputLine();
    Serial.print("Received new input: ");
    Serial.println(inputLine);

    readNewCommand(inputLine);
    Serial.print("Read new command: {");
    Serial.print(newCommand[0]);
    Serial.print(",");
    Serial.print(newCommand[1]);
    Serial.print(",");
    Serial.print(newCommand[2]);
    Serial.println("}");

    if (hasNewCommand) {
      switch (newCommand[0]) {
        case ACTION_STOP:
          if (currentCommand[0] == ACTION_LOOP) {
            doLoop = false;
            currentCommand[0] = newCommand[0];
            currentCommand[1] = newCommand[1];
            currentCommand[2] = newCommand[2];
          } else {
            Serial.print("Command ");
            Serial.print(currentCommand[0]);
            Serial.println(" cannot be executed.");
          }
          break;
        case ACTION_LOOP:
          if (currentCommand[0] == ACTION_STOP) {
            doLoop = true;
            currentCommand[0] = newCommand[0];
            currentCommand[1] = newCommand[1];
            currentCommand[2] = newCommand[2];
          } else {
            Serial.print("Command ");
            Serial.print(currentCommand[0]);
            Serial.println(" cannot be executed.");
          }
          break;
        default:
          Serial.print("Command ");
          Serial.print(currentCommand[0]);
          Serial.println(" cannot be executed.");
          break;
      }
      hasNewCommand = false;
    }

    // Find comma
//    byte indexComma = inputLine.indexOf(',');
//    double degreeTwo = inputLine.substring(0, indexComma).toDouble();
//    double degreeThree = inputLine.substring(indexComma+1).toDouble();
//    Serial.print("Parsed input: Degree Two = ");
//    Serial.print(degreeTwo);
//    Serial.print(", Degree Three = ");
//    Serial.print(degreeThree);
//    Serial.println();

//    setAngle(degreeTwo, JOINT_LF2);
//    setAngle(degreeThree, JOINT_LF3);

  }

  delay(10);
}