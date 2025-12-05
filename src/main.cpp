#include "../include/commandLine.h"
#include "../include/servoConfig.h"
#include "../include/servoControl.h"

// Application state variables
uint16_t servo_LF2_pulselength = 335;
uint16_t pulselength = 335;
int angleStart = 20;
int angleEnd = 45;
int angleLoop;
bool angleLoopInc;
bool doLoop;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  initializeServoDriver();

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