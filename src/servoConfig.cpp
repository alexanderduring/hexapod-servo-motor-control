#include <Arduino.h>
#include "../include/servoConfig.h"

int getJointType(int joint) {
  // Bounds checking to prevent buffer overflow
  if (joint < 0 || joint >= 18) {
    Serial.print("Error: Invalid joint number ");
    Serial.print(joint);
    Serial.println(". Valid range: 0-17");
    return -1;  // Return error code
  }

  int jointTypes[18] = {
    JOINT_TYPE_R1,JOINT_TYPE_R2,JOINT_TYPE_R3, // Right front
    JOINT_TYPE_R1,JOINT_TYPE_R2,JOINT_TYPE_R3, // Right middle
    JOINT_TYPE_R1,JOINT_TYPE_R2,JOINT_TYPE_R3, // Right back
    JOINT_TYPE_L1,JOINT_TYPE_L2,JOINT_TYPE_L3, // Left front
    JOINT_TYPE_L1,JOINT_TYPE_L2,JOINT_TYPE_L3, // Left middle
    JOINT_TYPE_L1,JOINT_TYPE_L2,JOINT_TYPE_L3  // Left back
  };

  return jointTypes[joint];
}

int getServoNumberOnDriver(int joint) {
  // Bounds checking to prevent buffer overflow
  if (joint < 0 || joint >= 18) {
    Serial.print("Error: Invalid joint number ");
    Serial.print(joint);
    Serial.println(". Valid range: 0-17");
    return -1;  // Return error code
  }

  return JOINT_TO_DRIVER_CHANNEL[joint];
}
