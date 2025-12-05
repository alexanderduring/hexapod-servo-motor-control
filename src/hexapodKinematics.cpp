#include <Arduino.h>
#include "../include/servoConfig.h"
#include "../include/hexapodKinematics.h"

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
