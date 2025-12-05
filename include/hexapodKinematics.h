#ifndef HEXAPOD_SERVO_MOTOR_CONTROL_HEXAPODKINEMATICS_H
#define HEXAPOD_SERVO_MOTOR_CONTROL_HEXAPODKINEMATICS_H

// ============================================================================
// Hexapod Kinematics Module
// ============================================================================
// This module handles the three-layer angle transformation:
// 1. Joint angle → Servo angle (mechanical calibration)
// 2. Servo angle → Pulse width (servo-specific calibration)
// 3. Pulse width → PWM driver
//
// Servo angle definition:
// We define a servo angle of 0° when the two screws form a line which is
// perpendicular to the servo casing.
// ============================================================================

// ============================================================================
// Function Declarations
// ============================================================================

// Restrict servo angle based on joint type mechanical limits
// Parameters:
//   angleServo - The desired servo angle in degrees
//   jointType - The joint type (JOINT_TYPE_L2, etc.)
// Returns: Constrained angle within safe mechanical limits
double restrictServoAngle(const double angleServo, const int jointType);

// Transform servo angle into pulse width for a specific joint
// Parameters:
//   angleServo - The servo angle in degrees (0° = perpendicular to casing)
//   joint - The joint number (0-17)
// Returns: Pulse width value for PWM driver (out of 4096)
int transformServoAngleIntoPulseWidth(const double angleServo, const int joint);

// Calculate pulse width from joint angle (main transformation function)
// This orchestrates the full three-layer transformation:
// 1. Converts joint angle to servo angle (mechanical mapping)
// 2. Applies safety restrictions
// 3. Converts servo angle to pulse width
// Parameters:
//   angle - The desired joint angle in degrees
//   joint - The joint number (0-17)
// Returns: Pulse width value for PWM driver (out of 4096)
int calculatePulseWidth(const double angle, const int joint);

#endif //HEXAPOD_SERVO_MOTOR_CONTROL_HEXAPODKINEMATICS_H
