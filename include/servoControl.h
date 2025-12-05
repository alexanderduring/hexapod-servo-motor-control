#ifndef HEXAPOD_SERVO_MOTOR_CONTROL_SERVOCONTROL_H
#define HEXAPOD_SERVO_MOTOR_CONTROL_SERVOCONTROL_H

// ============================================================================
// Servo Control Module
// ============================================================================
// This module provides the hardware interface layer for controlling servos
// via the Adafruit PWM Servo Driver. It abstracts the hardware details and
// provides a high-level interface for setting joint angles.
// ============================================================================

// ============================================================================
// Function Declarations
// ============================================================================

// Initialize the servo driver hardware
// Sets up I2C communication and configures PWM frequency
// Should be called once during setup()
void initializeServoDriver();

// Set the angle for a specific joint
// This is the main high-level interface for servo control
// Parameters:
//   angle - The desired joint angle in degrees
//   joint - The joint number (0-17)
// Handles:
//   - Error checking for invalid joints
//   - Warning for unmapped servos
//   - Angle to pulse width conversion via kinematics
//   - PWM driver communication
void setAngle(const double angle, const int joint);

#endif //HEXAPOD_SERVO_MOTOR_CONTROL_SERVOCONTROL_H
