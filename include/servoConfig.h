#ifndef HEXAPOD_SERVO_MOTOR_CONTROL_SERVOCONFIG_H
#define HEXAPOD_SERVO_MOTOR_CONTROL_SERVOCONFIG_H

// ============================================================================
// PWM Servo Driver Configuration
// ============================================================================

// The Adafruit PWM Servo Driver has a resolution of 4096 per pulse.
// Given a pwm frequency of 50 Hz, you get 50 pulses per second
// One pulse takes 20 ms or 20.000 µs
// The Servo Driver resolution splits those into 20.000 µs / 4096 = 4.8828125 µs steps
// which we can use to configure our pulse width:
// A setting of e.g. 190 corresponds to a pulse width of 190 steps * 4.88 µs = 928 µs

#define SERVOMIN  300 // 150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Most analog servos are optimized for a pwm frequency of ~50 Hz.

// ============================================================================
// Servo Hardware Specifications
// ============================================================================

// Hitec HS-300: Pulse Width: 900-2100 µs over 20ms
// Increase of pulse width turns the wheel clockwise

// ============================================================================
// Per-Servo Calibration Constants
// ============================================================================

// Left Front Leg - Joint 2
#define SERVO_LF2_PULSEWIDTH_MIN 238             // -45 degrees
#define SERVO_LF2_PULSEWIDTH_HORIZONTAL 335      //   0 degrees
#define SERVO_LF2_PULSEWIDTH_MAX 432             //  45 degrees clockwise
#define SERVO_LF2_PULSEWIDTH_PER_DEGREE 2.15555

// Left Front Leg - Joint 3
#define SERVO_LF3_PULSEWIDTH_MIN 278             // -45 degrees
#define SERVO_LF3_PULSEWIDTH_HORIZONTAL 335      //   0 degrees
#define SERVO_LF3_PULSEWIDTH_MAX 428             //  45 degrees clockwise
#define SERVO_LF3_PULSEWIDTH_PER_DEGREE 2

// ============================================================================
// Joint Definitions
// ============================================================================

// Joint numbers for left front leg
#define JOINT_LF1 9
#define JOINT_LF2 10
#define JOINT_LF3 11

// ============================================================================
// Joint Type Definitions
// ============================================================================

// Joint types define the mechanical configuration and movement constraints
#define JOINT_TYPE_R1 0  // Right leg, joint 1
#define JOINT_TYPE_R2 1  // Right leg, joint 2
#define JOINT_TYPE_R3 2  // Right leg, joint 3
#define JOINT_TYPE_L1 3  // Left leg, joint 1
#define JOINT_TYPE_L2 4  // Left leg, joint 2
#define JOINT_TYPE_L3 5  // Left leg, joint 3

// ============================================================================
// Function Declarations
// ============================================================================

// Get the joint type for a given joint number (0-17)
// Returns: Joint type constant (JOINT_TYPE_R1, etc.) or -1 on error
int getJointType(int joint);

// Get the PWM channel number for a given joint number (0-17)
// Returns: PWM channel (0-15), 99 for unmapped, or -1 on error
int getServoNumber(int joint);

#endif //HEXAPOD_SERVO_MOTOR_CONTROL_SERVOCONFIG_H
