#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "../include/servoConfig.h"
#include "../include/hexapodKinematics.h"
#include "../include/servoControl.h"

// Create the PWM servo driver instance
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

void initializeServoDriver() {
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);
  delay(10);  // Allow time for initialization
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
