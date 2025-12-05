#ifndef HEXAPOD_SERVO_MOTOR_CONTROL_COMMANDLINE_H
#define HEXAPOD_SERVO_MOTOR_CONTROL_COMMANDLINE_H

#include <WString.h>

#define ACTION_STOP 0
#define ACTION_LOOP 1

extern const byte maxChars;
extern char inputChars[];
extern int newCommand[3];
extern int currentCommand[3];  // Contains the action (code) and parameters (2) which is currently executed
extern bool hasNewCommand;

bool hasNewInput();

String getInputLine();

void readInputLine();

void readNewCommand(String);

#endif //HEXAPOD_SERVO_MOTOR_CONTROL_COMMANDLINE_H