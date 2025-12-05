
#include <Arduino.h>
#include <HardwareSerial.h>
#include <WString.h>

#define ACTION_STOP 0
#define ACTION_LOOP 1

// Define the variables declared as extern in commandLine.h
const byte maxChars = 20;
char inputChars[maxChars];
bool _hasNewInput = false;
int newCommand[3];
int currentCommand[3];  // Contains the action (code) and parameters (2) which is currently executed
bool hasNewCommand = false;

bool hasNewInput() {
    return _hasNewInput;
}

String getInputLine() {
    _hasNewInput = false;
    return String(inputChars);
}

void readInputLine() {
    static byte index = 0;

    while (Serial.available() > 0 && _hasNewInput == false) {
        const char endMarker = '\n';
        const char receivedCharacter = Serial.read();

        if (receivedCharacter != endMarker) {
            inputChars[index] = receivedCharacter;
            index++;
        }

        if (receivedCharacter == endMarker || index == maxChars) {
            // Terminate string
            inputChars[index] = '\0';
            index = 0;
            _hasNewInput = true;
        }
    }
}

// Supported commands:
// loop(x,y) : newCommand = {1,x,y}
// stop      : newCommand = {0,}
void readNewCommand(String inputLine) {
    const byte posOpeningBracket = inputLine.indexOf('(');
    const byte posComma = inputLine.indexOf(',');
    const byte posClosingBracket = inputLine.indexOf(')');
    const String commandString = inputLine.substring(0, posOpeningBracket);
    int command = 0;
    const int paramOne = inputLine.substring(posOpeningBracket+1, posComma).toInt();
    const int paramTwo = inputLine.substring(posComma+1, posClosingBracket).toInt();

    if (commandString == "stop") {
        command = ACTION_STOP;
    } else if (commandString == "loop") {
        command = ACTION_LOOP;
    }

    newCommand[0] = command;
    newCommand[1] = paramOne;
    newCommand[2] = paramTwo;

    hasNewCommand = true;
}
