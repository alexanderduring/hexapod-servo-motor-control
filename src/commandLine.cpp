
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
    // Use int for indexOf results since they return -1 for "not found"
    const int posOpeningBracket = inputLine.indexOf('(');
    const int posComma = inputLine.indexOf(',');
    const int posClosingBracket = inputLine.indexOf(')');

    byte command = 0;  // byte is enough for command codes (0, 1, etc.)
    int paramOne = 0;  // int needed for angles which can be negative
    int paramTwo = 0;

    // Extract command string (everything before '(' or the whole string if no '(')
    String commandString;
    if (posOpeningBracket != -1) {
        commandString = inputLine.substring(0, posOpeningBracket);
    } else {
        commandString = inputLine;
    }

    // Parse command
    if (commandString == "stop") {
        command = ACTION_STOP;
        // stop command doesn't need parameters
        hasNewCommand = true;
    } else if (commandString == "loop") {
        // Validate that we have all required punctuation for loop(x,y)
        if (posOpeningBracket == -1 || posComma == -1 || posClosingBracket == -1) {
            Serial.println("Error: Invalid loop command format. Expected: loop(x,y)");
            hasNewCommand = false;
            return;
        }

        // Validate order of punctuation
        if (posOpeningBracket >= posComma || posComma >= posClosingBracket) {
            Serial.println("Error: Invalid loop command format. Expected: loop(x,y)");
            hasNewCommand = false;
            return;
        }

        command = ACTION_LOOP;
        paramOne = inputLine.substring(posOpeningBracket+1, posComma).toInt();
        paramTwo = inputLine.substring(posComma+1, posClosingBracket).toInt();
        hasNewCommand = true;
    } else {
        Serial.print("Error: Unknown command '");
        Serial.print(commandString);
        Serial.println("'. Valid commands: stop, loop(x,y)");
        hasNewCommand = false;
        return;
    }

    newCommand[0] = command;
    newCommand[1] = paramOne;
    newCommand[2] = paramTwo;
}
