#include <Arduino.h>

/*
 * gbg29: Automated Toilet Lid Closing System
 * Adafruit Feather ESP32 V2
 *
 * Uses:
 * - Built-in SW38 button (GPIO 38)
 * - Potentiometer on A0
 * - TT motor with L9110 motor driver (A1, A2)
 *
 * Operation:
 * 1. Press built-in button.
 * 2. Check if lid is open (>5 degrees).
 * 3. Extend push rod to close lid.
 * 4. Monitor lid movement for obstructions.
 * 5. Retract push rod after closing.
 * 6. Confirm lid closed before flushing.
 *
 * NOTE: Flush actuator not yet implemented.
 */

// gbg29: Define inputs
//const int potentiometerPin = A0;
const int buttonPin = 38;  // Built-in SW38 button

// gbg29: Define TT motor outputs
const int motorPin1 = A1;
const int motorPin2 = A2;

// gbg29: Lid considered closed at 5 degrees or less
const int closedAngle = 5;

// gbg29: Motor timing (adjust after testing)
const unsigned long maxExtendTime = 10000;
const unsigned long retractTime = 3000;
const unsigned long noMovementTimeout = 1000;
const unsigned long debounceTime = 50;

// gbg29: Minimum angle change to count as movement
const int movementThreshold = 2;

// gbg29: Define system operating states
enum SystemState {
    IDLE,
    CLOSING,
    RETRACTING,
    READY_TO_FLUSH,
    ERROR_STATE
};

SystemState currentState = IDLE;

// gbg29: Track timing and movement
unsigned long motorStartTime = 0;
unsigned long lastMovementTime = 0;

int previousAngle = 0;

// gbg29: Button debounce variables
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// gbg29: Track whether closing succeeded
bool closingSuccessful = false;

// gbg29: get angle from gyro

// gbg29: Extend push rod
void motorForward() {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
}

// gbg29: Retract push rod
void motorReverse() {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
}

// gbg29: Stop TT motor
void motorOff() {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
}

// gbg29: Detect a single button press
bool buttonPressed() {

    bool reading = digitalRead(buttonPin);

    if (reading != lastButtonReading) {
        lastDebounceTime = millis();
    }

    if (millis() - lastDebounceTime >= debounceTime) {

        if (reading != stableButtonState) {

            stableButtonState = reading;

            if (stableButtonState == LOW) {
                lastButtonReading = reading;
                return true;
            }
        }
    }

    lastButtonReading = reading;
    return false;
}

// gbg29: Begin closing toilet lid
void startClosing() {

    Serial.println("Closing lid...");

    previousAngle = getSensorAngle();

    lastMovementTime = millis();
    motorStartTime = millis();

    closingSuccessful = false;
    currentState = CLOSING;

    motorForward();
}

// gbg29: Begin retracting push rod
void startRetracting() {

    Serial.println("Retracting push rod...");

    motorOff();

    motorStartTime = millis();
    currentState = RETRACTING;

    motorReverse();
}

// gbg29: Stop motor if an error occurs
void stopWithError(const char* message) {

    motorOff();

    Serial.print("ERROR: ");
    Serial.println(message);

    closingSuccessful = false;
    currentState = ERROR_STATE;
}

// gbg29: Placeholder for future flushing mechanism
void flushToilet() {

    motorOff();

    if (getSensorAngle() <= closedAngle) {

        Serial.println("Lid confirmed closed.");
        Serial.println("FLUSH COMMAND (not implemented)");

    } else {

        Serial.println("Flush blocked: lid is open.");
    }

    currentState = IDLE;
}

void setup() {

    Serial.begin(115200);

    // gbg29: Built-in SW38 button has pull-up resistor
    pinMode(buttonPin, INPUT);

    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);

    motorOff();

    // gbg29: ESP32 uses 12-bit ADC
    analogReadResolution(12);

    Serial.println("Automated Toilet System Ready");
}

void loop() {

    unsigned long currentTime = millis();
    int sensorAngle = getSensorAngle();

    bool pressed = buttonPressed();

    // gbg29: Start system only when button is pressed
    if (pressed && currentState == IDLE) {

        Serial.println("Button pressed!");

        if (sensorAngle > closedAngle) {

            startClosing();

        } else {

            Serial.println("Lid already closed.");
            currentState = READY_TO_FLUSH;
        }
    }

    switch (currentState) {

        case IDLE:

            // gbg29: Wait for button press
            break;

        case CLOSING: {

            // gbg29: Stop extending once lid closes
            if (sensorAngle <= closedAngle) {

                Serial.println("Lid successfully closed!");

                closingSuccessful = true;
                startRetracting();
                break;
            }

            // gbg29: Detect lid angle movement
            if (abs(sensorAngle - previousAngle)
                >= movementThreshold) {

                lastMovementTime = currentTime;
                previousAngle = sensorAngle;
            }

            // gbg29: Detect possible obstruction
            if (currentTime - lastMovementTime
                >= noMovementTimeout) {

                stopWithError(
                    "No lid movement detected. Possible obstruction."
                );

                break;
            }

            // gbg29: Prevent motor running too long
            if (currentTime - motorStartTime
                >= maxExtendTime) {

                stopWithError(
                    "Maximum closing time exceeded."
                );

                break;
            }

            break;
        }

        case RETRACTING:

            // gbg29: Reverse motor for calibrated duration
            if (currentTime - motorStartTime
                >= retractTime) {

                motorOff();

                Serial.println("Retraction complete (timed).");

                if (closingSuccessful) {

                    currentState = READY_TO_FLUSH;

                } else {

                    currentState = IDLE;
                }
            }

            break;

        case READY_TO_FLUSH:

            flushToilet();
            break;

        case ERROR_STATE:

            // gbg29: Motor remains stopped after error
            if (pressed) {

                Serial.println("Error acknowledged.");
                Serial.println("Check and reset mechanism.");

                currentState = IDLE;
            }

            break;
    }

    // gbg29: Print potentiometer angle every 250 ms
    static unsigned long lastPrintTime = 0;

    if (currentTime - lastPrintTime >= 250) {

        Serial.print("Angle: ");
        Serial.print(sensorAngle);

        Serial.print(" | State: ");
        Serial.println(currentState);

        lastPrintTime = currentTime;
    }
}
