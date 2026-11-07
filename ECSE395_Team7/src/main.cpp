#include <Arduino.h>
#include <Wire.h>

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

//ecs181: Define pins for gyro
const int SDA_PIN = 21;
const int SCL_PIN = 22;
#define MPU_ADDR 0x68;
const int USE_DEFAULT_PINS = 1; //ecs181 181 = use the board's default I2C pins (Wire.begin()), 0 = use SDA_PIN/SCL_PIN above
const int I2C_CLOCK = 400000;   //ecs181 try 100000 if you still see errors
const int PRINT_EVERY = 1000;

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

//ecs181: Kallmam filter code variables

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
uint8_t PrintCounter = 0;
uint32_t LastErrorPrint = 0;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004f*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004f * 0.004f * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

// ecs181: Restart the I2C peripheral (recovers from a stuck/timed-out bus)
void i2c_start(void) {
#if USE_DEFAULT_PINS
  Wire.begin();
#else
  Wire.begin(SDA_PIN, SCL_PIN);
#endif
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeOut(100);   // ms
}

void i2c_restart(void) {
  Wire.end();
  delay(5);
  i2c_start();
}

// ecs 181: endTransmission() codes: 0 ok, 1 too long, 2 NACK on address, 3 NACK on data, 4 other, 5 timeout
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  uint8_t err = Wire.endTransmission();
  if (err) {
    Serial.printf("writeReg(0x%02X) failed, endTransmission error %u\n", reg, err);
    return false;
  }
  return true;
}

// ecs181: Reads len bytes starting at reg, using a repeated start between the register-select write and the read (standard MPU6050 access pattern)
bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err) {
    if (millis() - LastErrorPrint > 500) {
      Serial.printf("readRegs(0x%02X): select failed, endTransmission error %u\n", reg, err);
      LastErrorPrint = millis();
    }
    return false;
  }
  uint8_t got = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len);
  if (got != len) {
    if (millis() - LastErrorPrint > 500) {
      Serial.printf("readRegs(0x%02X): wanted %u bytes, got %u\n", reg, len, got);
      LastErrorPrint = millis();
    }
    return false;
  }
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ecs181: Returns false if the read failed (values are left unchanged in that case)
bool gyro_signals(void) {
  uint8_t acc[6], gyro[6];
  if (!readRegs(0x3B, acc, 6) || !readRegs(0x43, gyro, 6)) {
    i2c_restart();
    return false;
  }
  int16_t AccXLSB = (int16_t)((acc[0] << 8) | acc[1]);
  int16_t AccYLSB = (int16_t)((acc[2] << 8) | acc[3]);
  int16_t AccZLSB = (int16_t)((acc[4] << 8) | acc[5]);
  int16_t GyroX   = (int16_t)((gyro[0] << 8) | gyro[1]);
  int16_t GyroY   = (int16_t)((gyro[2] << 8) | gyro[3]);
  int16_t GyroZ   = (int16_t)((gyro[4] << 8) | gyro[5]);

  RateRoll=(float)GyroX/65.5f;
  RatePitch=(float)GyroY/65.5f;
  RateYaw=(float)GyroZ/65.5f;
  AccX=(float)AccXLSB/4096.0f;
  AccY=(float)AccYLSB/4096.0f;
  AccZ=(float)AccZLSB/4096.0f;
  AngleRoll=atanf(AccY/sqrtf(AccX*AccX+AccZ*AccZ))*RAD_TO_DEG;
  AnglePitch=-atanf(AccX/sqrtf(AccY*AccY+AccZ*AccZ))*RAD_TO_DEG;
  return true;
}

void setup() {

    Serial.begin(115200);

    //ecs181: SETUP FOR ANGLE MEASUREMENT
    delay(3000);   // gives you time to open the serial monitor and see the startup messages
    i2c_start();
    #if USE_DEFAULT_PINS
        Serial.printf("I2C using board default pins: SDA=%d SCL=%d\n", (int)SDA, (int)SCL);
    #else
        Serial.printf("I2C using pins: SDA=%d SCL=%d\n", SDA_PIN, SCL_PIN);
    #endif
        delay(250);
    
    //ecs181: Diagnostic: is the sensor answering, and what is it?
    uint8_t who = 0;
    if (readRegs(0x75, &who, 1)) {
        Serial.printf("WHO_AM_I = 0x%02X (expected 0x68 for MPU6050; some clones differ)\n", who);
    } else {
        Serial.println("Could not read WHO_AM_I - see error code above");
    }

    //ecs181: Wake up, then configure: LPF ~10 Hz, accel +/-8g, gyro +/-500 deg/s
    writeReg(0x6B, 0x00);
    delay(100);
    writeReg(0x1A, 0x05);
    writeReg(0x1C, 0x10);
    writeReg(0x1B, 0x08);
    delay(50);

    //ecs181: ecs181: Gyro calibration: average 2000 good samples (give up after 6000 attempts)
    int good = 0;
    for (int attempts = 0; good < 2000 && attempts < 6000; attempts++) {
        if (gyro_signals()) {
        RateCalibrationRoll+=RateRoll;
        RateCalibrationPitch+=RatePitch;
        RateCalibrationYaw+=RateYaw;
        good++;
        }
        delay(1);
    }
    if (good > 0) {
        RateCalibrationRoll/=good;
        RateCalibrationPitch/=good;
        RateCalibrationYaw/=good;
    }
    Serial.printf("Calibration done with %d good samples\n", good);
    LoopTimer=micros();

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
    //ecs181: Gyro Update
    if (gyro_signals()) {
        RateRoll-=RateCalibrationRoll;
        RatePitch-=RateCalibrationPitch;
        RateYaw-=RateCalibrationYaw;
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        KalmanAngleRoll=Kalman1DOutput[0]; 
        KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
        KalmanAnglePitch=Kalman1DOutput[0]; 
        KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

        if (++PrintCounter >= PRINT_EVERY) {
        PrintCounter = 0;
        Serial.print("Roll Angle [°] ");
        Serial.print(KalmanAngleRoll);
        Serial.print(" Pitch Angle [°] ");
        Serial.println(KalmanAnglePitch);
        }
    }
    
    unsigned long currentTime = millis();
    //ecs181: depending on application change to pitch maybe
    int sensorAngle = KalmanAngleRoll;

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
