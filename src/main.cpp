#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <EEPROM.h>

#define ENC1A 2
#define ENC1B 3
#define ENC2A 5
#define ENC2B 6

#define IN1 13
#define IN2 12
#define ENA 10
#define IN3 7
#define IN4 8
#define ENB 9

#define EEPROM_ADDR_PIDA 0
#define EEPROM_ADDR_PIDB 12
#define EEPROM_ADDR_PIDBALANCE 24

volatile int32_t encCount[2] = {0,0};
volatile uint8_t encState[2] = {0,0}; // two-bit state: (A<<1)|B

const unsigned long PID_INTERVAL_MS = 50;
const float ENCODER_PULSES_PER_REV = 20.0f;
const float ENCODER_COUNTS_PER_REV = ENCODER_PULSES_PER_REV * 4.0f;
const int MIN_PWM_A = 35;
const int MIN_PWM_B = 40;

float balanceTargetRPM = 0.0f;
float balanceOutput = 0.0f;
float encoderVelA = 0.0f;
float encoderVelB = 0.0f;
int32_t prevEncCountA = 0;
int32_t prevEncCountB = 0;
unsigned long lastMotorPidTime = 0;
bool newGyroSample = false;

int16_t motorCommandA = 0;
int16_t motorCommandB = 0;
unsigned long lastPidTime = 0;
unsigned long lastPrint = 0;
unsigned long lastSerialCheck = 0;

uint8_t tuneMode = 0; // 0=balance, 1=motorA, 2=motorB

struct PID {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float integralMin;
    float integralMax;
    float lastError;
    float outMin;
    float outMax;
    float pTerm;
    float iTerm;
    float dTerm;
    unsigned long lastTime;

    PID(float Kp = 0, float Ki = 0, float Kd = 0, float minOut = -255, float maxOut = 255)
        : kp(Kp), ki(Ki), kd(Kd), setpoint(0), integral(0), integralMin(-1000), integralMax(1000), lastError(0), outMin(minOut), outMax(maxOut), pTerm(0), iTerm(0), dTerm(0), lastTime(0) {}

    void setTunings(float Kp, float Ki, float Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }

    void setOutputLimits(float minOut, float maxOut) {
        outMin = minOut;
        outMax = maxOut;
    }

    void setIntegralLimits(float minI, float maxI) {
        integralMin = minI;
        integralMax = maxI;
    }

    float getP() const { return pTerm; }
    float getI() const { return iTerm; }
    float getD() const { return dTerm; }

    float compute(float input, unsigned long now) {
        float error = setpoint - input;
        if (lastTime == 0) {
            lastTime = now;
            lastError = error;
        }
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0) {
            dt = 0.001f;
        }
        integral += error * dt;
        if (integral > integralMax) integral = integralMax;
        if (integral < integralMin) integral = integralMin;
        float derivative = (error - lastError) / dt;
        pTerm = kp * error;
        iTerm = ki * integral;
        dTerm = kd * derivative;
        float output = pTerm + iTerm + dTerm;
        if (output > outMax) output = outMax;
        if (output < outMin) output = outMin;
        lastError = error;
        lastTime = now;
        return output;
    }

    void reset() {
        integral = 0;
        lastError = 0;
        lastTime = 0;
    }
};

PID pidA(1.0f, 0.02f, 0.005f, -255, 255);
PID pidB(1.0f, 0.02f, 0.005f, -255, 255);
PID pidBalance(0.8f, 0.02f, 0.02f, -255, 255);

MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
float ypr[3];
float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;
bool imuCalibrated = false;

// simple forward sequence: 00->01->11->10->00
bool isForwardTransition(uint8_t oldS, uint8_t newS) {
    return (oldS==0 && newS==1) || (oldS==1 && newS==3) || (oldS==3 && newS==2) || (oldS==2 && newS==0);
}

void updateEncoder(uint8_t idx, uint8_t newState) {
    uint8_t old = encState[idx];
    if (old == newState) return;
    if (isForwardTransition(old, newState)) {
        encCount[idx]++;
    } else if (isForwardTransition(newState, old)) {
        encCount[idx]--;
    }
    encState[idx] = newState;
}

// ISR for encoder 1 (pins 2 and 3) - use external interrupts on both pins
void isrEnc1() {
    uint8_t a = digitalRead(ENC1A);
    uint8_t b = digitalRead(ENC1B);
    updateEncoder(0, (a<<1) | b);
}

// PCINT for encoder 2 (pins 5 and 6 are on PORTD -> PCINT2_vect)
ISR(PCINT2_vect) {
    uint8_t a = digitalRead(ENC2A);
    uint8_t b = digitalRead(ENC2B);
    updateEncoder(1, (a<<1) | b);
}

// Motor control helpers: speed -255..255
void setMotorA(int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    if (speed > 0 && speed < MIN_PWM_A) speed = MIN_PWM_A;
    if (speed < 0 && speed > -MIN_PWM_A) speed = -MIN_PWM_A;
    if (speed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, speed);
    } else if (speed < 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -speed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
    }
}

void setMotorB(int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    if (speed > 0 && speed < MIN_PWM_B) speed = MIN_PWM_B;
    if (speed < 0 && speed > -MIN_PWM_B) speed = -MIN_PWM_B;
    if (speed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, speed);
    } else if (speed < 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, -speed);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 0);
    }
}

void savePIDToEEPROM(uint16_t addr, PID &pid) {
    EEPROM.put(addr, pid.kp);
    EEPROM.put(addr + 4, pid.ki);
    EEPROM.put(addr + 8, pid.kd);
}

void loadPIDFromEEPROM(uint16_t addr, PID &pid) {
    float kp, ki, kd;
    EEPROM.get(addr, kp);
    EEPROM.get(addr + 4, ki);
    EEPROM.get(addr + 8, kd);
    pid.setTunings(kp, ki, kd);
}

void printMenu() {
    Serial.println("\n=== PID Tuning Menu ===");
    Serial.println("0: Balance PID");
    Serial.println("1: Motor A PID");
    Serial.println("2: Motor B PID");
    Serial.println("S: Save to EEPROM");
    Serial.println("L: Load from EEPROM");
    Serial.println("P: Print current values");
    Serial.println("Usage: 0Kp=0.8 Ki=0.02 Kd=0.02");
    Serial.println("======================\n");
}

void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "P") {
            Serial.println("\n=== Current PID Values ===");
            Serial.print("Balance: Kp="); Serial.print(pidBalance.kp); Serial.print(" Ki="); Serial.print(pidBalance.ki); Serial.print(" Kd="); Serial.println(pidBalance.kd);
            Serial.print("MotorA:  Kp="); Serial.print(pidA.kp); Serial.print(" Ki="); Serial.print(pidA.ki); Serial.print(" Kd="); Serial.println(pidA.kd);
            Serial.print("MotorB:  Kp="); Serial.print(pidB.kp); Serial.print(" Ki="); Serial.print(pidB.ki); Serial.print(" Kd="); Serial.println(pidB.kd);
            Serial.println("===========================\n");
        } else if (input == "S") {
            savePIDToEEPROM(EEPROM_ADDR_PIDBALANCE, pidBalance);
            savePIDToEEPROM(EEPROM_ADDR_PIDA, pidA);
            savePIDToEEPROM(EEPROM_ADDR_PIDB, pidB);
            Serial.println("PID values saved to EEPROM\n");
        } else if (input == "L") {
            loadPIDFromEEPROM(EEPROM_ADDR_PIDBALANCE, pidBalance);
            loadPIDFromEEPROM(EEPROM_ADDR_PIDA, pidA);
            loadPIDFromEEPROM(EEPROM_ADDR_PIDB, pidB);
            Serial.println("PID values loaded from EEPROM\n");
        } else if (input.length() > 0 && isdigit(input[0])) {
            uint8_t mode = input[0] - '0';
            if (mode <= 2) {
                tuneMode = mode;
                Serial.print("Tune mode: ");
                Serial.println(mode == 0 ? "Balance" : (mode == 1 ? "Motor A" : "Motor B"));
                
                // Parse Kp, Ki, Kd
                if (input.indexOf("Kp=") >= 0) {
                    float kp = input.substring(input.indexOf("Kp=") + 3).toFloat();
                    if (tuneMode == 0) pidBalance.kp = kp;
                    else if (tuneMode == 1) pidA.kp = kp;
                    else pidB.kp = kp;
                    Serial.print("Kp set to: "); Serial.println(kp);
                }
                if (input.indexOf("Ki=") >= 0) {
                    float ki = input.substring(input.indexOf("Ki=") + 3).toFloat();
                    if (tuneMode == 0) pidBalance.ki = ki;
                    else if (tuneMode == 1) pidA.ki = ki;
                    else pidB.ki = ki;
                    Serial.print("Ki set to: "); Serial.println(ki);
                }
                if (input.indexOf("Kd=") >= 0) {
                    float kd = input.substring(input.indexOf("Kd=") + 3).toFloat();
                    if (tuneMode == 0) pidBalance.kd = kd;
                    else if (tuneMode == 1) pidA.kd = kd;
                    else pidB.kd = kd;
                    Serial.print("Kd set to: "); Serial.println(kd);
                }
                Serial.println();
            }
        } else if (input == "?" || input == "H") {
            printMenu();
        }
    }
}

void setup() {
    Serial.begin(115200);

    // encoders inputs
    pinMode(ENC1A, INPUT_PULLUP);
    pinMode(ENC1B, INPUT_PULLUP);
    pinMode(ENC2A, INPUT_PULLUP);
    pinMode(ENC2B, INPUT_PULLUP);

    // motors
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    // read initial states
    encState[0] = (digitalRead(ENC1A) << 1) | digitalRead(ENC1B);
    encState[1] = (digitalRead(ENC2A) << 1) | digitalRead(ENC2B);

    // attach external interrupts for encoder1 pins (2 and 3)
    attachInterrupt(digitalPinToInterrupt(ENC1A), isrEnc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1B), isrEnc1, CHANGE);

    // enable pin change interrupt only for PD5 and PD6 (digital pins 5 and 6)
    // PCMSK2 bits correspond to PCINT16..23 -> PD0..PD7. PD5 -> bit5, PD6 -> bit6
    PCMSK2 |= (1 << 5) | (1 << 6); // enable PCINT21 (PD5) and PCINT22 (PD6)
    PCICR |= (1 << PCIE2); // enable PCINT for port D

    // stop motors initially
    setMotorA(0);
    setMotorB(0);

    pidA.setpoint = 0;
    pidB.setpoint = 0;
    pidBalance.setpoint = 92.0f;
    pidA.setIntegralLimits(-200, 200);
    pidB.setIntegralLimits(-200, 200);
    pidBalance.setIntegralLimits(-100, 100);

    Wire.begin();
    
    // Load PID from EEPROM
    loadPIDFromEEPROM(EEPROM_ADDR_PIDBALANCE, pidBalance);
    loadPIDFromEEPROM(EEPROM_ADDR_PIDA, pidA);
    loadPIDFromEEPROM(EEPROM_ADDR_PIDB, pidB);
    
    lastMotorPidTime = millis();
    prevEncCountA = encCount[0];
    prevEncCountB = encCount[1];
    
    mpu.initialize();
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        imuCalibrated = true;
        pidBalance.setpoint = 0.0f;
        Serial.println("MPU6050 DMP ready");
        printMenu();
    } else {
        Serial.print("DMP init failed: ");
        Serial.println(devStatus);
    }
}

void loop() {
    unsigned long now = millis();
    
    if (dmpReady) {
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
            Serial.print("FIFO overflow, reset. count=");
            Serial.println(fifoCount);
            mpu.resetFIFO();
            fifoCount = 0;
        }

        if (fifoCount >= packetSize) {
            while (fifoCount >= packetSize) {
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                fifoCount -= packetSize;

                Quaternion q;
                VectorFloat gravity;
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                yaw = ypr[0] * 180.0f / 3.14159265f;
                pitch = ypr[1] * 180.0f / 3.14159265f;
                roll = ypr[2] * 180.0f / 3.14159265f;
                newGyroSample = true;
            }
        }
    }

    if (imuCalibrated && newGyroSample) {
        balanceOutput = pidBalance.compute(pitch, now);
        pidA.setpoint = balanceOutput;
        pidB.setpoint = balanceOutput;
        newGyroSample = false;
    }

    int16_t outA = motorCommandA;
    int16_t outB = motorCommandB;
    unsigned long motorDt = now - lastMotorPidTime;
    if (motorDt >= PID_INTERVAL_MS) {
        int32_t c0;
        int32_t c1;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            c0 = encCount[0];
            c1 = encCount[1];
        }

        float revA = float(c0 - prevEncCountA) / ENCODER_COUNTS_PER_REV;
        float revB = float(c1 - prevEncCountB) / ENCODER_COUNTS_PER_REV;
        encoderVelA = revA * (60000.0f / motorDt);
        encoderVelB = revB * (60000.0f / motorDt);
        prevEncCountA = c0;
        prevEncCountB = c1;

        outA = constrain((int16_t)pidA.compute(encoderVelA, now), -255, 255);
        outB = constrain((int16_t)pidB.compute(encoderVelB, now), -255, 255);
        motorCommandA = outA;
        motorCommandB = outB;
        lastMotorPidTime = now;
    }

    if (!imuCalibrated) {
        outA = 0;
        outB = 0;
    }

    setMotorA(outA);
    setMotorB(outB);

    handleSerialInput();

    if (now - lastPrint >= 200) {
        int32_t c0;
        int32_t c1;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            c0 = encCount[0];
            c1 = encCount[1];
        }
        float error = pidBalance.setpoint - pitch;

            Serial.print(">pitch:"); Serial.print(pitch);
        Serial.print(",bal:"); Serial.print(balanceOutput);
        Serial.print(",velA:"); Serial.print(encoderVelA);
        Serial.print(",velB:"); Serial.print(encoderVelB);
        Serial.print(",cmdA:"); Serial.print(outA);
        Serial.print(",cmdB:"); Serial.print(outB);
        Serial.print(",p:"); Serial.print(pidBalance.getP());
        Serial.print(",i:"); Serial.print(pidBalance.getI());
        Serial.print(",d:"); Serial.print(pidBalance.getD());
        Serial.print(",err:"); Serial.print(error);
        Serial.print(",enc1:"); Serial.print(c0);
        Serial.print(",enc2:"); Serial.println(c1);
        lastPrint = now;
    }
}