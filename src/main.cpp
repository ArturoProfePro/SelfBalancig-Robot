#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

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

volatile int32_t encCount[2] = {0,0};
volatile uint8_t encState[2] = {0,0}; // two-bit state: (A<<1)|B

const unsigned long PID_INTERVAL_MS = 50;
int16_t motorCommandA = 0;
int16_t motorCommandB = 0;
unsigned long lastPidTime = 0;
unsigned long lastPrint = 0;

struct PID {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float lastError;
    float outMin;
    float outMax;
    unsigned long lastTime;

    PID(float Kp = 0, float Ki = 0, float Kd = 0, float minOut = -255, float maxOut = 255)
        : kp(Kp), ki(Ki), kd(Kd), setpoint(0), integral(0), lastError(0), outMin(minOut), outMax(maxOut), lastTime(0) {}

    void setTunings(float Kp, float Ki, float Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }

    void setOutputLimits(float minOut, float maxOut) {
        outMin = minOut;
        outMax = maxOut;
    }

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
        float derivative = (error - lastError) / dt;
        float output = kp * error + ki * integral + kd * derivative;
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
    pidBalance.setpoint = 0;

    Wire.begin();
    mpu.initialize();
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("MPU6050 DMP ready");
    } else {
        Serial.print("DMP init failed: ");
        Serial.println(devStatus);
    }
}

void loop() {
    unsigned long now = millis();
    float pitch = 0.0f;

    if (dmpReady) {
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
            fifoCount = 0;
        }

        if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) {
                fifoCount = mpu.getFIFOCount();
            }
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            Quaternion q;
            VectorFloat gravity;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            pitch = ypr[1] * 180.0f / 3.14159265f;
        }
    }

    float balanceOutput = pidBalance.compute(pitch, now);
    int16_t speed = constrain((int16_t)balanceOutput, -255, 255);
    motorCommandA = speed;
    motorCommandB = speed;
    setMotorA(speed);
    setMotorB(speed);

    if (now - lastPrint >= 200) {
        int32_t c0;
        int32_t c1;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            c0 = encCount[0];
            c1 = encCount[1];
        }
        Serial.print("Enc1: "); Serial.print(c0);
        Serial.print("\tEnc2: "); Serial.print(c1);
        Serial.print("\tPitch: "); Serial.print(pitch);
        Serial.print("\tCmd: "); Serial.println(speed);
        lastPrint = now;
    }
}