#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define IN1 11
#define IN2 12
#define ENA 10
#define IN3 7
#define IN4 8
#define ENB 9

#define MIN_PWM_A 39
#define MIN_PWM_B 35
#define RATE_LIMIT 20  // Макс прирост PWM за итерацию (для плавности и защиты от brown-out)
#define MAX_PWM 100
#define SETPOINT_ANGLE -96.24f
#define MAX_ANGLE_ERROR 50.0f

float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;

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
        : kp(Kp), ki(Ki), kd(Kd), setpoint(0), integral(0), integralMin(-100), integralMax(100), 
          lastError(0), outMin(minOut), outMax(maxOut), pTerm(0), iTerm(0), dTerm(0), lastTime(0) {}

    void setTunings(float Kp, float Ki, float Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }

    float compute(float input, unsigned long now) {
        float error = setpoint - input;
        if (lastTime == 0) {
            lastTime = now;
            lastError = error;
        }
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0) dt = 0.001f;

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

    float getP() const { return pTerm; }
    float getI() const { return iTerm; }
    float getD() const { return dTerm; }
};

PID pidBalance(10, 0, 0);

MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
float ypr[3];

unsigned long lastPrint = 0;
int16_t lastMotorPwmA = 0;
int16_t lastMotorPwmB = 0;

void setMotorA(int16_t speed) {
    if (speed > MAX_PWM) speed = MAX_PWM;
    if (speed < -MAX_PWM) speed = -MAX_PWM;
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
    if (speed > MAX_PWM) speed = MAX_PWM;
    if (speed < -MAX_PWM) speed = -MAX_PWM;
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

void printMenu() {
    Serial.println("\n=== Balance PID Test ===");
    Serial.println("KP=<v>   - set Kp");
    Serial.println("KI=<v>   - set Ki");
    Serial.println("KD=<v>   - set Kd");
    Serial.println("TARGET=<v> - set target pitch angle");
    Serial.println("P        - print current values");
    Serial.println("M        - print this menu");
    Serial.println("========================\n");
}

void handleSerialInput() {
    if (Serial.available() == 0) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    if (input == "P") {
        Serial.println("\n=== Status ===");
        Serial.print("Pitch: "); Serial.print(pitch);
        Serial.print("°  Target: "); Serial.print(pidBalance.setpoint); Serial.println("°");
        Serial.print("Output: "); Serial.println(pidBalance.compute(pitch, millis()));
        Serial.print("Kp="); Serial.print(pidBalance.kp);
        Serial.print(" Ki="); Serial.print(pidBalance.ki);
        Serial.print(" Kd="); Serial.println(pidBalance.kd);
        Serial.println("==============\n");
        return;
    }

    if (input == "M") {
        printMenu();
        return;
    }

    if (input.startsWith("KP=")) {
        pidBalance.kp = input.substring(3).toFloat();
        Serial.print("Kp = "); Serial.println(pidBalance.kp);
    } else if (input.startsWith("KI=")) {
        pidBalance.ki = input.substring(3).toFloat();
        Serial.print("Ki = "); Serial.println(pidBalance.ki);
    } else if (input.startsWith("KD=")) {
        pidBalance.kd = input.substring(3).toFloat();
        Serial.print("Kd = "); Serial.println(pidBalance.kd);
    } else if (input.startsWith("TARGET=")) {
        pidBalance.setpoint = input.substring(7).toFloat();
        Serial.print("Target pitch = "); Serial.println(pidBalance.setpoint);
    } else {
        Serial.print("Unknown: "); Serial.println(input);
        Serial.println("Send M for menu");
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    setMotorA(0);
    setMotorB(0);

    Wire.begin();
    mpu.initialize();
    
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("MPU6050 DMP ready");
        pidBalance.setpoint = SETPOINT_ANGLE;
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
            mpu.resetFIFO();
            fifoCount = 0;
        } else if (fifoCount >= packetSize) {
            while (fifoCount >= packetSize) {
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                fifoCount -= packetSize;
            }
            Quaternion q;
            VectorFloat gravity;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * 180.0f / M_PI;
            pitch = ypr[1] * 180.0f / M_PI;
            roll = ypr[2] * 180.0f / M_PI;

            int16_t output = (int16_t)pidBalance.compute(roll, now);
            
            // Rate limiting для плавности и защиты от brown-out
            if (output > lastMotorPwmA + RATE_LIMIT) {
                output = lastMotorPwmA + RATE_LIMIT;
            } else if (output < lastMotorPwmA - RATE_LIMIT) {
                output = lastMotorPwmA - RATE_LIMIT;
            }
            lastMotorPwmA = output;
            lastMotorPwmB = output;
            
            if(pidBalance.setpoint - roll > MAX_ANGLE_ERROR) {
                output = 0; 
            } else if(pidBalance.setpoint - roll < -MAX_ANGLE_ERROR) {
                output = 0;
            }
            setMotorA(output);
            setMotorB(output);
        }
    }

    handleSerialInput();

    if (now - lastPrint >= 200) {
        lastPrint = now;
        float error = pidBalance.setpoint - pitch;
        Serial.print(">pitch:"); Serial.print(pitch);
        Serial.print(",roll:"); Serial.print(roll);
        Serial.print(",yaw:"); Serial.print(yaw);

        Serial.print(",target:"); Serial.print(pidBalance.setpoint);
        Serial.print(",out:"); Serial.print((int16_t)pidBalance.compute(pitch, now));
        Serial.print(",p:"); Serial.print(pidBalance.getP());
        Serial.print(",i:"); Serial.print(pidBalance.getI());
        Serial.print(",d:"); Serial.print(pidBalance.getD());
        Serial.print(",err:"); Serial.println(error);
    }
}