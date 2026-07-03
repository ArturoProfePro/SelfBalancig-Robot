#include <Arduino.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

SoftwareSerial btSerial(A0, A1);

MPU6050 mpu;

#define ENC1A 3
#define ENC1B 4
#define ENC2A 2
#define ENC2B 5

volatile long enc1 = 0;
volatile long enc2 = 0;

#define motorA1 12
#define motorA2 13
#define motorB1 7
#define motorB2 8
#define motorAPWM 11
#define motorBPWM 10

unsigned long lastLoopCheck = 0;
const unsigned long loopInterval = 10; // 100 Гц (dt = 0.01с)

static long prevEnc1 = 0;
static long prevEnc2 = 0;

#define MinPWM 35

// --- Базовая вертикаль ---
const float initialSetPoint = 10.5f;

// Ограничение максимального угла наклона, который может потребовать регулятор скорости
const float MAX_TARGET_ANGLE = 5.0f;

struct PID
{
    float kp, ki, kd;
    float setpoint;
    float integral;
    float integralMin, integralMax;
    float lastError;
    float outMin, outMax;
    unsigned long lastTime;

    PID(float Kp = 0, float Ki = 0, float Kd = 0, float minOut = -255, float maxOut = 255)
        : kp(Kp), ki(Ki), kd(Kd), setpoint(0), integral(0), integralMin(-100), integralMax(100), lastError(0), outMin(minOut), outMax(maxOut), lastTime(0) {}

    void SetSetpoint(float value) { setpoint = value; }

    float compute(float input, unsigned long now)
    {
        float error = setpoint - input;
        if (lastTime == 0)
        {
            lastTime = now;
            lastError = error;
        }
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0)
            dt = 0.010f;

        integral += error * dt;
        if (integral > integralMax)
            integral = integralMax;
        if (integral < integralMin)
            integral = integralMin;

        float derivative = (error - lastError) / dt;
        float output = (kp * error) + (ki * integral) + (kd * derivative);

        if (output > outMax)
            output = outMax;
        if (output < outMin)
            output = outMin;

        lastError = error;
        lastTime = now;
        return output;
    }
};

PID pidBalance(22.0f, 0.0f, 0.7f, -255, 255);

// ВНЕШНИЙ ПИД (PI_speed + Позиция): Медленный контур, управляющий скоростью и удержанием точки.
// Выдает целевой угол отклонения от вертикали. Тормозит накопление интеграла жесткими лимитами.
PID pidSpeed(0.5f, 0.0f, 0.0f, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);

float btTargetSpeed = 0.0f;
float btSteeringBias = 0.0f;
const float BT_SPEED_STEP = 5.0f;
const float BT_STEERING_STEP = 15.0f;
const float BT_SPEED_MAX = 40.0f;
const float BT_STEERING_MAX = 100.0f;
const unsigned long BT_COMMAND_TIMEOUT = 500; // 1

long targetEncoderTicks = 0;
long startEncoderPos = 0;
char btCommandBuffer[10];
int btCommandIndex = 0;
bool btDistanceActive = false;

const long WASD_DISTANCE = 5;

int setSpeedA(int speed)
{
    int target_PWM = 0;
    if (speed > 0)
    {
        target_PWM = map(speed, 0, 255, MinPWM, 255);
        digitalWrite(motorA1, false);
        digitalWrite(motorA2, true);
    }
    else if (speed < 0)
    {
        target_PWM = map(speed, -255, 0, -255, -MinPWM);
        digitalWrite(motorA1, true);
        digitalWrite(motorA2, false);
    }
    else
    {
        target_PWM = 0;
    }
    analogWrite(motorAPWM, abs(target_PWM));
    return target_PWM;
}

int setSpeedB(int speed)
{
    int target_PWM = 0;
    if (speed > 0)
    {
        target_PWM = map(speed, 0, 255, MinPWM, 255);
        digitalWrite(motorB1, false);
        digitalWrite(motorB2, true);
    }
    else if (speed < 0)
    {
        target_PWM = map(speed, -255, 0, -255, -MinPWM);
        digitalWrite(motorB1, true);
        digitalWrite(motorB2, false);
    }
    else
    {

        target_PWM = 0;
    }
    analogWrite(motorBPWM, abs(target_PWM));
    return target_PWM;
}

void processBluetoothCommands()
{
    while (btSerial.available())
    {
        int incoming = btSerial.read();
        if (incoming < 0)
            continue;

        char cmd = (char)incoming;
        if (cmd >= 'A' && cmd <= 'Z')
            cmd += 'a' - 'A';

        if (cmd >= '0' && cmd <= '9')
        {
            if (btCommandIndex < 9)
            {
                btCommandBuffer[btCommandIndex++] = cmd;
            }
        }
        else if (cmd == '\r' || cmd == '\n')
        {
            if (btCommandIndex > 0)
            {
                btCommandBuffer[btCommandIndex] = '\0';
                targetEncoderTicks = atol(btCommandBuffer);
                startEncoderPos = (enc1 + enc2) / 2;
                btDistanceActive = true;
                btTargetSpeed = 0;
                btSteeringBias = 0;
                btSerial.print(F("Target: "));
                btSerial.println(targetEncoderTicks);
                btCommandIndex = 0;
            }
        }
        else if (cmd == 'c')
        {
            btCommandIndex = 0;
            memset(btCommandBuffer, 0, 10);
        }
        else
        {
            switch (cmd)
            {
            case 'w':
                if (!btDistanceActive)
                {
                    startEncoderPos = (enc1 + enc2) / 2;
                    targetEncoderTicks = WASD_DISTANCE;
                    btTargetSpeed = BT_SPEED_MAX * 0.1f;
                    btDistanceActive = true;
                }
                break;
            case 's':
                if (!btDistanceActive)
                {
                    startEncoderPos = (enc1 + enc2) / 2;
                    targetEncoderTicks = WASD_DISTANCE;
                    btTargetSpeed = -BT_SPEED_MAX * 0.1f;
                    btDistanceActive = true;
                }
                break;
            case 'a':
                if (!btDistanceActive)
                {
                    startEncoderPos = (enc1 + enc2) / 2;
                    targetEncoderTicks = WASD_DISTANCE;
                    btSteeringBias = -BT_STEERING_MAX;
                    btTargetSpeed = BT_SPEED_MAX * 0.1f;
                    btDistanceActive = true;
                }
                break;
            case 'd':
                if (!btDistanceActive)
                {
                    startEncoderPos = (enc1 + enc2) / 2;
                    targetEncoderTicks = WASD_DISTANCE;
                    btSteeringBias = BT_STEERING_MAX;
                    btTargetSpeed = BT_SPEED_MAX * 0.1f;
                    btDistanceActive = true;
                }
                break;
            case 'x':
                btTargetSpeed = 0;
                btSteeringBias = 0;
                btDistanceActive = false;
                targetEncoderTicks = 0;
                break;
            default:
                break;
            }
        }
    }
}

void flag1()
{
    if (digitalRead(ENC1A) == digitalRead(ENC1B))
        enc1++;
    else
        enc1--;
}
void flag2()
{
    if (digitalRead(ENC2A) == digitalRead(ENC2B))
        enc2--;
    else
        enc2++;
}

uint8_t fifoBuffer[45];

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000UL);

    pinMode(motorAPWM, OUTPUT);
    pinMode(motorBPWM, OUTPUT);
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);

    pinMode(ENC1A, INPUT_PULLUP);
    pinMode(ENC1B, INPUT_PULLUP);
    pinMode(ENC2A, INPUT_PULLUP);
    pinMode(ENC2B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1A), flag1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2A), flag2, RISING);

    btSerial.begin(9600);
    btSerial.println(F("BT Ready"));

    mpu.initialize();
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);

        pidSpeed.integralMin = -2.0f;
        pidSpeed.integralMax = 2.0f;
        pidSpeed.SetSetpoint(0);

        Serial.println(F("DMP Готов!"));
    }
    else
    {
        Serial.print(F("Ошибка DMP: "));
        Serial.println(devStatus);
    }
}

float pitch;
float actualAveragedSpeed = 0.0f;
float positionError = 0.0f;

void loop()
{
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        pitch = ypr[2] * RAD_TO_DEG;
    }

    unsigned long now = millis();

    processBluetoothCommands();

    if (now - lastLoopCheck >= loopInterval)
    {
        noInterrupts();
        long enc1Copy = enc1;
        long enc2Copy = enc2;
        interrupts();

        long currentEncoderAvg = (enc1Copy + enc2Copy) / 2;
        long distanceTraveled = abs(currentEncoderAvg - startEncoderPos);

        // Проверка достижения целевого расстояния
        if (btDistanceActive && targetEncoderTicks > 0)
        {
            if (distanceTraveled >= targetEncoderTicks)
            {
                btTargetSpeed = 0;
                btSteeringBias = 0;
                btDistanceActive = false;
                btSerial.println(F("Done"));
            }
        }

        long currentSpeed = (enc1Copy + enc2Copy) - (prevEnc1 + prevEnc2);

        actualAveragedSpeed = (actualAveragedSpeed * 0.92f) + ((float)currentSpeed * 0.08f);

        positionError += actualAveragedSpeed;
        positionError = constrain(positionError, -100.0f, 100.0f);

        pidSpeed.SetSetpoint(btTargetSpeed);
        float speedInput = actualAveragedSpeed + (positionError * 0.005f);
        float targetAngleOffset = pidSpeed.compute(speedInput, now);

        float finalSetpoint = initialSetPoint + targetAngleOffset;
        pidBalance.SetSetpoint(finalSetpoint);

        float totalPWM = pidBalance.compute(pitch, now);

        if (abs(pitch) >= 70)
        {
            setSpeedA(0);
            setSpeedB(0);
            noInterrupts();
            enc1 = 0;
            enc2 = 0;
            interrupts();
            pidBalance.integral = 0;
            pidSpeed.integral = 0;
            positionError = 0.0f;
            actualAveragedSpeed = 0.0f;
        }
        else
        {
            int leftPWM = constrain((int)(totalPWM + btSteeringBias), -255, 255);
            int rightPWM = constrain((int)(totalPWM - btSteeringBias), -255, 255);
            setSpeedA(leftPWM);
            setSpeedB(rightPWM);
        }

        Serial.print(">pitch:");
        Serial.print(pitch);
        Serial.print(",averagedSpeed:");
        Serial.print(actualAveragedSpeed);
        Serial.print(",positionError:");
        Serial.print(positionError);
        Serial.print(",targetAngleOffset:");
        Serial.print(targetAngleOffset);
        Serial.print(",setPoint:");
        Serial.print(pidBalance.setpoint);
        Serial.print(",PWM:");
        Serial.println(totalPWM);

        prevEnc1 = enc1Copy;
        prevEnc2 = enc2Copy;
        lastLoopCheck = now;
    }
}