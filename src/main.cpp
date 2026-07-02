#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(6, 11); // RX, TX

MPU6050 mpu;

#define ENC1A 2
#define ENC1B 4
#define ENC2A 3
#define ENC2B 5

volatile long enc1 = 0;
volatile long enc2 = 0;

#define motorA1 12
#define motorA2 13
#define motorB1 7
#define motorB2 8
#define motorAPWM 10
#define motorBPWM 9

unsigned long lastLoopCheck = 0;
const unsigned long loopInterval = 10;
const unsigned long loopIntervalForControl = 100; // 100 Гц

static long prevEnc1 = 0;
static long prevEnc2 = 0;

#define MinPWM 35

// --- Переменные для адаптивного сетпоинта ---
const float initialSetPoint = 6.9f; // ПРОВЕРЬ ЭТОТ УГОЛ РУКАМИ!

float smoothedPWM = 0.0f;

const int targetEnc = 0;

const float K_pos = 0.002f;  // Насколько сильно робот хочет вернуться в начальную точку
const float K_speed = 0.05f; // Насколько сильно робот гасит свою скорость (аналог Kd для колес)

unsigned long controlTimer = 500;       // Таймер для сброса
const unsigned long controlDelay = 500; // Промежуток времени (0.5 сек)
bool isControlActive = false;           // Флаг, что управление сейчас активно

// Твои переменные добавки к ШИМ
int AddPWMA = 0;
int AddPWMB = 0;

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
            dt = 0.005f; // Защита от нуля

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

// Выход ПИДа расширен до стандартных лимитов ШИМ (-255, 255), так как функции маппинга ожидают этот диапазон
PID pidBalance(20.0f, 0.1f, 0.6f, -255, 255);
// PID pidEncoder(0.1f, 0.1f, 0.0f, -2.5f, 2.5f);
PID pidPosition(0.2f, 0.0f, 0.0f, -5.0f, 5.0f);

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
    BTSerial.begin(9600); // 9600 для стабильности SoftwareSerial
    Wire.begin();
    Wire.setClock(400000UL); // 400 кГц

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

    mpu.initialize();
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);
        pidBalance.SetSetpoint(initialSetPoint);
        Serial.println(F("DMP Готов!"));
    }
    else
    {
        Serial.print(F("Ошибка DMP: "));
        Serial.println(devStatus);
        while (1)
            ;
    }
}

float pitch;

void intervalControl()
{
    // Если управление не запускали, ничего не делаем
    if (!isControlActive)
        return;

    // Если с момента запуска прошло больше 500 мс
    if (millis() - controlTimer >= controlDelay)
    {
        AddPWMA = 0;
        AddPWMB = 0;
        isControlActive = false; // Выключаем таймер до следующего нажатия
        Serial.println(F("Время вышло: AddPWM сброшен в 0"));
    }
}
void loop()
{
    // 1. Чтение Bluetooth команд (Управление через смещение энкодеров)
    if (BTSerial.available() > 0)
    {
        char key = BTSerial.read();

        // Смещаем энкодеры, чтобы внешний ПИД думал, что мы ушли с точки, и вел робота за собой
        if (key == 'W')
        {
            noInterrupts();
            enc1 -= 50;
            enc2 -= 50;
            interrupts();
            controlTimer = millis();
            isControlActive = true;
        }
        else if (key == 'S')
        {
            noInterrupts();
            enc1 += 50;
            enc2 += 50;
            interrupts();
            controlTimer = millis();
            isControlActive = true;
        }
        else if (key == 'A')
        {
            noInterrupts();
            enc1 -= 15;
            enc2 += 15;
            interrupts();
            controlTimer = millis();
            isControlActive = true;
        }
        else if (key == 'D')
        {
            noInterrupts();
            enc1 += 15;
            enc2 -= 15;
            interrupts();
            controlTimer = millis();
            isControlActive = true;
        }
    }

    // 2. Получение угла от MPU6050
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
    intervalControl();

    // 3. Главный контур управления (100 Гц)
    if (now - lastLoopCheck >= loopInterval)
    {
        // Атомарно забираем тики одометрии
        noInterrupts();
        long enc1Copy = enc1;
        long enc2Copy = enc2;
        interrupts();

        // Текущая точка робота (среднее или сумма тиков)
        long currentPosition = enc1Copy + enc2Copy;

        // --- ВНЕШНИЙ ПИД (Позиция -> Целевой угол) ---
        // Цель внешнего ПИДа всегда 0 (удерживать координату старта)
        pidPosition.SetSetpoint(targetEnc);

        // Считаем, на какой угол нужно отклониться, чтобы вернуться в targetEnc
        // На выходе получаем значение от -5.0 до 5.0 (так как мы ограничили outMin/outMax)
        float angleCorrection = pidPosition.compute(currentPosition, now);

        // Корректируем базовый балансировочный угол
        float targetSetPoint = initialSetPoint - angleCorrection;

        // --- ВНУТРЕННИЙ ПИД (Угол -> ШИМ моторов) ---
        pidBalance.SetSetpoint(targetSetPoint);
        float totalPWM = pidBalance.compute(pitch, now);

        // Проверка на аварийное падение (угол > 70 градусов)
        if (abs(pitch) >= 70)
        {
            setSpeedA(0);
            setSpeedB(0);
            // Обнуляем энкодеры и интегратор при падении, чтобы робот не бесился в руках
            noInterrupts();
            enc1 = 0;
            enc2 = 0;
            interrupts();
            pidPosition.integral = 0;
            pidBalance.integral = 0;
        }
        else
        {
            // Передаем чистый скорректированный ПИДом сигнал на моторы
            setSpeedA(totalPWM);
            setSpeedB(totalPWM);
        }

        // Вывод данных в Serial Monitor / Teleplot
        Serial.print(">pitch:");
        Serial.print(pitch);
        Serial.print(",position:");
        Serial.print(currentPosition);
        Serial.print(",angleCorrection:");
        Serial.print(angleCorrection);
        Serial.print(",PWM:");
        Serial.print(totalPWM);
        Serial.print(",setPoint:");
        Serial.println(pidBalance.setpoint);

        prevEnc1 = enc1Copy;
        prevEnc2 = enc2Copy;
        lastLoopCheck = now;
    }
}