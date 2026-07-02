#include <Arduino.h>

#define motorA1 12
#define motorA2 13
#define motorB1 7
#define motorB2 8
#define motorAPWM 10
#define motorBPWM 9

void setup()
{
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);
    pinMode(motorAPWM, OUTPUT);
    pinMode(motorBPWM, OUTPUT);
    Serial.begin(9600);
}
void loop()
{

    Serial.println("Motor A forward");
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    analogWrite(motorAPWM, 255);
    delay(2000);

    Serial.println("Motor A backward");
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    analogWrite(motorAPWM, 255);
    delay(2000);

    Serial.println("Motor B forward");
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    analogWrite(motorBPWM, 255);
    delay(2000);

    Serial.println("Motor B backward");
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    analogWrite(motorBPWM, 255);
    delay(2000);
}
