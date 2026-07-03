#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(5, 6); // RX, TX

void setup()
{
    Serial.begin(115200);
    BTSerial.begin(9600);
    Serial.println("Тест Bluetooth запущен. Отправь что-нибудь с телефона...");
}

void loop()
{
    if (BTSerial.available() > 0)
    {
        char c = BTSerial.read();
        Serial.print("Получено: ");
        Serial.println(c);
    }
}