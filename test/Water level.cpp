#include <Arduino.h>
#define VCC_IN 12
#define SIGNAL_PIN 36
#define SENSOR_MIN 0
#define SENSOR_MAX 1650

int VALUE = 0;
int LEVEL = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(VCC_IN, OUTPUT);
    digitalWrite(VCC_IN, LOW);
}

void loop()
{
    digitalWrite(VCC_IN, HIGH);
    delay(10);
    VALUE = analogRead(SIGNAL_PIN);
    digitalWrite(VCC_IN, LOW);
    LEVEL = map(VALUE, SENSOR_MIN, SENSOR_MAX, 0, 4);

    Serial.print(VALUE);
    delay(500);
}


