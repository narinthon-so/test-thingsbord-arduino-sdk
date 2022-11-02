#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 13 //กำหนดว่าขาของเซนเซอร์ 18B20 ต่อกับขา 13
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorWaterTemp(&oneWire);

void setup(void) {
  Serial.begin(115200);
  Serial.println("Test Temperature 18B20");
  sensorWaterTemp.begin();
}

void loop(void) {
  sensorWaterTemp.requestTemperatures(); //สั่งอ่านค่าอุณหภูมิ
  Serial.print(sensorWaterTemp.getTempCByIndex(0)); // แสดงค่าอุณหภูมิ
  Serial.println(" *C");
  delay(1000);
}