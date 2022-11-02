#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(0x27,20,4); 

int sw1 = 32 ; //mode
int swState1 = 0;

int sw2 = 33 ; //pump1
int swState2 = 0;

int sw3 = 25 ; //pump2
int swState3 = 0;

int sw4 = 26 ; //food
int swState4 = 0;

int sw5 = 27 ; //vit
int swState5 = 0;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  pinMode(sw3, INPUT);
  pinMode(sw4, INPUT);
  pinMode(sw5, INPUT);

  lcd.clear();
  lcd.setCursor(4,1);
  lcd.print("THIS IS THE");
  lcd.setCursor(3,2);
  lcd.print("MENU TUTORIAL");
  delay(5000);
  lcd.clear();

}

void loop() {
    
  swState1 = digitalRead(sw1);
  Serial.print("Mode : ");
  Serial.print(swState1);
  Serial.print("  ");

  swState2 = digitalRead(sw2);
  Serial.print("Pump1 : ");
  Serial.print(swState2);
  Serial.print("  ");

  swState3 = digitalRead(sw3);
  Serial.print("Pump2 : ");
  Serial.print(swState3);
  Serial.print("  ");

  swState4 = digitalRead(sw4);
  Serial.print("Food : ");
  Serial.print(swState4);
  Serial.print("  ");

  swState5 = digitalRead(sw5);
  Serial.print("Vitamin : ");
  Serial.println(swState5);
  Serial.print("  ");
  
  delay(300);

  /*if (digitalRead(sw1) == 1) {
    if (swState1 == 1) {
      digitalWrite(led, 1);
      swState = 1;
    } else {
      digitalWrite(led, 0);
      swState = 0;
    }*/
  }
