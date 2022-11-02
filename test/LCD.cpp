//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("WATER LEVEL: 100 %");
  //lcd.setCursor(13,0);
  //lcd.print("100%");

  lcd.setCursor(0, 1);
  lcd.print("TEMP: 32 Celsius");
 //lcd.setCursor(9,1);
 //lcd.print("°C");

  lcd.setCursor(0, 2);
  lcd.print("TDS: 150 PPM");

  lcd.setCursor(0, 3);
  lcd.print("PH: ");

  

}


void loop()
{
}
