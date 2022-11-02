#include <Arduino.h>
#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);

const byte SET = 32;
const byte UP = 33;
const byte DW = 25;
const byte CAN = 26;

void setup()
{
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  pinMode(SET, INPUT);
  pinMode(UP, INPUT);
  pinMode(DW, INPUT);
  pinMode(CAN, INPUT);
}

void loop()
{
  static boolean Display = true;
  static boolean ButtonSet = false;

  if (Display == true)
  {
    Display = false;
    lcd.setCursor(0, 0);
    lcd.print("HELLO LCD");
    lcd.setCursor(0, 1);
    lcd.print("BY MCU");
  }

  if (digitalRead(SET) == 0)
  {
    delay(1);
    if (digitalRead(SET) == 0)
    {
      if (SET == false)
      {
        ButtonSet = true;
        SelectMenu();
        Display = true;
      }
    }
  }
  else
  {
    ButtonSet = false;
  }
}

void SelectMenu (void)
{
  boolean Display = true;
  boolean Exit = false;
  boolean ButtonSet = true;
  boolean ButtonUp = false;
  boolean ButtonDw = false;
  boolean ButtonCan = false;

  byte Menu = 0;
  const char MenuText[4][21] = {
      " 1: Menu 1          ",
      " 2: Menu 2          ",
      " 3: Menu 3          ",
      " 4: Menu 4          ",
  };

  while (Exit == false)
  {
    if (Display == true)
    {
      Display = false;
      lcd.setCursor(0, 0);
      lcd.print("Select Menu 1-4");
      lcd.setCursor(0, 1);
      lcd.print(MenuText[Menu]);
    }
//-------------------------------------------- SET
    if (digitalRead(SET) == 0)
    {
      delay(1);
      if (digitalRead(SET) == 0)
      {
        if (ButtonSet == false)
        {
          ButtonSet = true;
          switch (Menu)
          {
          case 0: Serial.print("You Select Menu 1");break;
          case 1: Serial.print("You Select Menu 2");break;
          case 2: Serial.print("You Select Menu 3");break;
          case 3: Serial.print("You Select Menu 4");break;
          
          }
        }
      }
    }
    else
    {
      ButtonSet = false;
    }
//-------------------------------------------- UP
    if (digitalRead(UP) == 0)
    {
      delay(1);
      if (digitalRead(UP) == 0)
      {
        if (ButtonUp == false)
        {
          ButtonUp = true;
          if(Menu<3){
            Menu++;
          }else{
            Menu = 0;
          }
          Display = true;
        }
      }
    }
    else
    {
      ButtonUp = false;
    }
//-------------------------------------------- DW
    if (digitalRead(DW) == 0)
    {
      delay(1);
      if (digitalRead(DW) == 0)
      {
        if (ButtonDw == false)
        {
          ButtonDw = true;
          if(Menu>0){
            Menu--;
          }else{
            Menu = 3;
          }
          Display = true;

        }
      }
    }
    else
    {
      ButtonDw = false;
    }
//-------------------------------------------- Exit
    if (digitalRead(CAN) == 0)
    {
      delay(1);
      if (digitalRead(CAN) == 0)
      {
        if (ButtonCan == false)
        {
          ButtonCan = true;
          Exit = true;
        }
      }
    }
    else
    {
      ButtonCan = false;
    }
  }
}
