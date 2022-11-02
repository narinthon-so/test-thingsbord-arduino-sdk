#include <Arduino.h>
#include "main.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network
  uint8_t WIFI_TIMEOUT = 0;
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    WIFI_TIMEOUT++;
    if (WIFI_TIMEOUT == 20)
    {
      digitalWrite(LED_WIFI_STATUS, LOW);
      Serial.println("fialed to connect wifi");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED)
    digitalWrite(LED_WIFI_STATUS, HIGH);
  Serial.println("Connected to AP");
}

void printDateTime(const RtcDateTime &dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  Serial.print(datestring);
}

void InitRTC()
{
  Serial.print("Init RTC \ncompiled: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);

  //--------RTC SETUP ------------
  // if you are using ESP-01 then uncomment the line below to reset the pins to
  // the available pins for SDA, SCL
  // Wire.begin(0, 2); // due to limited pins, use pin 0 and 2 for SDA, SCL

  Rtc.Begin();
  RtcEeprom.Begin();

  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial.println();
  // Rtc.SetDateTime(compiled);

  if (!Rtc.IsDateTimeValid())
  {
    if (Rtc.LastError() != 0)
    {
      // we have a communications error
      // see https://www.arduino.cc/en/Reference/WireEndTransmission for
      // what the number means
      Serial.print("RTC communications error = ");
      Serial.println(Rtc.LastError());
    }
    else
    {
      Serial.println("RTC lost confidence in the DateTime!");
      Rtc.SetDateTime(compiled);
    }
  }

  if (!Rtc.GetIsRunning())
  {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled)
  {
    Serial.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(compiled);
  }

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);

  /* comment out on a second run to see that the info is stored long term */
  // Store something in memory on the Eeprom

  // store starting address of string
  RtcEeprom.SetMemory(0, stringAddr);
  // store the string, nothing longer than 32 bytes due to paging
  uint8_t written = RtcEeprom.SetMemory(stringAddr, (const uint8_t *)data, sizeof(data) - 1); // remove the null terminator strings add
  // store the length of the string
  RtcEeprom.SetMemory(1, written); // store the
  /* end of comment out section */
}

void checkRTC()
{
  if (!Rtc.IsDateTimeValid())
  {
    if (Rtc.LastError() != 0)
    {
      // we have a communications error
      // see https://www.arduino.cc/en/Reference/WireEndTransmission for
      // what the number means
      Serial.print("RTC communications error = ");
      Serial.println(Rtc.LastError());
    }
    else
    {
      // Common Causes:
      //    1) the battery on the device is low or even missing and the power line was disconnected
      Serial.println("RTC lost confidence in the DateTime!");
    }
  }
  // Serial.println();

  // RtcTemperature temp = Rtc.GetTemperature();
  // temp.Print(Serial);
  // // you may also get the temperature as a float and print it
  // // Serial.print(temp.AsFloatDegC());
  // Serial.println("C");
}

void reconnect()
{
  // Loop until we're reconnected
  status = WiFi.status();
  if (status != WL_CONNECTED)
  {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}

RPC_Response processSetMode(const RPC_Data &data)
{
  Serial.println("Received the setMode RPC method");
  bool tb_mode = data;
  log_i("tb_mode = %d", tb_mode);
  if (tb_mode)
    mode = true; // manual
  else
    mode = false;                     // auto
  tb.sendAttributeBool("mode", mode); // send att for update btn status
  return RPC_Response("mode", mode);
}

RPC_Response processGetMode(const RPC_Data &data)
{
  Serial.println("Received the getMode RPC method");
  tb.sendAttributeBool("mode", mode); // send att for update btn status
  return RPC_Response("mode", mode);
}

RPC_Response processSetPump1(const RPC_Data &data)
{
  Serial.println("Received the SetPump1 RPC method");

  bool tb_pump1 = data;
  log_i("tb_pump1 = %d", tb_pump1);
  // manual
  if (mode)
  {
    if (tb_pump1)
    {
      digitalWrite(PUMP1, LOW); // on
      Serial.print("pump1 on\n");
      tb.sendAttributeBool("pump1", !digitalRead(PUMP1)); // send att for update btn status
    }
    else
    {
      digitalWrite(PUMP1, HIGH); // off
      Serial.print("pump1 off\n");
      tb.sendAttributeBool("pump1", !digitalRead(PUMP1)); // send att for update btn status
    }
  }
  else
  {
    tb.sendAttributeBool("pump1", !digitalRead(PUMP1)); // send att for update btn status
  }
  return RPC_Response("pump1", !digitalRead(PUMP1));
}

RPC_Response processGetPump1(const RPC_Data &data)
{
  Serial.println("Received the GetPump1 RPC method");
  tb.sendAttributeBool("pump1", !digitalRead(PUMP1)); // send att for update btn status
  return RPC_Response("pump1", !digitalRead(PUMP1));
}

RPC_Response processSetPump2(const RPC_Data &data)
{
  Serial.println("Received the SetPump2 RPC method");

  bool tb_pump2 = data;
  log_i("tb_pump2 = %d", tb_pump2);
  // manual
  if (mode)
  {
    if (tb_pump2)
    {
      digitalWrite(PUMP2, LOW); // on
      Serial.print("pump2 on\n");
      tb.sendAttributeBool("pump2", !digitalRead(PUMP2)); // send att for update btn status
    }
    else
    {
      digitalWrite(PUMP2, HIGH); // off
      Serial.print("pump2 off\n");
      tb.sendAttributeBool("pump2", !digitalRead(PUMP2)); // send att for update btn status
    }
  }
  else
  {
    tb.sendAttributeBool("pump2", !digitalRead(PUMP2)); // send att for update btn status
  }

  return RPC_Response("pump2", !digitalRead(PUMP2));
}

RPC_Response processGetPump2(const RPC_Data &data)
{
  Serial.println("Received the GetPump2 RPC method");
  tb.sendAttributeBool("pump2", !digitalRead(PUMP2)); // send att for update btn status
  return RPC_Response("pump2", !digitalRead(PUMP2));
}

RPC_Response processSetFood(const RPC_Data &data)
{
  Serial.println("Received the SetFood RPC method");

  bool tb_food = data;
  log_i("tb_food = %d", tb_food);
  // manual
  if (mode)
  {
    if (tb_food)
    {
      servo2.write(180);
      tb.sendAttributeBool("food", true); // send att for update btn status
      vTaskDelay(3000);
      servo2.write(0);
      tb.sendAttributeBool("food", false); // send att for update btn status
    }
    // else
    // {
    //   digitalWrite(PUMP2, HIGH); // off
    //   Serial.print("pump2 off\n");
    //   tb.sendAttributeBool("pump2", !digitalRead(PUMP2)); // send att for update btn status
    // }
  }
  else
  {
    tb.sendAttributeBool("food", false); // send att for update btn status
  }

  return RPC_Response("food", false);
}

RPC_Response processSetWaterLV(const RPC_Data &data)
{
  Serial.println("Received the setWL RPC method");

  return RPC_Response(NULL, NULL);
}

RPC_Response processSetVit(const RPC_Data &data)
{
  Serial.println("Received the SetMineral RPC method");

  bool tb_vit = data;
  log_i("tb_vit = %d", tb_vit);
  // manual
  if (mode)
  {
    if (tb_vit)
    {
      servo1.write(180);
      tb.sendAttributeBool("vit", true); // send att for update btn status
      vTaskDelay(3000);
      servo1.write(0);
      tb.sendAttributeBool("vit", false); // send att for update btn status
    }
    // else
    // {
    //   digitalWrite(SW_VIT, HIGH); // off
    //   Serial.print("SW_VIT off\n");
    //   tb.sendAttributeBool("vit", digitalRead(SW_VIT)); // send att for update btn status
    // }
  }
  else
  {
    tb.sendAttributeBool("vit", false); // send att for update btn status
  }

  return RPC_Response("vit", false);
}

RPC_Response processSetWaterTemp(const RPC_Data &data)
{
  Serial.println("Received the setWT RPC method");
  
  return RPC_Response(NULL, NULL);
}

// RPC handlers
RPC_Callback rpc_callbacks[] = {
    {"setMode", processSetMode},
    {"getMode", processGetMode},
    {"setPump1", processSetPump1},
    {"getPump1", processGetPump1},
    {"setPump2", processSetPump2},
    {"getPump2", processGetPump2},
    {"setFood", processSetFood},
    {"setVit", processSetVit},
    {"setWT", processSetWaterTemp}, // sp water temp
    {"setWL", processSetWaterLV}, // sp water lv
};

// void processSharedAttributeUpdate(const Shared_Attribute_Data &data)
// {
//   for (JsonObject::iterator it = data.begin(); it != data.end(); ++it)
//   {
//     Serial.println(it->key().c_str());
//     Serial.println(it->value().as<char *>()); // We have to parse data by it's type in the current example we will show here only char data
//   }

//   int jsonSize = measureJson(data) + 1;
//   char buffer[jsonSize];
//   serializeJson(data, buffer, jsonSize);
//   Serial.println(buffer);
// }

// Shared_Attribute_Callback shared_att_callbacks[1] = {
//     processSharedAttributeUpdate};

void updateAttributes()
{
  log_i("update thingsboard attbuites");
  tb.sendAttributeBool("mode", mode);                 // send att for update btn status
  tb.sendAttributeBool("pump1", !digitalRead(PUMP1)); // send att for update btn status
  tb.sendAttributeBool("pump2", !digitalRead(PUMP2)); // send att for update btn status
  tb.sendAttributeBool("food", false);                // send att for update btn status
  tb.sendAttributeBool("vit", false);                 // send att for update btn status
}

void setup()
{
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  InitRTC();

  InitWiFi();

  pinMode(PH_PIN, INPUT);
  pinMode(TDS_PIN, INPUT);

  pinMode(SW_MODE, INPUT);
  pinMode(SW_PUMP1, INPUT);
  pinMode(SW_PUMP2, INPUT);
  pinMode(SW_FOOD, INPUT);
  pinMode(SW_VIT, INPUT);

  pinMode(VCC_IN, OUTPUT);
  digitalWrite(VCC_IN, LOW);

  /*
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  */

  pinMode(PUMP1, OUTPUT);
  pinMode(PUMP2, OUTPUT);
  digitalWrite(PUMP1, HIGH);
  digitalWrite(PUMP2, HIGH);

  servo1.attach(SERVO1);
  servo2.attach(SERVO2);

  sensorWaterTemp.begin();
  /////////////////////////LED WIFI/////////////////////////////
  pinMode(LED_WIFI_STATUS, OUTPUT);
  //////////////////////////////////////////////////////////////

  servo1.write(0);
  servo2.write(0);

  xTaskCreate(TB_TASK, "TB_TASK", 4096, NULL, 9, NULL);
  xTaskCreate(PH_TASK, "PH_TASK", 4096, NULL, TSK_SENSORS_PRIORITY, NULL);
  xTaskCreate(TDS_TASK, "TDS_TASK", 4096, NULL, TSK_SENSORS_PRIORITY, NULL);
  xTaskCreate(WATER_TEMPERATURE_TASK, "WATER_TEMPERATURE_TASK", 4096, NULL, TSK_SENSORS_PRIORITY, NULL);
  xTaskCreate(WATER_LV_TASK, "WATER_LV_TASK", 4096, NULL, TSK_SENSORS_PRIORITY, NULL);
  xTaskCreate(CONTROL_TASK, "CONTROL_TASK", 4096, NULL, 9, NULL);
}

void loop()
{
}

void TB_TASK(void *pvParameters)
{
  for (;;)
  {
    static unsigned long tbms = millis();
    if (millis() - tbms > 30000U) //ส่งค่าขิึ้น tb 30s
    {
      tbms = millis();
      Serial.println("ส่งค่าขิึ้น tb 3s");
      tb.sendTelemetryFloat("PH", ph);
      tb.sendTelemetryFloat("TDS", tds);
      tb.sendTelemetryFloat("WT", wt);
      tb.sendTelemetryInt("WT LV", lev);

      updateAttributes();

      // Serial.printf("\ntb status = %d\n", tb.sendTelemetryInt("WT LV", lev));
    }

    // Reconnect to WiFi, if needed
    if (WiFi.status() != WL_CONNECTED)
    {
      reconnect();
      return;
    }

    // // Reconnect to ThingsBoard, if needed
    if (!tb.connected())
    {
      subscribed = false;

      // Connect to the ThingsBoard
      Serial.print("Connecting to: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" with token ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN))
      {
        Serial.println("Failed to connect");
        return;
      }
    }

    // Subscribe for RPC, if needed
    if (!subscribed)
    {
      // RPC
      Serial.println("Subscribing for RPC...");
      // Perform a subscription. All consequent data processing will happen in
      // callbacks as denoted by callbacks[] array.
      if (!tb.RPC_Subscribe(rpc_callbacks, COUNT_OF(rpc_callbacks))) // 1
      {
        Serial.println("Failed to subscribe for RPC");
        return;
      }
      Serial.println("Subscribe RPC done"); // end

      // // Shared_Attributes
      // Serial.println("Subscribing for shared attribute updates...");
      // if (!tb.Shared_Attributes_Subscribe(shared_att_callbacks, 1))
      // {
      //   Serial.println("Failed to subscribe for shared attribute updates");
      //   return;
      // }
      // Serial.println("Subscribe Shared_Attributes done"); // end

      subscribed = true;
    }

    // Process messages
    tb.loop();
    vTaskDelay(1);
  }

  vTaskDelete(NULL);
}

void PH_TASK(void *pvParameters)
{
  for (;;)
  {
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U)
    { // every 40 milliseconds,read the analog value from the ADC
      analogSampleTimepoint = millis();
      analogBufferPH[analogBufferIndexPH] = analogRead(PH_PIN); // read the analog value and store into the buffer
      analogBufferIndexPH++;
      if (analogBufferIndexPH == SCOUNT_PH)
      {
        analogBufferIndexPH = 0;
      }
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
      printTimepoint = millis();
      for (copyIndexPH = 0; copyIndexPH < SCOUNT_PH; copyIndexPH++)
      {
        analogBufferTempPH[copyIndexPH] = analogBufferPH[copyIndexPH];

        // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        avgVolyagePH = getMedianNum(analogBufferTempPH, SCOUNT_PH) * (float)VREF_PH / 4095.0;

        // convert voltage value to ph value
        ph = -4.10 * avgVolyagePH + calibration_value;

        // Serial.print("voltage:");
        // Serial.print(avgVolyagePH,2);
        // Serial.print("V   ");
        // Serial.printf("PH Value: %f\n", ph);
      }
    }
    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

void TDS_TASK(void *pvParameters)
{
  for (;;)
  {
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U)
    { // every 40 milliseconds,read the analog value from the ADC
      analogSampleTimepoint = millis();
      analogBufferTDS[analogBufferIndexTDS] = analogRead(TDS_PIN); // read the analog value and store into the buffer
      analogBufferIndexTDS++;
      if (analogBufferIndexTDS == SCOUNT_TDS)
      {
        analogBufferIndexTDS = 0;
      }
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
      printTimepoint = millis();
      for (copyIndexTDS = 0; copyIndexTDS < SCOUNT_TDS; copyIndexTDS++)
      {
        analogBufferTempTDS[copyIndexTDS] = analogBufferTDS[copyIndexTDS];

        // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        avgVoltageTDS = getMedianNum(analogBufferTempTDS, SCOUNT_TDS) * (float)VREF_TDS / 1980.0;
        temperatureTDS = wt; // use real water temp from sensor
        // temperatureTDS compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationCoefficient = 1.0 + 0.02 * (temperatureTDS - 25.0);
        // temperatureTDS compensation
        float compensationVoltage = avgVoltageTDS / compensationCoefficient;

        // convert voltage value to tds value
        tds = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

        // Serial.print("voltage:");
        // Serial.print(avgVoltageTDS,2);
        // Serial.print("V   ");
        // Serial.printf("TDS Value: %f ppm\n", tds);
      }
    }
    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

void WATER_TEMPERATURE_TASK(void *pvParameters)
{
  for (;;)
  {
    static unsigned long getWT = millis();
    if (millis() - getWT > 1000U)
    {
      getWT = millis();
      sensorWaterTemp.requestTemperatures();   //สั่งอ่านค่าอุณหภูมิ
      wt = sensorWaterTemp.getTempCByIndex(0); // แสดงค่าอุณหภูมิ
    }
    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

void WATER_LV_TASK(void *pvParameters)
{
  for (;;)
  {
    static unsigned long printwl = millis();
    if (millis() - printwl > 1000U)
    {
      printwl = millis();

      digitalWrite(VCC_IN, HIGH);
      delay(10);
      val = analogRead(SIGNAL_PIN);
      digitalWrite(VCC_IN, LOW);
      // lev = map(val, SENSOR_MIN, SENSOR_MAX, 0, 4);
      lev = map(val, SENSOR_MIN, SENSOR_MAX, 0, 100);
      lev > 100 ? lev = 100 : lev = lev;
      // Serial.print(val);
      // Serial.printf("WT LV : %d %\n", lev);
      // Serial.printf("WT LV ana: %d %\n\n", val);
    }
  }
  vTaskDelay(1);

  vTaskDelete(NULL);
}
/////////////////////////////////////////////////////////////////////////////////////////LCD
void LCD_TASK(void *pvParameters)
{
  for (;;)
  {
    static unsigned long printwl = millis();
    if (millis() - printwl > 1000U)
    {
      printwl = millis();
    }
  }
  vTaskDelay(1);

  vTaskDelete(NULL);
}

void CONTROL_TASK(void *pvParameters)
{
  for (;;)
  {
    static unsigned long printValues = millis();
    if (millis() - printValues > 1000) // 1s
    {
      printValues = millis();

      // Serial.printf("\nPH : %.2f\n", ph);
      // Serial.printf("TDS : %.2fppm\n", tds);
      // Serial.printf("WT : %.2f *C\n", wt);
      // Serial.printf("WT LV : %d %\n", lev);
      // Serial.printf("WT LV ana: %d\n\n", val);

      checkRTC();
      RtcDateTime now = Rtc.GetDateTime();
      printDateTime(now);
      // Serial.printf("hr: %d", now.Hour());

      // Serial.printf("\nSW_MODE: %d\n", digitalRead(SW_MODE));
      // Serial.printf("SW_PUMP1: %d\n", digitalRead(SW_PUMP1));
      // Serial.printf("SW_PUMP2: %d\n", digitalRead(SW_PUMP2));
      // Serial.printf("SW_FOOD: %d\n", digitalRead(SW_FOOD));
      // Serial.printf("SW_VIT: %d\n", digitalRead(SW_VIT));
      Serial.printf("\nMODE: %s\n", mode == false ? "AUTO" : "MANUAL");

      // auto
      if (!mode)
      {
        // 7.00
        if (now.Hour() == 7 && now.Minute() == 0 && now.Second() == 0)
        {
          Serial.printf("ให้อาหาร 7.00\n");
          servo2.write(180);
          vTaskDelay(3000);
          servo2.write(0);
        }
        // 19.00
        if (now.Hour() == 19 && now.Minute() == 0 && now.Second() == 0)
        {
          Serial.printf("ให้อาหาร 19.00\n");
          servo2.write(180);
          vTaskDelay(3000);
          servo2.write(0);
        }
        // TEST
        // if (now.Hour() == 15 && now.Minute() == 58 && now.Second() == 0)
        // {
        //   Serial.printf("ให้อาหาร 16.00\n");
        //   servo2.write(180);
        //   vTaskDelay(3000);
        //   servo2.write(0);
        // }
      }
    }

    // change mode
    if (digitalRead(SW_MODE) == 1)
    {
      while (digitalRead(SW_MODE) == 1)
        ;
      vTaskDelay(200);
      if (mode == false)
        mode = true;
      else
        mode = false;

      tb.sendAttributeBool("mode", mode); // send att for update btn status
    }                                     // end

    // mode manual
    if (mode)
    {
      ///////////////////////////////////////////////////////////////////////////////////////
      // pump 1
      if (digitalRead(SW_PUMP1) == 1)
      {
        Serial.printf("control pum1 manual\n");
        while (digitalRead(SW_PUMP1) == 1)
          ;
        vTaskDelay(200);
        digitalWrite(PUMP1, !digitalRead(PUMP1)); // toggle
        vTaskDelay(200);
        tb.sendAttributeBool("pump1", !digitalRead(PUMP1)); // send att for update btn status
      }                                                     // end pump 1
      // pump 2
      if (digitalRead(SW_PUMP2) == 1)
      {
        Serial.printf("control pum2 manual\n");
        while (digitalRead(SW_PUMP2) == 1)
          ;
        vTaskDelay(200);
        digitalWrite(PUMP2, !digitalRead(PUMP2));
        vTaskDelay(200);
        tb.sendAttributeBool("pump2", !digitalRead(PUMP2)); // send att for update btn status
      }                                                     // end pump 2
        //////////////////////////////////////////////////////////////////////////////////

      // servo food
      if (digitalRead(SW_FOOD) == 1)
      {
        Serial.printf("control servo food manual\n");
        while (digitalRead(SW_FOOD) == 1)
          ;
        vTaskDelay(200);
        servo2.write(180);
        tb.sendAttributeBool("food", true); // send att for update btn status
        vTaskDelay(3000);
        servo2.write(0);
        tb.sendAttributeBool("food", false); // send att for update btn status
      }                                      // end servo food

      // servo vit
      if (digitalRead(SW_VIT) == 1)
      {
        Serial.printf("control servo vit manual\n");
        while (digitalRead(SW_VIT) == 1)
          ;
        vTaskDelay(200);
        servo1.write(180);
        tb.sendAttributeBool("vit", true); // send att for update btn status
        vTaskDelay(3000);
        servo1.write(0);
        tb.sendAttributeBool("vit", false); // send att for update btn status
      }                                     // end servo vit

    } // end manual

    static unsigned long autoModeCheckInterval = millis();
    if (millis() - autoModeCheckInterval > 1000 * 30) // 30s
    {
      autoModeCheckInterval = millis();
      // mode auto
      if (!mode)
      {

        // water temp
        if (wt < 28.0)
        {
          Serial.printf("control heater on\n");
        }

        // vit
        if (tds < 80.0)
        {
          Serial.printf("control servo vit\n");
          servo1.write(180);
          vTaskDelay(3000);
          servo1.write(0);
        }

        // water lv
        if (lev < 80)
        {
          Serial.printf("control pump water lv on\n");
        }
        if (lev > 99)
        {
          Serial.printf("control pump water lv off\n");
        }
      }
    }

    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}