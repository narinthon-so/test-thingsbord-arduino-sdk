#ifndef _MAIN_H_
#define _MAIN_H_

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define TSK_SENSORS_PRIORITY 8
/**
 * @param ph
 * @param tds
 * @param wt
 * @param wl
 */

/////////////////////// PH ////////////////////////////////////////////

#define PH_PIN 35
#define VREF_PH 3.3  // analog reference voltage(Volt) of the ADC
#define SCOUNT_PH 30 // sum of sample point

int analogBufferPH[SCOUNT_PH]; // store the analog value in the array, read from ADC
int analogBufferTempPH[SCOUNT_PH];
int analogBufferIndexPH = 0;
int copyIndexPH = 0;

float avgVolyagePH = 0;
float ph = 0;
float calibration_value = 20.24 - 1.7; // 21.34 - 0.7

/////////////////////// END PH ////////////////////////////////////////////
/////////////////////// WT LV //////////////////////////////////////////
#define VCC_IN 12
#define SIGNAL_PIN 37
#define SENSOR_MIN 0
#define SENSOR_MAX 1200
/*
#define LED_4 15 // แดง
#define LED_3 4  // ส้ม
#define LED_2 17 // เหลือง
#define LED_1 16 // เขียว
*/

int val = 0;
int lev = 0;

// String water;
/////////////////////// END WT LV //////////////////////////////////////
/////////////////////// TDS ////////////////////////////////////////////

#define TDS_PIN 36
#define VREF_TDS 3.3  // analog reference voltage(Volt) of the ADC
#define SCOUNT_TDS 30 // sum of sample point

int analogBufferTDS[SCOUNT_TDS]; // store the analog value in the array, read from ADC
int analogBufferTempTDS[SCOUNT_TDS];
int analogBufferIndexTDS = 0;
int copyIndexTDS = 0;

float avgVoltageTDS = 0;
float tds = 0;
float temperatureTDS = 25; // current temperatureTDS for compensation

/////////////////////// END TDS ////////////////////////////////////////////

/////////////////////// WATER TEMPERATURE ////////////////////////////////////////////

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 13 //กำหนดว่าขาของเซนเซอร์ 18B20 ต่อกับขา 13
float wt = 0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorWaterTemp(&oneWire);

/////////////////////// END WT ////////////////////////////////////////////

/////////////////////// RTC ////////////////////////////////////////////
// CONNECTIONS:
// DS3231 SDA --> SDA
// DS3231 SCL --> SCL
// DS3231 VCC --> 3.3v or 5v
// DS3231 GND --> GND

/* for software wire use below
#include <SoftwareWire.h>  // must be included here so that Arduino library object file references work
#include <RtcDS3231.h>

SoftwareWire myWire(SDA, SCL);
RtcDS3231<SoftwareWire> Rtc(myWire);
 for software wire use above */

/* for normal hardware wire use below */
#include <Wire.h> // must be included here so that Arduino library object file references work
#include <RtcDS3231.h>
#include <EepromAT24C32.h>

RtcDS3231<TwoWire> Rtc(Wire);
EepromAt24c32<TwoWire> RtcEeprom(Wire);
#define countof(a) (sizeof(a) / sizeof(a[0]))

const char data[] = "What time is it in Greenwich?";
const uint16_t stringAddr = 64; // stored on page boundary
/////////////////////// END RTC ////////////////////////////////////////////

/////////////////////// THINGS BOARD ////////////////////////////////////////////

#include <WiFi.h>        // WiFi control for ESP32
#include <ThingsBoard.h> // ThingsBoard SDK
////////////////////////////LED WIFI/////////////////////////////////
#define LED_WIFI_STATUS 2
////////////////////////////////////////////////////////////////////
// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

// WiFi access point
#define WIFI_AP_NAME "YOUR_AP_NAME"
// WiFi password
#define WIFI_PASSWORD "YOUR_AP_PASSWORD"

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
#define TOKEN "6vMPUZxrf0XVXoGImk6u"
// ThingsBoard server instance.
#define THINGSBOARD_SERVER "192.168.199.233" // IP rpi

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD 115200

// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;
//////////////////////////////////// ctrl //////////////////////////////////////////////////
#include <Servo.h>
Servo servo1;
Servo servo2;

bool mode = false;

#define SW_MODE 32
#define SW_PUMP1 33
#define SW_PUMP2 25
#define SW_FOOD 26
#define SW_VIT 27

#define PUMP1 0  // 0
#define PUMP2 17 // 17
#define IN3 23   // 23
#define IN4 16   // 16

#define SERVO1 14 // 23
#define SERVO2 12 // 16

uint8_t sp_wt; //water temp setpoint
uint8_t sp_wl; //water lv setpoint

/////////////////////////////////////////// end ctrl //////////////////////////////////////

// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);

int status = WL_IDLE_STATUS;

/////////////////////// END THINGS BOARD ////////////////////////////////////////////

// func freeRTOS
void TB_TASK(void *pvParameters);                // tb
void PH_TASK(void *pvParameters);                // PH
void TDS_TASK(void *pvParameters);               // TDS
void WATER_TEMPERATURE_TASK(void *pvParameters); // Water Temperature
void WATER_LV_TASK(void *pvParameters);          // Water level
void CONTROL_TASK(void *pvParameters);           // Control relay servo etc.
void LCD_TASK(void *pvParameters);               // LCD
#endif