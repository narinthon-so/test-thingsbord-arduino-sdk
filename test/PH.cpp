// Original source code: https://wiki.keyestudio.com/KS0429_keyestudio_TDS_Meter_V1.0#Test_Code
// Project details: https://RandomNerdTutorials.com/esp32-tds-water-quality-sensor/
#include <Arduino.h>
#define PH_PIN 36
#define VREF_PH 3.3  // analog reference voltage(Volt) of the ADC
#define SCOUNT_PH 30 // sum of sample point

int analogBufferPH[SCOUNT_PH]; // store the analog value in the array, read from ADC
int analogBufferTempPH[SCOUNT_PH];
int analogBufferIndexPH = 0;
int copyIndexPH = 0;

float avgVolyagePH = 0;
float ph = 0;
float calibration_value = 20.24 - 1.7; // 21.34 - 0.7

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

void setup()
{
  Serial.begin(115200);
  pinMode(PH_PIN, INPUT);
}

void loop()
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
      avgVolyagePH = getMedianNum(analogBufferTempPH, SCOUNT_PH) * (float)VREF_PH / 4096.0;

      // convert voltage value to ph value
      ph = -4.10 * avgVolyagePH + calibration_value;

      // Serial.print("voltage:");
      // Serial.print(avgVolyagePH,2);
      // Serial.print("V   ");
      Serial.print("PH Value:");
      Serial.println(ph, 2);
      Serial.printf("ana = %d\n", analogRead(PH_PIN));
    }
  }
}