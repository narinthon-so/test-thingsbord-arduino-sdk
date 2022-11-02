// Original source code: https://wiki.keyestudio.csom/KS0429_keyestudio_TDS_Meter_V1.0#Test_Code
// Project details: https://RandomNerdTutorials.com/esp32-tds-water-quality-sensor/
#include <Arduino.h>
#define TDS_PIN 36
#define VREF_TDS 3.3  // analog reference voltage(Volt) of the ADC
#define SCOUNT_TDS 30 // sum of sample point

////////////////////////MAP////////////////////////
#define SENSOR_MIN 0
#define SENSOR_MAX 4096
////////////////////////MAP////////////////////////

int analogBufferTDS[SCOUNT_TDS]; // store the analog value in the array, read from ADC
int analogBufferTempTDS[SCOUNT_TDS];
int analogBufferIndexTDS = 0;
int copyIndexTDS = 0;

float avgVoltageTDS = 0;
float tds = 0;
float temperatureTDS = 27; // current temperatureTDS for compensation

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
    // pinMode(TDS_PIN, INPUT);
}

void loop()
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
            avgVoltageTDS = getMedianNum(analogBufferTempTDS, SCOUNT_TDS) * (float)VREF_TDS / 4096.0;

            // temperatureTDS compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
            float compensationCoefficient = 1.0 + 0.02 * (temperatureTDS - 27.0);
            // temperatureTDS compensation
            float compensationVoltage = avgVoltageTDS / compensationCoefficient;

            // convert voltage value to tds value
            tds = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

            // Serial.print("voltage:");
            // Serial.print(avgVoltageTDS,2);
            // Serial.print("V   ");

            ////////////////////////MAP////////////////////////
            tds = map(tds, SENSOR_MIN, SENSOR_MAX, 0, 14);
            tds > 50 ? tds = 4 : tds = tds;
            tds > 80 ? tds = 5 : tds = tds;
            tds > 100 ? tds = 6 : tds = tds;
            tds > 200 ? tds = 7 : tds = tds;
            tds > 300 ? tds = 8 : tds = tds;
            tds > 400 ? tds = 9 : tds = tds;
            tds > 500 ? tds = 10 : tds = tds;
            ////////////////////////MAP////////////////////////

            Serial.print("TDS Value:");
            Serial.print(tds, 0);
            Serial.println("ppm");
            Serial.printf("ana = %d\n", analogRead(TDS_PIN));
        }
    }
}