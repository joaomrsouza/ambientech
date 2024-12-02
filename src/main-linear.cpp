#include <Arduino.h>

#include "DHT.h"

typedef struct
{
  int data[10];
  int curIndex;
} Buffer;

#define DHT_PIN 5
#define RAIN_PIN 34

DHT sensorDHT;
Buffer temperatureBuffer;
Buffer humidityBuffer;
Buffer rainBuffer;

void setup()
{
  sensorDHT.setup(DHT_PIN, DHT::DHT11);
  delayMicroseconds(1000); // Wait for sensor to boot

  Serial.begin(9600);

  temperatureBuffer.curIndex = 0;
  humidityBuffer.curIndex = 0;
  rainBuffer.curIndex = 0;
}

void loop()
{
  float temperatureRead = sensorDHT.getTemperature();
  if (sensorDHT.getStatus() == DHT::ERROR_NONE)
  {
    temperatureBuffer.data[temperatureBuffer.curIndex] = temperatureRead;
    Serial.println("Temperature Buffer " + String(temperatureBuffer.curIndex) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex++]));
    Serial.println("Temperature Buffer " + String(temperatureBuffer.curIndex) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex++]));
  }

  Serial.println("Humidity Buffer " + String(humidityBuffer.curIndex) + ": " + String(humidityBuffer.data[humidityBuffer.curIndex++]));
  Serial.println("Humidity Queue TIMEOUT");

  Serial.println("Rain Buffer " + String(rainBuffer.curIndex) + ": " + String(rainBuffer.data[rainBuffer.curIndex++]));
  Serial.println("Rain Queue TIMEOUT");

  if (temperatureBuffer.curIndex == 10)
  {
    Serial.println("Sending Temperature data...");
    temperatureBuffer.curIndex = 0;
  }

  if (humidityBuffer.curIndex == 10)
  {
    Serial.println("Sending Humidity data...");
    humidityBuffer.curIndex = 0;
  }

  if (rainBuffer.curIndex == 10)
  {
    Serial.println("Sending Rain data...");
    rainBuffer.curIndex = 0;
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
}