#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "DHT.h"

typedef struct
{
  int data[10];
  int curIndex;
} Buffer;

#define DHT_PIN 5
#define RAIN_PIN 34

DHT sensorDHT;

TaskHandle_t xTaskHandleReadTemperature = NULL;
TaskHandle_t xTaskHandleReadHumidity = NULL;
TaskHandle_t xTaskHandleReadRain = NULL;
TaskHandle_t xTaskHandleSendData = NULL;

QueueHandle_t xQueueHandleTemperature = NULL;
QueueHandle_t xQueueHandleHumidity = NULL;
QueueHandle_t xQueueHandleRain = NULL;

void vTaskReadTemperature(void *pvParams);
void vTaskReadHumidity(void *pvParams);
void vTaskReadRain(void *pvParams);
void vTaskSendData(void *pvParams);

void setup()
{
  sensorDHT.setup(DHT_PIN, DHT::DHT11);
  delayMicroseconds(1000); // Wait for sensor to boot

  xQueueHandleTemperature = xQueueCreate(10, sizeof(int));
  xQueueHandleHumidity = xQueueCreate(10, sizeof(int));
  xQueueHandleRain = xQueueCreate(10, sizeof(int));

  xTaskCreatePinnedToCore(vTaskReadTemperature, "Read Temperature Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadTemperature, 0);
  xTaskCreatePinnedToCore(vTaskReadHumidity, "Read Humidity Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadHumidity, 0);
  xTaskCreatePinnedToCore(vTaskReadRain, "Read Rain Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadRain, 0);
  xTaskCreatePinnedToCore(vTaskSendData, "Send Data Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleSendData, 1);
}

void loop()
{
}

void vTaskReadTemperature(void *pvParams)
{
  while (1)
  {
    int sensorRead = sensorDHT.getTemperature();
    if (sensorDHT.getStatus() == DHT::ERROR_NONE)
    {
      xQueueSend(xQueueHandleTemperature, &sensorRead, pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskReadHumidity(void *pvParams)
{

  while (1)
  {
    int sensorRead = sensorDHT.getHumidity();
    if (sensorDHT.getStatus() == DHT::ERROR_NONE)
    {
      xQueueSend(xQueueHandleHumidity, &sensorRead, pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskReadRain(void *pvParams)
{
  pinMode(RAIN_PIN, INPUT);

  while (1)
  {
    int sensorRead = analogRead(RAIN_PIN);
    xQueueSend(xQueueHandleRain, &sensorRead, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskSendData(void *pvParams)
{
  Serial.begin(9600);

  Buffer temperatureBuffer;
  temperatureBuffer.curIndex = 0;

  Buffer humidityBuffer;
  humidityBuffer.curIndex = 0;

  Buffer rainBuffer;
  rainBuffer.curIndex = 0;

  while (1)
  {
    if (xQueueReceive(xQueueHandleTemperature, &temperatureBuffer.data[temperatureBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("Temperature Buffer " + String(temperatureBuffer.curIndex) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex++]));
    else
      Serial.println("Temperature Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleHumidity, &humidityBuffer.data[humidityBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("Humidity Buffer " + String(humidityBuffer.curIndex) + ": " + String(humidityBuffer.data[humidityBuffer.curIndex++]));
    else
      Serial.println("Humidity Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleRain, &rainBuffer.data[rainBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("Rain Buffer " + String(rainBuffer.curIndex) + ": " + String(rainBuffer.data[rainBuffer.curIndex++]));
    else
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
}