#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "DHT.h"

#define DHTTYPE DHT11

typedef struct
{
  float data[10];
  int curIndex;
} FloatBuffer;

typedef struct
{
  int data[10];
  int curIndex;
} IntBuffer;

#define DHT_PIN 32
#define RAIN_PIN 34

DHT dht(DHT_PIN, DHTTYPE);

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

void simulateRequest();

void setup()
{
  Serial.begin(9600);

  dht.begin();
  delayMicroseconds(1000); // Wait for sensor to boot

  randomSeed(analogRead(0));

  xQueueHandleTemperature = xQueueCreate(10, sizeof(float));
  xQueueHandleHumidity = xQueueCreate(10, sizeof(float));
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
    Serial.println("TASK 1: Reading temperature...");
    float sensorRead = dht.readTemperature();
    if (!isnan(sensorRead))
      xQueueSend(xQueueHandleTemperature, &sensorRead, pdMS_TO_TICKS(1000));
    else
      Serial.println("TASK 1: Error reading temperature");

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskReadHumidity(void *pvParams)
{
  while (1)
  {
    Serial.println("TASK 2: Reading humidity...");
    float sensorRead = dht.readHumidity();
    if (!isnan(sensorRead))
      xQueueSend(xQueueHandleHumidity, &sensorRead, pdMS_TO_TICKS(1000));
    else
      Serial.println("TASK 2: Error reading humidity");

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskReadRain(void *pvParams)
{
  pinMode(RAIN_PIN, INPUT);

  while (1)
  {
    Serial.println("TASK 3: Reading rain...");
    int sensorRead = analogRead(RAIN_PIN);
    xQueueSend(xQueueHandleRain, &sensorRead, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskSendData(void *pvParams)
{
  FloatBuffer temperatureBuffer;
  temperatureBuffer.curIndex = 0;

  FloatBuffer humidityBuffer;
  humidityBuffer.curIndex = 0;

  IntBuffer rainBuffer;
  rainBuffer.curIndex = 0;

  while (1)
  {
    Serial.println("TASK 4");

    if (xQueueReceive(xQueueHandleTemperature, &temperatureBuffer.data[temperatureBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 4: Temperature Buffer " + String(temperatureBuffer.curIndex) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex++]));
    else
      Serial.println("TASK 4: Temperature Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleHumidity, &humidityBuffer.data[humidityBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 4: Humidity Buffer " + String(humidityBuffer.curIndex) + ": " + String(humidityBuffer.data[humidityBuffer.curIndex++]));
    else
      Serial.println("TASK 4: Humidity Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleRain, &rainBuffer.data[rainBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 4: Rain Buffer " + String(rainBuffer.curIndex) + ": " + String(rainBuffer.data[rainBuffer.curIndex++]));
    else
      Serial.println("TASK 4: Rain Queue TIMEOUT");

    if (temperatureBuffer.curIndex == 10)
    {
      Serial.println("TASK 4: Sending Temperature data...");
      simulateRequest();
      temperatureBuffer.curIndex = 0;
    }

    if (humidityBuffer.curIndex == 10)
    {
      Serial.println("TASK 4: Sending Humidity data...");
      simulateRequest();
      humidityBuffer.curIndex = 0;
    }

    if (rainBuffer.curIndex == 10)
    {
      Serial.println("TASK 4: Sending Rain data...");
      simulateRequest();
      rainBuffer.curIndex = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void simulateRequest()
{
  // Generate a random delay between 1000ms (1 second) and 3000ms (3 seconds)
  int delayTime = random(1000, 3001); // random(min, max) includes min, excludes max
  Serial.print("TASK 4: Simulating request, waiting for ");
  Serial.print(delayTime);
  Serial.println(" milliseconds...");

  delay(delayTime); // Pause for the random duration
  Serial.println("TASK 4: Request simulation complete.");
}