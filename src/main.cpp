#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "DHT.h"
#include "RTClib.h"
#include "secrets.h"

#define DHTTYPE DHT11

#define BUFFER_SIZE 10
const int buffer_size = BUFFER_SIZE;

typedef struct
{
  int value;
  char *timestamp;
} IntSensorRead;

typedef struct
{
  float value;
  char *timestamp;
} FloatSensorRead;

typedef struct
{
  IntSensorRead data[BUFFER_SIZE];
  int curIndex;
} IntSensorReadBuffer;

typedef struct
{
  FloatSensorRead data[BUFFER_SIZE];
  int curIndex;
} FloatSensorReadBuffer;

#define DHT_PIN 32
#define RAIN_PIN 34

DHT dht(DHT_PIN, DHTTYPE);
RTC_DS1307 rtc;

HTTPClient http;

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

template <typename T>
void sendData(T buffer, String sensorName);

void setup()
{
  Serial.begin(9600);
  WiFi.begin(SSID, PASSWORD);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }

  dht.begin();
  delayMicroseconds(1000); // Wait for sensor to boot

  randomSeed(analogRead(0));

  xQueueHandleTemperature = xQueueCreate(10, sizeof(FloatSensorRead));
  xQueueHandleHumidity = xQueueCreate(10, sizeof(FloatSensorRead));
  xQueueHandleRain = xQueueCreate(10, sizeof(IntSensorRead));

  xTaskCreatePinnedToCore(vTaskReadTemperature, "Read Temperature Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadTemperature, 0);
  xTaskCreatePinnedToCore(vTaskReadHumidity, "Read Humidity Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadHumidity, 0);
  xTaskCreatePinnedToCore(vTaskReadRain, "Read Rain Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadRain, 0);
  xTaskCreatePinnedToCore(vTaskSendData, "Send Data Task: ", configMINIMAL_STACK_SIZE + (1024 * 2), NULL, 1, &xTaskHandleSendData, 1);
}

void loop()
{
}

void vTaskReadTemperature(void *pvParams)
{
  while (1)
  {
    Serial.println("TASK 1: Reading temperature...");

    float value = dht.readTemperature();
    DateTime now = rtc.now();

    if (!isnan(value))
    {
      FloatSensorRead sensorRead;
      sensorRead.value = value;
      sensorRead.timestamp = (char *)malloc(strlen(now.timestamp().c_str()) + 1);
      strcpy(sensorRead.timestamp, now.timestamp().c_str());

      xQueueSend(xQueueHandleTemperature, &sensorRead, pdMS_TO_TICKS(1000));
    }
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
    float value = dht.readHumidity();
    DateTime now = rtc.now();

    if (!isnan(value))
    {
      FloatSensorRead sensorRead;
      sensorRead.value = value;
      sensorRead.timestamp = (char *)malloc(strlen(now.timestamp().c_str()) + 1);
      strcpy(sensorRead.timestamp, now.timestamp().c_str());

      xQueueSend(xQueueHandleHumidity, &sensorRead, pdMS_TO_TICKS(1000));
    }
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
    int value = analogRead(RAIN_PIN);
    DateTime now = rtc.now();

    IntSensorRead sensorRead;
    sensorRead.value = value;
    sensorRead.timestamp = (char *)malloc(strlen(now.timestamp().c_str()) + 1);
    strcpy(sensorRead.timestamp, now.timestamp().c_str());

    xQueueSend(xQueueHandleRain, &sensorRead, pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskSendData(void *pvParams)
{
  FloatSensorReadBuffer temperatureBuffer;
  temperatureBuffer.curIndex = 0;

  FloatSensorReadBuffer humidityBuffer;
  humidityBuffer.curIndex = 0;

  IntSensorReadBuffer rainBuffer;
  rainBuffer.curIndex = 0;

  while (1)
  {
    Serial.println("TASK 4");

    if (xQueueReceive(xQueueHandleTemperature, &temperatureBuffer.data[temperatureBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 4: Temperature Buffer " + String(temperatureBuffer.curIndex) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex].timestamp) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex++].value));
    else
      Serial.println("TASK 4: Temperature Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleHumidity, &humidityBuffer.data[humidityBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 4: Humidity Buffer " + String(humidityBuffer.curIndex) + ": " + String(humidityBuffer.data[humidityBuffer.curIndex].timestamp) + ": " + String(humidityBuffer.data[humidityBuffer.curIndex++].value));
    else
      Serial.println("TASK 4: Humidity Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleRain, &rainBuffer.data[rainBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 4: Rain Buffer " + String(rainBuffer.curIndex) + ": " + String(rainBuffer.data[rainBuffer.curIndex].timestamp) + ": " + String(rainBuffer.data[rainBuffer.curIndex++].value));
    else
      Serial.println("TASK 4: Rain Queue TIMEOUT");

    if (temperatureBuffer.curIndex == buffer_size)
    {
      Serial.println("TASK 4: Sending Temperature data...");
      sendData(temperatureBuffer, "temperature");
      temperatureBuffer.curIndex = 0;
    }

    if (humidityBuffer.curIndex == buffer_size)
    {
      Serial.println("TASK 4: Sending Humidity data...");
      sendData(humidityBuffer, "humidity");
      humidityBuffer.curIndex = 0;
    }

    if (rainBuffer.curIndex == buffer_size)
    {
      Serial.println("TASK 4: Sending Rain data...");
      sendData(rainBuffer, "rain");
      rainBuffer.curIndex = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

template <typename T>
void sendData(T buffer, String sensorName)
{
  JsonDocument json;
  json["sensor"] = sensorName;

  JsonArray data = json["data"].to<JsonArray>();

  for (int i = 0; i < buffer.curIndex; i++)
  {
    JsonObject dataObj = data.add<JsonObject>();
    dataObj["value"] = buffer.data[i].value;
    dataObj["timestamp"] = buffer.data[i].timestamp;
  }

  if (WiFi.status() != WL_CONNECTED)
    return;

  http.begin(DATA_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-access-token", SERVER_TOKEN);

  String jsonString;
  serializeJson(json, jsonString);
  Serial.println(jsonString);

  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0)
  {
    String response = http.getString();
    Serial.println("TASK 4: Response code: " + String(httpResponseCode));
    Serial.println("TASK 4: Response: " + response);
  }
  else
  {
    Serial.print("TASK 4: Error on sending POST: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}
