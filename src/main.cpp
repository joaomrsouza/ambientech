#include <Arduino.h>
#include <SPI.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "DHT.h"
#include "RTClib.h"
#include "secrets.h"

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
#define MQ135_PIN 33
#define MQ2_PIN 35

#define DHTTYPE DHT11

DHT dht(DHT_PIN, DHTTYPE);
RTC_DS1307 rtc;

HTTPClient http;
WiFiClientSecure *client = new WiFiClientSecure;

TaskHandle_t xTaskHandleReadTemperature = NULL;
TaskHandle_t xTaskHandleReadHumidity = NULL;
TaskHandle_t xTaskHandleReadRain = NULL;
TaskHandle_t xTaskHandleReadQoA = NULL;
TaskHandle_t xTaskHandleReadSmoke = NULL;
TaskHandle_t xTaskHandleSendData = NULL;

QueueHandle_t xQueueHandleTemperature = NULL;
QueueHandle_t xQueueHandleHumidity = NULL;
QueueHandle_t xQueueHandleRain = NULL;
QueueHandle_t xQueueHandleQoA = NULL;
QueueHandle_t xQueueHandleSmoke = NULL;

void vTaskReadTemperature(void *pvParams);
void vTaskReadHumidity(void *pvParams);
void vTaskReadRain(void *pvParams);
void vTaskReadQoA(void *pvParams);
void vTaskReadSmoke(void *pvParams);
void vTaskSendData(void *pvParams);

template <typename T>
void sendData(T buffer, String sensorName);

void setup()
{
  Serial.begin(9600);

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

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

  client->setCACert(ROOT_CERT);

  dht.begin();
  delayMicroseconds(1000); // Wait for sensor to boot

  randomSeed(analogRead(0));

  xQueueHandleTemperature = xQueueCreate(10, sizeof(FloatSensorRead));
  xQueueHandleHumidity = xQueueCreate(10, sizeof(FloatSensorRead));
  xQueueHandleRain = xQueueCreate(10, sizeof(IntSensorRead));
  xQueueHandleQoA = xQueueCreate(10, sizeof(FloatSensorRead));
  xQueueHandleSmoke = xQueueCreate(10, sizeof(IntSensorRead));

  xTaskCreatePinnedToCore(vTaskReadTemperature, "Read Temperature Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadTemperature, 0);
  xTaskCreatePinnedToCore(vTaskReadHumidity, "Read Humidity Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadHumidity, 0);
  xTaskCreatePinnedToCore(vTaskReadRain, "Read Rain Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadRain, 0);
  xTaskCreatePinnedToCore(vTaskReadQoA, "Read QoA Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadQoA, 0);
  xTaskCreatePinnedToCore(vTaskReadSmoke, "Read Smoke Task: ", configMINIMAL_STACK_SIZE + 1024, NULL, 1, &xTaskHandleReadSmoke, 0);
  xTaskCreatePinnedToCore(vTaskSendData, "Send Data Task: ", configMINIMAL_STACK_SIZE + (1024 * 6), NULL, 1, &xTaskHandleSendData, 1);
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

    if (!isnan(value) && value != 0)
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

    if (!isnan(value) && value != 0)
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

    if (value != 0)
    {
      IntSensorRead sensorRead;
      sensorRead.value = map(value, 0, 4095, 100, 0); // Remap rain data
      sensorRead.timestamp = (char *)malloc(strlen(now.timestamp().c_str()) + 1);
      strcpy(sensorRead.timestamp, now.timestamp().c_str());

      xQueueSend(xQueueHandleRain, &sensorRead, pdMS_TO_TICKS(1000));
    }
    else
      Serial.println("TASK 3: Error reading rain");

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskReadQoA(void *pvParams)
{
  vTaskDelay(pdMS_TO_TICKS(1000 * 60 * 5)); // Wait for warm-up

  while (1)
  {
    Serial.println("TASK 4: Reading QoA...");
    float value = analogRead(MQ135_PIN);
    DateTime now = rtc.now();

    if (!isnan(value) && value != 0)
    {
      FloatSensorRead sensorRead;
      sensorRead.value = map(value, 0, 4095, 100, 0); // Remap qoa data
      sensorRead.timestamp = (char *)malloc(strlen(now.timestamp().c_str()) + 1);
      strcpy(sensorRead.timestamp, now.timestamp().c_str());

      xQueueSend(xQueueHandleQoA, &sensorRead, pdMS_TO_TICKS(1000));
    }
    else
      Serial.println("TASK 4: Error reading QoA");

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vTaskReadSmoke(void *pvParams)
{
  pinMode(MQ2_PIN, INPUT);

  vTaskDelay(pdMS_TO_TICKS(1000 * 60 * 5)); // Wait for warm-up

  while (1)
  {
    Serial.println("TASK 5: Reading smoke...");
    int value = analogRead(MQ2_PIN);
    DateTime now = rtc.now();

    if (value != 0)
    {
      IntSensorRead sensorRead;
      sensorRead.value = map(value, 0, 4095, 0, 100); // Remap smoke data;
      sensorRead.timestamp = (char *)malloc(strlen(now.timestamp().c_str()) + 1);
      strcpy(sensorRead.timestamp, now.timestamp().c_str());

      xQueueSend(xQueueHandleSmoke, &sensorRead, pdMS_TO_TICKS(1000));
    }
    else
      Serial.println("TASK 5: Error reading Smoke");

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

  FloatSensorReadBuffer qoaBuffer;
  qoaBuffer.curIndex = 0;

  IntSensorReadBuffer smokeBuffer;
  smokeBuffer.curIndex = 0;

  while (1)
  {
    Serial.println("TASK 6");

    if (xQueueReceive(xQueueHandleTemperature, &temperatureBuffer.data[temperatureBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 6: Temperature Buffer " + String(temperatureBuffer.curIndex) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex].timestamp) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex++].value));
    else
      Serial.println("TASK 6: Temperature Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleHumidity, &humidityBuffer.data[humidityBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 6: Humidity Buffer " + String(humidityBuffer.curIndex) + ": " + String(humidityBuffer.data[humidityBuffer.curIndex].timestamp) + ": " + String(humidityBuffer.data[humidityBuffer.curIndex++].value));
    else
      Serial.println("TASK 6: Humidity Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleRain, &rainBuffer.data[rainBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 6: Rain Buffer " + String(rainBuffer.curIndex) + ": " + String(rainBuffer.data[rainBuffer.curIndex].timestamp) + ": " + String(rainBuffer.data[rainBuffer.curIndex++].value));
    else
      Serial.println("TASK 6: Rain Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleQoA, &qoaBuffer.data[qoaBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 6: QoA Buffer " + String(qoaBuffer.curIndex) + ": " + String(qoaBuffer.data[qoaBuffer.curIndex].timestamp) + ": " + String(qoaBuffer.data[qoaBuffer.curIndex++].value));
    else
      Serial.println("TASK 6: QoA Queue TIMEOUT");

    if (xQueueReceive(xQueueHandleSmoke, &smokeBuffer.data[smokeBuffer.curIndex], pdMS_TO_TICKS(1000)) == pdTRUE)
      Serial.println("TASK 6: Smoke Buffer " + String(smokeBuffer.curIndex) + ": " + String(smokeBuffer.data[smokeBuffer.curIndex].timestamp) + ": " + String(smokeBuffer.data[smokeBuffer.curIndex++].value));
    else
      Serial.println("TASK 6: Smoke Queue TIMEOUT");

    if (temperatureBuffer.curIndex == buffer_size)
    {
      Serial.println("TASK 6: Sending Temperature data...");
      sendData(temperatureBuffer, "temperature");
      temperatureBuffer.curIndex = 0;
    }

    if (humidityBuffer.curIndex == buffer_size)
    {
      Serial.println("TASK 6: Sending Humidity data...");
      sendData(humidityBuffer, "humidity");
      humidityBuffer.curIndex = 0;
    }

    if (rainBuffer.curIndex == buffer_size)
    {
      Serial.println("TASK 6: Sending Rain data...");
      sendData(rainBuffer, "rain");
      rainBuffer.curIndex = 0;
    }

    if (qoaBuffer.curIndex == buffer_size)
    {
      Serial.println("TASK 6: Sending QoA data...");
      sendData(qoaBuffer, "qoa");
      qoaBuffer.curIndex = 0;
    }

    if (smokeBuffer.curIndex == buffer_size)
    {
      Serial.println("TASK 6: Sending Smoke data...");
      sendData(smokeBuffer, "smoke");
      smokeBuffer.curIndex = 0;
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

  http.begin(*client, DATA_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-access-token", SERVER_TOKEN);

  String jsonString;
  serializeJson(json, jsonString);
  Serial.println(jsonString);

  int httpResponseCode = http.POST(jsonString);

  if (httpResponseCode > 0)
  {
    String response = http.getString();
    Serial.println("TASK 6: Response code: " + String(httpResponseCode));
    Serial.println("TASK 6: Response: " + response);
  }
  else
  {
    Serial.print("TASK 6: Error on sending POST: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}
