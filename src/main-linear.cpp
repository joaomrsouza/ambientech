// #include <Arduino.h>

// #include "DHT.h"

// #define DHTTYPE DHT11

// typedef struct
// {
//   float data[10];
//   int curIndex;
// } FloatSensorReadBuffer;

// typedef struct
// {
//   int data[10];
//   int curIndex;
// } IntSensorReadBuffer;

// #define DHT_PIN 32
// #define RAIN_PIN 34

// DHT dht(DHT_PIN, DHTTYPE);

// FloatSensorReadBuffer temperatureBuffer;
// FloatSensorReadBuffer humidityBuffer;
// IntSensorReadBuffer rainBuffer;

// void simulateRequest();

// void setup()
// {
//   pinMode(RAIN_PIN, INPUT);

//   dht.begin();
//   delayMicroseconds(1000); // Wait for sensor to boot

//   Serial.begin(9600);

//   temperatureBuffer.curIndex = 0;
//   humidityBuffer.curIndex = 0;
//   rainBuffer.curIndex = 0;

//   randomSeed(analogRead(0));
// }

// void loop()
// {
//   Serial.println("==========  Loop  ==========");

//   Serial.println("MILIS: " + String(millis()));
//   float temperatureRead = dht.readTemperature();
//   if (!isnan(temperatureRead))
//   {
//     temperatureBuffer.data[temperatureBuffer.curIndex] = temperatureRead;
//     Serial.println("Temperature Buffer " + String(temperatureBuffer.curIndex) + ": " + String(temperatureBuffer.data[temperatureBuffer.curIndex++]));
//   }
//   else
//     Serial.println("Error reading temperature");

//   float humidityRead = dht.readHumidity();
//   if (!isnan(humidityRead))
//   {
//     humidityBuffer.data[humidityBuffer.curIndex] = humidityRead;
//     Serial.println("Humidity Buffer " + String(humidityBuffer.curIndex) + ": " + String(humidityBuffer.data[humidityBuffer.curIndex++]));
//   }
//   else
//     Serial.println("Error reading humidity");

//   int rainRead = analogRead(RAIN_PIN);
//   rainBuffer.data[rainBuffer.curIndex] = rainRead;
//   Serial.println("Rain Buffer " + String(rainBuffer.curIndex) + ": " + String(rainBuffer.data[rainBuffer.curIndex++]));

//   if (temperatureBuffer.curIndex == 10)
//   {
//     Serial.println("Sending Temperature data...");
//     simulateRequest();
//     temperatureBuffer.curIndex = 0;
//   }

//   if (humidityBuffer.curIndex == 10)
//   {
//     Serial.println("Sending Humidity data...");
//     simulateRequest();
//     humidityBuffer.curIndex = 0;
//   }

//   if (rainBuffer.curIndex == 10)
//   {
//     Serial.println("Sending Rain data...");
//     simulateRequest();
//     rainBuffer.curIndex = 0;
//   }

//   delay(5000);
// }

// void simulateRequest()
// {
//   // Generate a random delay between 1000ms (1 second) and 3000ms (3 seconds)
//   int delayTime = random(1000, 3001); // random(min, max) includes min, excludes max
//   Serial.print("Simulating request, waiting for ");
//   Serial.print(delayTime);
//   Serial.println(" milliseconds...");

//   delay(delayTime); // Pause for the random duration
//   Serial.println("Request simulation complete.");
// }