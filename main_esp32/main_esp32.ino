// BMP280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp;
int currentStateButton;
bool isBMP280Connected = false;

// DHT22
#include "DHT.h"
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
bool isDHT22Connected = false;

// AWS IoT
#include "cert.h" // inside AWS Private Key, AWS Certificate, Google Key Api and Wifi configuration
#define THING_NAME "ESP32_Weather_Station"
#define AWS_IOT_TOPIC "esp32/1/pub"

// Connect
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// Wifi
#include <WiFi.h>
#define WIFI_TIMEOUT 20000 // ms

// time & NTP
#include <NTPClient.h>
#include <ESP32Time.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
ESP32Time rtc(0);
bool isTimeUpdate = false;

// JSON
#include <ArduinoJson.h>

// OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool isSSD1306Connected = false;

// Geolocation
#include <WifiLocation.h>
bool isLocated = false;
bool isSendLocalization = false;
WifiLocation location (googleApiKey);

// FreeRTOS & Other
#include <Arduino.h>
#include <math.h>
#define DEVICE_ID 1
#define PERIOD_TIME_SEND_DATA 1 // minutes
#define PERIOD_TIME_UPDATE_TIME 60 // minutes
#define MAX_DELAY_COUNTER 1440
int redLed = 13;
int greenLed = 12;

SemaphoreHandle_t xSemaphore = NULL;

// Structure
int DelayCounter = 0;
struct {
  float temperature[MAX_DELAY_COUNTER];
  float pressure[MAX_DELAY_COUNTER];
  float humidity[MAX_DELAY_COUNTER];
  String timestamp[MAX_DELAY_COUNTER];
  bool isSend[MAX_DELAY_COUNTER];
} MainWeatherInfo;

bool isReadyToSave = false;
struct {
  float temperature;
  float pressure;
  float humidity;
  String timestamp;
} TmpWeatherInfo;

struct {
  float latitude;
  float longitude;
  float accuracy;
} MainGeoInfo;

void wifiConnect(void* parameters) {
  for (;;) {
    Serial.println(F("FreeRTOS: Checking Wifi connection..."));
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("FreeRTOS: Wifi Connected"));
      vTaskDelay(WIFI_TIMEOUT / portTICK_PERIOD_MS);
      if (WiFi.status() == WL_CONNECTED && isTimeUpdate == true && isLocated == false) {
        //geolocation();
      }
    }
    else {
      Serial.println(F("FreeRTOS: Connecting to Wifi..."));
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      unsigned long startTime = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT ) {}
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("FreeRTOS: Failed to connection Wifi"));
        vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
      }
      else {
        Serial.println(F("FreeRTOS: Wifi Connected"));
        Serial.println("Local ESP32 IP: ");
        Serial.println(WiFi.localIP());
        vTaskDelay(WIFI_TIMEOUT / portTICK_PERIOD_MS);
      }
    }
  }
}

void AWSConnect(void* parameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED && isTimeUpdate == true) {
      Serial.println(F("FreeRTOS: Connect to AWS"));
      if (client.connected() == true)
      {
        Serial.println(F("FreeRTOS: AWS IoT Connected"));
        if (isLocated == true && isSendLocalization == false) {
          xTaskCreate(sendGeoData, "Send Geo data to AWS", 3000, NULL, 1, NULL);
        }
        else {
          xTaskCreate(sendWeatherData, "Send Weather data to AWS", 3000, NULL, 1, NULL);
        }
        vTaskDelay(PERIOD_TIME_SEND_DATA * 60 * 1000 / portTICK_PERIOD_MS);
      }
      else {
        net.setCACert(AWS_ROOT_CA_CERT);
        net.setCertificate(AWS_DEVICE_CERT);
        net.setPrivateKey(AWS_PRIVATE_KEY);

        client.setServer(AWS_IOT_ENDPOINT, 8883);
        Serial.println(F("FreeRTOS: Connecting to AWS IoT Core..."));

        while (!client.connect(THING_NAME)) {}
        if (client.connected() == false)
        {
          Serial.println(F("FreeRTOS: Failed to connection AWS IoT"));
          vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
        }
        else {
          if (DelayCounter != 0) {

            Serial.println(F("FreeRTOS: AWS IoT Connected"));
            xTaskCreate(sendWeatherData, "Send data to AWS", 3000, NULL, 1, NULL);
            vTaskDelay(PERIOD_TIME_SEND_DATA * 60 * 1000 / portTICK_PERIOD_MS);
          }
          vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
        }
      }
    }
    else {
      vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
    }
  }
}

void sendGeoData(void* parameters) {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("FreeRTOS: Send Geo data to AWS"));
    StaticJsonDocument<200> doc;
    doc["lat"] = MainGeoInfo.latitude;
    doc["lng"] = MainGeoInfo.longitude;
    doc["accuracy"] = MainGeoInfo.accuracy;
    doc["device_id"] = DEVICE_ID;
    doc["timestamp"] = rtc.getTime("%Y-%m-%dT%H:%M:%SZ");
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    client.publish(AWS_IOT_TOPIC, jsonBuffer);
    Serial.println(F("FreeRTOS: Geo data sent to AWS"));
    isSendLocalization = true;
    vTaskDelete(NULL);
  }
}

void sendWeatherData(void* parameters)
{
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("FreeRTOS: Send weather data to AWS"));
      if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
      {
        Serial.println(DelayCounter);
        int i = 0;
        for (i; i < DelayCounter; i++) {
          StaticJsonDocument<200> doc;
          doc["temperature"] = MainWeatherInfo.temperature[i + 1];
          doc["pressure"] = MainWeatherInfo.pressure[i + 1];
          doc["humidity"] = MainWeatherInfo.humidity[i + 1];
          doc["device_id"] = DEVICE_ID;
          doc["timestamp"] = MainWeatherInfo.timestamp[i + 1];

          char jsonBuffer[512];
          serializeJson(doc, jsonBuffer);
          client.publish(AWS_IOT_TOPIC, jsonBuffer);
          Serial.println(F("FreeRTOS: Weather data sent to AWS"));
          MainWeatherInfo.isSend[DelayCounter] == true;
        }
        DelayCounter = DelayCounter - i;
        Serial.println(DelayCounter);
        xSemaphoreGive( xSemaphore );
      }
    }
    vTaskDelete(NULL);
  }
}


void updateTimeNTP(void* parameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("FreeRTOS: Update Time from NTP Serveer"));
      timeClient.begin();
      timeClient.setTimeOffset(0);
      while (!timeClient.update()) {
        timeClient.forceUpdate();
      }
      time_t epochTime = timeClient.getEpochTime();
      struct tm *ptm = gmtime ((time_t *)&epochTime);

      rtc.setTime(ptm->tm_sec, ptm->tm_min, ptm->tm_hour, ptm->tm_mday, ptm->tm_mon + 1, ptm->tm_year + 1900);
      Serial.println(F("FreeRTOS: RTC Clock in now updated"));
      isTimeUpdate = true;
      vTaskDelay(PERIOD_TIME_UPDATE_TIME * 60 * 1000 / portTICK_PERIOD_MS);
    }
    else {
      Serial.println(F("FreeRTOS: None Wi-fi connection. Update time is delay"));
      vTaskDelay(10 * 1000  / portTICK_PERIOD_MS);
    }
  }
}

void saveWeatherInfo(void*parameters) {
  for (;;) {
    if (isReadyToSave == true) {
      if ( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
      {
        Serial.println(F("FreeRTOS: Save weather data and time to structure"));
        Serial.println(DelayCounter);
        if (DelayCounter < MAX_DELAY_COUNTER - 1) {
          if (MainWeatherInfo.isSend[DelayCounter] == false) {
            DelayCounter++;
          }
        }
        else {
          DelayCounter = 0;
        }
        MainWeatherInfo.temperature[DelayCounter] = TmpWeatherInfo.temperature;
        MainWeatherInfo.humidity[DelayCounter] = TmpWeatherInfo.humidity;
        MainWeatherInfo.pressure[DelayCounter] = TmpWeatherInfo.pressure;
        MainWeatherInfo.timestamp[DelayCounter] = TmpWeatherInfo.timestamp;
        MainWeatherInfo.isSend[DelayCounter] = false;
        xSemaphoreGive( xSemaphore );
      }
      Serial.println(DelayCounter);
      isReadyToSave = false;
      vTaskDelay(PERIOD_TIME_SEND_DATA * 60 * 1000 / portTICK_PERIOD_MS);
    }
    else {
      vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
    }
  }
}

void readWeatherInfo(void* parameters) {
  for (;;) {
    float temp = dht.readTemperature();
    float hPa = bmp.readPressure() / 100;
    float pressu = roundf(hPa * 100) / 100;
    float humi = dht.readHumidity();
    if (isnan(humi) || isnan(hPa) || isnan(temp))
      vTaskDelay(5 * 1000 / portTICK_PERIOD_MS);

    TmpWeatherInfo.temperature = temp;
    TmpWeatherInfo.humidity = humi;
    TmpWeatherInfo.pressure = pressu;

    if (isTimeUpdate == true) {
      TmpWeatherInfo.timestamp = rtc.getTime("%Y-%m-%dT%H:%M:%SZ");
      isReadyToSave = true;
      vTaskDelay(20 * 1000 / portTICK_PERIOD_MS);
    }
    else {
      vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
    }
  }
}

void geolocation() {
  Serial.println (F("FreeRTOS: Checking localization"));
  location_t loc = location.getGeoFromWiFi();
  if (location.wlStatusStr (location.getStatus ()) == "OK" && loc.lat != 0 && loc.lon != 0) {
    MainGeoInfo.latitude = loc.lat;
    MainGeoInfo.longitude = loc.lon;
    MainGeoInfo.accuracy = loc.accuracy;
    isLocated = true;
  }
}

void startDisplay() {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextWrap(0);
  display.setCursor(49, 10);
  display.println("ESP32");
  display.setTextWrap(0);
  display.setCursor(10, 18);
  display.println("IoT Weather Staion");
  display.println("");
  display.setTextWrap(0);
  display.setCursor(34, 34);
  display.println("created by");
  display.setTextWrap(0);
  display.setCursor(1, 42);
  display.println("Dominik Dziegielewski");
  display.display();
}

void updateDisplay(void* parameters) {
  for (;;) {
    display.clearDisplay();

    display.setCursor(0, 0);
    display.println(rtc.getTime("%H:%M"));
    
    currentStateButton = digitalRead(0);
    
    //if (currentStateButton == LOW)
    
    if (WiFi.status() == WL_CONNECTED) {
      display.setCursor(90, 0);
      display.print("Online");
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, HIGH);
    }
    else {
      display.setCursor(86, 0);
      display.println("Offline");
      digitalWrite(redLed, HIGH);
      digitalWrite(greenLed, LOW);
    }
    display.drawLine(0, 10, 128, 10, 1);

    display.setCursor(0, 15);
    display.println("");
    display.print("Temperature: ");
    display.print(TmpWeatherInfo.temperature);
    display.println(" C");
    display.println("");
    display.print("Humidity: ");
    display.print(TmpWeatherInfo.humidity);
    display.println(" %");
    display.println("");
    display.print("Pressure: ");
    display.print(TmpWeatherInfo.pressure);
    display.println(" hPa");

    display.display();
    vTaskDelay(5 * 1000 / portTICK_PERIOD_MS);
  }
}

bool checkAllSensor() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, HIGH);
  pinMode(0, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) {
    Serial.println(F("Could not find a SSD1306 display"));
  }
  else {
    isSSD1306Connected = true;
    display.setTextSize(1);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    startDisplay();
  }

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a BMP280 sensor"));
  }
  else {
    isBMP280Connected = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
  dht.begin();
  delay(4000);
  if (isnan(dht.readHumidity()) || isnan(dht.readTemperature()) || isnan(dht.readTemperature(true))) {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  else {
    isDHT22Connected = true;
  }

  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  if (isBMP280Connected == true && isDHT22Connected == true && isSSD1306Connected == true) {
    Serial.println(F("All sensors are connected"));

    return true;
  }
  else {
    Serial.println(F("Not find a sensor! Check log"));
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  if (checkAllSensor()) {
    xSemaphore = xSemaphoreCreateMutex();
    xTaskCreate(wifiConnect, "Connect to Wifi", 5000, NULL, 6, NULL);
    xTaskCreate(updateTimeNTP, "Update Time from NTP Serveer", 3000, NULL, 3, NULL);
    xTaskCreate(readWeatherInfo, "Read weather sensor and time", 3000, NULL, 2, NULL);
    xTaskCreate(saveWeatherInfo, "Save weather data and time to structure", 3000, NULL, 5, NULL);
    xTaskCreate(AWSConnect, "Connect to AWS", 10000, NULL, 4, NULL);
    xTaskCreate(updateDisplay, "OLED Display", 3000, NULL, 3, NULL);
  }
}

void loop() {
}