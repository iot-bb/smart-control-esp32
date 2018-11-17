/*
    Smart Control
    Author: Supachai Chaimangua (Tor)
    UpdatedDate: 2018-11-11T23:00:00+07:00
    CreatedDate: 2018-11-11T23:00:00+07:00

    Comment:

    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/

#include <Arduino.h>
#include <ArduinoJson.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <Preferences.h>
#include "FS.h"
#include <Adafruit_Sensor.h>
#include "DHT.h"

#define BLE_PIN 2
#define SWITCH_PIN 5
#define TEMPERATURE_PIN 18
#define SWITCH_STATUS_PIN 23
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DHTTYPE DHT11
#define DATA_SIZE 5

String INPUT_JSON = "";
std::string END_JSON = "}";

Preferences preferences;

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

bool bleConnected = false;
bool oldBleConnected = false;

uint8_t temperature = 0;
uint8_t status = 0;
uint8_t set_enable = 0;
uint8_t set_min = 0;
uint8_t set_max = 0;
uint8_t data[DATA_SIZE];
uint8_t check_enable = 1;
int delay_count = 0;
int max = 500;

DHT dht(TEMPERATURE_PIN, DHTTYPE);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("BLE is connnected");
      bleConnected = true;
      digitalWrite(BLE_PIN, HIGH);
    };

    void onDisconnect(BLEServer* pServer) {
       digitalWrite(BLE_PIN, LOW);
       bleConnected = false;
      Serial.println("BLE is disconnnected");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic){
    Serial.println("====== onRead");
  }
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    pCharacteristic->setValue("OK");
    // Serial.println("====== value: " + String(value.c_str()));
    DynamicJsonBuffer jsonBuffer;
    INPUT_JSON = INPUT_JSON + String(value.c_str());
    // Serial.println("====== json: " + INPUT_JSON);
    if (value.find(END_JSON) != std::string::npos) {
      Serial.println("=== Temperature Configruation");
      JsonObject& root = jsonBuffer.parseObject(INPUT_JSON);
      
      char enable_char[20];
      char min_char[20];
      char max_char[20];

      char switch_char[20];

      strcpy(enable_char, root["enable"].as<String>().c_str());
      preferences.putString("ENABLE", enable_char);
      set_enable = (uint8_t) atoi(enable_char);
      Serial.println("- Enable: " + String(set_enable));

      strcpy(min_char, root["min"].as<String>().c_str());
      preferences.putString("MIN", min_char);
      set_min = (uint8_t) atoi(min_char);
      Serial.println("- Min: " + String(set_min));

      strcpy(max_char, root["max"].as<String>().c_str());
      preferences.putString("MAX", max_char);
      set_max = (uint8_t) atoi(max_char);
      Serial.println("- Max: " + String(set_max));

      strcpy(switch_char, root["switch"].as<String>().c_str());
      uint8_t switch_int = (uint8_t) atoi(switch_char);
      Serial.println("- Switch: " + String(switch_int));
      digitalWrite(SWITCH_PIN, switch_int);

      INPUT_JSON = "";
    }
  }
};
void setBle() {
  Serial.println("Set up Bluetooth...");
  BLEDevice::init("IoTBB-SmartControl");
  pServer = BLEDevice::createServer();

  pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic->addDescriptor(new BLE2902());
  pServer->setCallbacks(new MyServerCallbacks());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("IoTBB-SmartControl");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Start Smart Control...");
  pinMode(BLE_PIN, OUTPUT); // Blue tooth
  pinMode(SWITCH_PIN, OUTPUT); // Switch
  pinMode(TEMPERATURE_PIN, INPUT); // Temperature
  pinMode(SWITCH_STATUS_PIN, INPUT); // Switch status

  preferences.begin("my-app", false);
  char set_enable_char[1];
  char set_min_char[20];
  char set_max_char[20];
  preferences.getString("ENABLE", set_enable_char, 1);
  preferences.getString("MIN", set_min_char, 20);
  preferences.getString("MAX", set_max_char, 20);
  if (set_enable_char[0] != 0) {
    set_enable = (uint8_t)atoi(set_enable_char);
  } else {
    set_enable_char[0] = '0';
    preferences.putString("ENABLE", set_enable_char);
  }

  if (set_min_char[0] != 0) {
    set_min = (uint8_t)atoi(set_min_char);
  } else {
    set_min_char[0] = '0';
    preferences.putString("MIN", set_min_char);
  }

  if (set_max_char[0] != 0) {
    set_max = (uint8_t)atoi(set_max_char);
  } else {
    set_max_char[0] = '1';
    set_max_char[1] = '0';
    set_max_char[2] = '0';
    preferences.putString("MAX", set_max_char);
  }
  temperature = dht.readTemperature();
  setBle();
  Serial.print("- Enable: ");
  Serial.println(set_enable_char[0]);
  Serial.print("- Set Min: ");
  Serial.println(set_min_char);
  Serial.print("- Set Max: ");
  Serial.println(set_max_char);
}

void loop() {
  // Serial.println("=== enabled" + String(set_enable));
  if (set_enable == check_enable) {
    // Serial.println("=== enabled");
    if (set_min > temperature) {
      digitalWrite(SWITCH_PIN, HIGH);
    } else if (set_max < temperature) {
      digitalWrite(SWITCH_PIN, LOW);
    }
  }
  // notify changed value
  if (bleConnected && delay_count == 100) {
    temperature = dht.readTemperature();
    status = digitalRead(SWITCH_STATUS_PIN);
    // Serial.println();
    data[0] = set_enable;
    data[1] = set_min;
    data[2] = set_max;
    data[3] = temperature;
    data[4] = status;
    // Serial.println("===========");
    // Serial.println(set_enable);
    // Serial.println(set_min);
    // Serial.println(set_max);
    // Serial.println(temperature);
    // Serial.println(status);
    pCharacteristic->setValue(data, DATA_SIZE);
    pCharacteristic->notify();
  }
  if (delay_count > max) {
      delay_count = 0;
  } 
  // disconnecting
  if (!bleConnected && oldBleConnected) {
    // Serial.println(status);
    // Serial.println(temperature);
    // Serial.println(set_value);eConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("Start advertising");
      oldBleConnected = bleConnected;
  }
  // connecting
  if (bleConnected && !oldBleConnected) {
      // do stuff here on connecting
      oldBleConnected = bleConnected;
  }
  delay(10);
  delay_count++;
}