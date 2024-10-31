#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "Adafruit_SHT31.h" // change to bme280// change to aht20
#include <BH1750.h> // change to tsl2561
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BME280.h>

#include "DFRobot_PH.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include <util/crc16.h>
#include "GravityTDS.h"
#include <Adafruit_AHTX0.h>

#define DEBUG

#define CLIENT_ID "esp32-tower"

// other defines:
#define ESP_SERIAL_BAUD 9600
#define ESP_SERIAL Serial1
#define WATER1_LEVEL 12 // the main tank water level
#define WATER2_LEVEL 5  // the nutrition tank water level
#define WATER3_LEVEL 5  // the nutrition tank water level
#define UMBRELLA_DELAY_TIME 24000 // 24s

// define pin numbers:
#define PH_PIN A0  // PH sensor    (place in the tower tank)
#define TDS_PIN A1 // TDS sensor    (place in the tower tank)

#define DS18B20_PIN 30        // water temperature sensor (place in the tower tank)
#define RAIN_PIN 31           // rain sensor (normal one) (place in the tower tank)
#define LIGHT_PIN_SCL         // light sensor SCL pin
#define LIGHT_PIN_SDA         // light sensor SDA pin
#define SHT_PIN_SCL           // SHT sensor SCL pin (temperature and humidity)
#define SHT_PIN_SDA           // SHT sensor SDA pin (temperature and humidity)
#define HC_SR04_PIN_TRIG_1 32 // HC-SR04 sensor TRIG pin (water level of main tank)
#define HC_SR04_PIN_ECHO_1 34 // HC-SR04 sensor ECHO pin (water level of main tank)
#define HC_SR04_PIN_TRIG_2 33 // HC-SR04 sensor TRIG pin (water level of nutrition 1 tank)
#define HC_SR04_PIN_ECHO_2 35 // HC-SR04 sensor ECHO pin (water level of nutrition 1 tank)
#define HC_SR04_PIN_TRIG_3 36 // HC-SR04 sensor TRIG pin (water level of nutrition 2 tank)
#define HC_SR04_PIN_ECHO_3 38 // HC-SR04 sensor ECHO pin (water level of nutrition 2 tank)
#define CTHT_TREN_UMBRELLA_PIN 37 // CTHT sensor pin (for umbrella)
#define CTHT_DUOI_UMBRELLA_PIN 39 // CTHT sensor pin (for umbrella)

#define RELAY_PUMP_1_PIN 22  // relay for 2 pump (pump from nutrition tank to tower tank)
#define RELAY_PUMP_2_PIN 23  // relay for pump (pump from main tank to tower tank)
#define RELAY_PUMP_3_PIN 24  // relay for pump (pump for tower tank)
#define RELAY_LIGHT_1_PIN 25 // relay for LIGHT (light control for tower tank)
#define RELAY_LIGHT_2_PIN 26 // relay for insect killer light(insect killer control for tower tank)
#define RELAY_FAN_1_PIN 27   // relay for overheat fan ()

#define BTS_TOWER_RPWM_PIN 4    // BTS7960 RPWM pin (for tunring tower left and right)
#define BTS_TOWER_LPWM_PIN 5    // BTS7960 LPWM pin (for tunring tower left and right)
#define BTS_UMBRELLA_RPWM_PIN 6 // BTS7960 RPWM pin (for open/close umbrella)
#define BTS_UMBRELLA_LPWM_PIN 7 // BTS7960 LPWM pin (for open/close umbrella)
#define BTS_HEATER_RPWM_PIN 2   // BTS7960 RPWM pin (for x3 heater)
#define BTS_HEATER_LPWM_PIN 3   // BTS7960 LPWM pin (for x3 heater)

// function declarations:
void readSHT();
void readLight();
bool readRain();
void readPH();
void readTDS();
void readWaterTemp();
float readDistance(int trigPin, int echoPin);
void relayControl(int relayPin, bool isOn);
void btsControl(byte rpwmPin, byte lpwmPin, byte speed, byte direction);
void warningLight(int pin, int times);
void sendDataToESP();
void handleSerialESP();

// libraries variables:
Adafruit_SHT31 sht31 = Adafruit_SHT31();
BH1750 lightMeter(0x23);

GravityTDS tds;
OneWire oneWire(DS18B20_PIN);
DallasTemperature DS18B20(&oneWire);
DFRobot_PH ph;

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
// Adafruit_BME280 bme;
Adafruit_AHTX0 aht;