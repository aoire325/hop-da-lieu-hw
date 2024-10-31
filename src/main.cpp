#include <Arduino.h>
#define SERIAL_RX_BUFFER_SIZE 1024
#include <config.h>

bool isShtError = false;

unsigned long lasControl = 15000; // on the first time, we need to read sensor first

float temp = -1, humi = -1, lux = -1;
float phVal = -1, tdsVal = -1, waterTemp = -1;
bool isRain = false, isWater1LevelLow = false, isWater2LevelLow = false, isWater3LevelLow = false;
bool isUmbrellaOpen = false, isUmbrellaMoving = false;
bool umbrellaControl = false;

int count_bts_heater = 0, count_pump_tower = 0, count_bts_umbrella = 0;
// for example: in web server we can set time to turn on/off heater
// so if we set time to turn on heater is 8:00 AM and turn off heater is 10:00 AM
// then in this time, isBtsHeaterAllowed = true, else isBtsHeaterAllowed = false
// we use esp32 to read time from server and then change this variable
bool isBtsHeaterAllowed = true;
bool isBtsHeaterOn = false;
unsigned long lastHeaterControl = 0;

unsigned long lastPumpTowerControl = 0;
int timePumpTowerControlOn = 15;  // 15 minutes
int timePumpTowerControlOff = 45; // 45 minutes
bool isPumpTowerAllowed = true;

bool isBtsTowerAllowed = true;
unsigned long timeBtsTowerChange = 0;
int timeBtsTowerChangeDelay = 1500; // 1.5 seconds
// web control
bool manualMode = false; // false: auto, true: manual
bool isLightControl = false, isInsectLightControl = false;
bool isPump1Control = false, isPump2Control = false, isPump3Control = false;

unsigned long lastReadSensor = 0; // 15 seconds

void setup()
{
  Serial.begin(115200);
  Serial1.begin(ESP_SERIAL_BAUD);

  pinMode(PH_PIN, INPUT);
  pinMode(TDS_PIN, INPUT);

  // pinmode for relay
  for (int i = 22; i < 30; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  // pinmode
  for (int i = 30; i <= 40; i++)
  {
    pinMode(i, INPUT);
  }
  pinMode(CTHT_DUOI_UMBRELLA_PIN, INPUT_PULLUP);
  // if the umbrella is open (or not at the bottom), then close it
  if (digitalRead(CTHT_DUOI_UMBRELLA_PIN) == HIGH)
  {
    btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 255, 0);
    while (digitalRead(CTHT_DUOI_UMBRELLA_PIN) == HIGH)
    {
      Serial.println("[INFO] --- Waiting for umbrella to reach the bottom");
      delay(1000);
    }
  }
  pinMode(HC_SR04_PIN_TRIG_1, OUTPUT);
  pinMode(HC_SR04_PIN_TRIG_2, OUTPUT);
  pinMode(HC_SR04_PIN_TRIG_3, OUTPUT);

  // pinmode for BTS7960
  for (int i = 2; i < 8; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  Wire.begin();
  // sht init
  // if (!sht31.begin(0x44))
  // {
  //   Serial.println("[Warning] Couldn't find SHT31");
  //   isShtError = true;
  // }

  // light init
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
  {
    Serial.println(F("[Warning] Error initialising BH1750 light sensor"));
  }
  // tsl2561 init
  if (!tsl.begin())
  {
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
  }
  // aht init
  if (!aht.begin())
  {
    isShtError = true;
    Serial.println("Could not find a valid AHT20 sensor, check wiring!");
  }

  // tds init
  tds.setPin(TDS_PIN);
  tds.setAref(5.0);      // reference voltage on ADC, default 5.0V on Arduino UNO
  tds.setAdcRange(1024); // 1024 for 10bit ADC;4096 for 12bit ADC
  tds.begin();           // initialization

  // ph init
  ph.begin();

  // water temperature init
  DS18B20.begin();

  // // turn on tower pump
  // relayControl(RELAY_PUMP_3_PIN, true);
}

void loop()
{
  handleSerialESP();
  if (millis() - lastReadSensor > 5 * 1000)
  {
    Serial.println("[INFO] --- Reading sensor");
    lastReadSensor = millis();
    count_bts_umbrella++;
    readSHT();
    readLight();
    readPH();
    readTDS();
    readWaterTemp();
    isRain = readRain();
    isWater1LevelLow = readDistance(HC_SR04_PIN_TRIG_1, HC_SR04_PIN_ECHO_1) > WATER1_LEVEL;
    isWater2LevelLow = readDistance(HC_SR04_PIN_TRIG_2, HC_SR04_PIN_ECHO_2) > WATER2_LEVEL;
    isWater3LevelLow = readDistance(HC_SR04_PIN_TRIG_3, HC_SR04_PIN_ECHO_3) > WATER2_LEVEL;

    Serial.println("[INFO] --- Sending data to ESP32");
    sendDataToESP();
  }

  if (millis() - lasControl > 3 * 1000)
  {
    lasControl = millis();

    if (!manualMode) // auto mode
    {

      // newer code for controlling umbrella using CTHT sensor
      if ((isRain || (lux > 20000 && lux < 60000)) && !isUmbrellaOpen && !isUmbrellaMoving)
      {
        Serial.println("[Warning - AUTO] --- Rain or too much light. Opening umbrella");
        btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 255, 0);
        delay(UMBRELLA_DELAY_TIME);
        btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 0, 0);
        isUmbrellaOpen = true;
      }
      if (digitalRead(CTHT_DUOI_UMBRELLA_PIN) == LOW)
      {
        Serial.println("[Warning - AUTO] --- Umbrella reached the bottom. Stopping umbrella");
        isUmbrellaMoving = false;
        isUmbrellaOpen = false;
        btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 0, 0);
      }
      if (isUmbrellaOpen && !isUmbrellaMoving && (!isRain && lux < 20000))
      {
        Serial.println("[Warning - AUTO] --- Not rain or light. Closing umbrella");
        btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 255, 1);
        isUmbrellaMoving = true;
      }

      // warning light for hc-sr04---------------------------------------
      if (isWater1LevelLow) // if water level of main tank is low
      {
        Serial.println("[Warning - AUTO] --- Water level of main tank is low");
        // not enough water in main tank so no more pumping
        relayControl(RELAY_PUMP_2_PIN, false); // turn off the pump
        warningLight(RELAY_LIGHT_1_PIN, 5);    // warning light
      }
      // warning light for hc-sr04---------------------------------------
      if (isWater2LevelLow || isWater3LevelLow) // if water level of nutrition tank is low
      {
        Serial.println("[Warning - AUTO] --- Water level of nutrition tank is low");
        // not enough water in nutrition tank so no more pumping
        relayControl(RELAY_PUMP_1_PIN, false); // turn off the pump
        warningLight(RELAY_LIGHT_1_PIN, 3);    // warning light
      }

      // insect light handling---------------------------------------------
      if (lux < 10)
      {
        Serial.println("[INFO - AUTO] --- Insect light is on");
        relayControl(RELAY_LIGHT_2_PIN, true);
      }
      else
      {
        Serial.println("[INFO - AUTO] --- Insect light is off");
        relayControl(RELAY_LIGHT_2_PIN, false);
      }
      if (lux < 1)
      {
        Serial.println("[INFO - AUTO] --- Light is on");
        relayControl(RELAY_LIGHT_1_PIN, true);
      }
      else
      {
        Serial.println("[INFO - AUTO] --- Light is off");
        relayControl(RELAY_LIGHT_1_PIN, false);
      }
    }
    else
    {                     // manual mode
      if (isLightControl) // light control
      {
        Serial.println("[INFO - MANUAL] --- Light is on");
        relayControl(RELAY_LIGHT_1_PIN, true);
      }
      else
      {
        Serial.println("[INFO - MANUAL] --- Light is off");
        relayControl(RELAY_LIGHT_1_PIN, false);
      }

      if (isInsectLightControl) // insect light control
      {
        Serial.println("[INFO - MANUAL] --- Insect light is on");
        relayControl(RELAY_LIGHT_2_PIN, true);
      }
      else
      {
        Serial.println("[INFO - MANUAL] --- Insect light is off");
        relayControl(RELAY_LIGHT_2_PIN, false);
      }
      // umbrella control
      if (umbrellaControl && !isUmbrellaOpen)
      {
        Serial.println("[Warning - MANUAL] --- Opening umbrella");
        btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 255, umbrellaControl);
        delay(UMBRELLA_DELAY_TIME);
        btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 0, 0);
        isUmbrellaOpen = true;
      }
      else if (!umbrellaControl && isUmbrellaOpen)
      {
        Serial.println("[Warning - MANUAL] --- Closing umbrella");
        btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 255, umbrellaControl);
        isUmbrellaMoving = true;
      }
      if (digitalRead(CTHT_DUOI_UMBRELLA_PIN) == LOW)
      {
        Serial.println("[Warning - MANUAL] --- Umbrella reached the bottom. Stopping umbrella");
        isUmbrellaMoving = false;
        isUmbrellaOpen = false;
        btsControl(BTS_UMBRELLA_RPWM_PIN, BTS_UMBRELLA_LPWM_PIN, 0, 0);
      }

      // pump control handling---------------------------------------------
      if (isPump1Control)
      {
        relayControl(RELAY_PUMP_1_PIN, true);
      }
      else
      {
        relayControl(RELAY_PUMP_1_PIN, false);
      }

      if (isPump2Control)
      {
        relayControl(RELAY_PUMP_2_PIN, true);
      }
      else
      {
        relayControl(RELAY_PUMP_2_PIN, false);
      }

      if (isPump3Control)
      {
        relayControl(RELAY_PUMP_3_PIN, true);
      }
      else
      {
        relayControl(RELAY_PUMP_3_PIN, false);
      }
    }
  }
  // // bts tower control handling---------------------------------------------
  // currently not tested======================
  if (millis() - timeBtsTowerChange > 15 * 60000)
  {
    timeBtsTowerChange = millis();
    static bool isBtsTowerDirection = true;
    if (isBtsTowerAllowed)
    {
      Serial.println("[INFO] --- BTS Tower is moving");
      btsControl(BTS_TOWER_RPWM_PIN, BTS_TOWER_LPWM_PIN, 255, isBtsTowerDirection);
      isBtsTowerDirection = !isBtsTowerDirection;
      delay(timeBtsTowerChangeDelay);
      btsControl(BTS_TOWER_RPWM_PIN, BTS_TOWER_LPWM_PIN, 0, 0);
    }
    else
    {
      btsControl(BTS_TOWER_RPWM_PIN, BTS_TOWER_LPWM_PIN, 0, 0);
    }
  }

  // bts heater control handling---------------------------------------------
  //  120 on time, 60 off time for bts_heater
  if (millis() - lastHeaterControl > 15 * 1000)
  {
    lastHeaterControl = millis();
    count_bts_heater++;
    // heater on handling---------------------------------------------
    if (isBtsHeaterAllowed && count_bts_heater == 8 && !isBtsHeaterOn)
    {
      // turn on heater after 1 minute and can only be turned on if it is allowed
      Serial.println("[INFO] --- Heater is on");
      count_bts_heater = 0;
      isBtsHeaterOn = !isBtsHeaterOn;
      relayControl(RELAY_FAN_1_PIN, isBtsHeaterOn);
      btsControl(BTS_HEATER_RPWM_PIN, BTS_HEATER_LPWM_PIN, 255, 1);
    }
    // heater off handling---------------------------------------------
    if (count_bts_heater == 8 && isBtsHeaterOn) // 8 * 15s = 120s = 2 minutes
    {
      // turn off heater after 2 minutes or if it is on and run out of time then it can still be turned off
      Serial.println("[INFO] --- Heater is off");
      count_bts_heater = 0;
      isBtsHeaterOn = !isBtsHeaterOn;
      relayControl(RELAY_FAN_1_PIN, isBtsHeaterOn);
      btsControl(BTS_HEATER_RPWM_PIN, BTS_HEATER_LPWM_PIN, 0, 1);
    }
  }

  // pump tower control handling---------------------------------------------
  if (millis() - lastPumpTowerControl > 5000)
  {
    // 15mins on, 45mins off
    lastPumpTowerControl = millis();
    if (isPumpTowerAllowed)
    {
      if (count_pump_tower == 0)
      {
        relayControl(RELAY_PUMP_3_PIN, true);
      }
      else if (count_pump_tower == int(timePumpTowerControlOn / 5))
      {
        // turn off pump after 15 mins
        relayControl(RELAY_PUMP_3_PIN, false);
      }
      else if (count_pump_tower == int(timePumpTowerControlOff / 5))
      {
        // turn on pump after 1 hour
        relayControl(RELAY_PUMP_3_PIN, true);
        count_pump_tower = 0;
      }
    }
    count_pump_tower++;
  }
}

/**
 * @brief read temperature and humidity sensor
 */
void readSHT()
{
  if (!isShtError)
  {
    // temp = sht31.readTemperature();
    // humi = sht31.readHumidity();
    // temp = bme.readTemperature();
    // humi = bme.readHumidity();
    sensors_event_t h, t;
    aht.getEvent(&h, &t);
    humi = h.relative_humidity;
    temp = t.temperature;
#ifdef DEBUG
    Serial.print("[DEBUG] --- Temperature: ");
    Serial.print(temp);
    Serial.print("C, Humidity: ");
    Serial.print(humi);
    Serial.println("%");
#endif
  }
}

/**
 * @brief read light sensor
 */
void readLight()
{
  //   if (lightMeter.measurementReady())
  //   {
  //     lux = lightMeter.readLightLevel();
  // #ifdef DEBUG
  //     Serial.print("[DEBUG] ---Light: ");
  //     Serial.print(lux);
  //     Serial.println(" lx");
  // #endif
  //   }
  sensors_event_t event;
  tsl.getEvent(&event);
  if (event.light)
  {
    lux = event.light;
#ifdef DEBUG
    Serial.print("[DEBUG] ---Light: ");
    Serial.print(lux);
    Serial.println(" lx");
#endif
  }
  else
  {
    Serial.println("[Warning] --- Light sensor error");
  }
}

/**
 * @brief read rain sensor
 * @return true if rain, false if not
 */
bool readRain()
{
  return !digitalRead(RAIN_PIN);
}

/**
 * @brief read PH sensor
 */
void readPH()
{
  // remember to read water temperature sensor first
  float vol = analogRead(PH_PIN) * 5000.0 / 1024;
  phVal = ph.readPH(vol, waterTemp);
}

/**
 * @brief read TDS sensor
 */
void readTDS()
{
  // remember to read water temperature sensor first
  tds.setTemperature(waterTemp);
  tds.update();
  tdsVal = tds.getTdsValue();
}

/**
 * @brief read water temperature sensor
 */
void readWaterTemp()
{
  DS18B20.requestTemperatures();
  waterTemp = DS18B20.getTempCByIndex(0);
}

/**
 * @brief read distance sensor
 * @param trigPin trigger pin
 * @param echoPin echo pin
 * @return distance in cm
 */
float readDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

/**
 * @brief control relay
 * @param relayPin relay pin
 * @param isOn true if on, false if off
 */
void relayControl(int relayPin, bool isOn)
{
  // all relay is active low
  digitalWrite(relayPin, !isOn);
}

/**
 * @brief control BTS7960
 * @param rpwmPin RPWM pin
 * @param lpwmPin LPWM pin
 * @param speed speed of motor
 * @param direction direction of motor
 */
void btsControl(byte rpwmPin, byte lpwmPin, byte speed, byte direction)
{
  if (direction == 1)
  {
    analogWrite(rpwmPin, speed);
    analogWrite(lpwmPin, 0);
  }
  else
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, speed);
  }
}

/**
 * @brief warning light
 * to warning user about something using light
 */
void warningLight(int pin, int times)
{
  // turn on/off the light for times times
  for (int i = 0; i < times; i++)
  {
    digitalWrite(pin, HIGH);
    delay(500);
    digitalWrite(pin, LOW);
    delay(500);
  }
}

/**
 * @brief send data to esp32
 */
void sendDataToESP()
{
  JsonDocument doc;

  doc["clientID"] = CLIENT_ID;
  JsonObject data = doc["data"].to<JsonObject>();
  data["temp"] = temp;
  data["humi"] = humi;
  data["phVal"] = phVal;
  data["tdsVal"] = tdsVal;
  data["waterTemp"] = waterTemp;
  data["lux"] = lux;
  data["isRain"] = isRain;
  data["isWater1LevelLow"] = isWater1LevelLow;
  data["isWater2LevelLow"] = isWater2LevelLow;
  data["isWater3LevelLow"] = isWater3LevelLow;
  data["isUmbrellaOpen"] = isUmbrellaOpen;
  data["isBtsHeaterOn"] = isBtsHeaterOn;

  String output;

  doc.shrinkToFit(); // optional

  serializeJson(doc, output);

  Serial1.println(output);
}

/**
 * @brief receive data from esp32
 */
void handleSerialESP()
{
  if (Serial1.available())
  {
    String input = Serial1.readStringUntil('\n');
    //     // Serial1.flush();
    Serial.println("[INFO] --- Received data from ESP32");
    Serial.println(input);
    //
    //     JsonDocument doc;
    //     DeserializationError error = deserializeJson(doc, input);
    //     if (error)
    //     {
    //       Serial.print("deserializeJson() failed: ");
    //       Serial.println(error.c_str());
    //       return;
    //     }
    //
    //     manualMode = doc["a"];                            // 1 / 0
    //     isLightControl = doc["b"];                        // 1 / 0
    //     isInsectLightControl = doc["c"];                  // 1 / 0
    //     isPump1Control = doc["d"];                        // 1 / 0
    //     isPump2Control = doc["e"];                        // 1 / 0
    //     isPump3Control = doc["f"];                        // 1 / 0
    //     umbrellaControl = doc["g"];                       // 1 / 0
    //     isBtsTowerAllowed = doc["h"];                     // 1 / 0
    //     isBtsHeaterAllowed = doc["i"];                    // 1 / 0
    //     isPumpTowerAllowed = doc["j"];                    // 1 / 0
    //     timePumpTowerControlOn = doc["k"].as<int>();      // %5 == 0
    //     timePumpTowerControlOff = doc["l"].as<int>();     // %5 == 0
    // split the string into an array of strings
    String arr[12];
    int i = 0;
    for (int j = 0; j < input.length() - 1; j++)
    {
      if (input[j] == ' ')
      {
        i++;
      }
      else
      {
        arr[i] += input[j];
      }
    }

    arr[0] == "1" ? manualMode = true : manualMode = false;
    arr[1] == "1" ? isLightControl = true : isLightControl = false;
    arr[2] == "1" ? isInsectLightControl = true : isInsectLightControl = false;
    arr[3] == "1" ? isPump1Control = true : isPump1Control = false;
    arr[4] == "1" ? isPump2Control = true : isPump2Control = false;
    arr[5] == "1" ? isPump3Control = true : isPump3Control = false;
    arr[6] == "1" ? umbrellaControl = true : umbrellaControl = false;
    arr[7] == "1" ? isBtsTowerAllowed = true : isBtsTowerAllowed = false;
    arr[8] == "1" ? isBtsHeaterAllowed = true : isBtsHeaterAllowed = false;
    arr[9] == "1" ? isPumpTowerAllowed = true : isPumpTowerAllowed = false;
    timePumpTowerControlOn = arr[10].toInt();
    timePumpTowerControlOff = arr[11].toInt();
    Serial.println(manualMode);
  }
}
