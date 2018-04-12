// Custom-built IoT sensor powered by MySensors (https://www.mysensors.org)
//
// Author Ando Roots <ando@sqroot.eu> 2018

//#define MY_DEBUG

#define MY_NODE_ID 01
unsigned long SLEEP_TIME = 900000;

// Force a sending of reading values after N measurements
// even if the values have not changed
static const uint8_t FORCE_UPDATE_N_READS = 8;

// Sensor ID-s
#define CHILD_ID_TEMP 0
#define CHILD_ID_LIGHT 1
#define CHILD_ID_DOOR 2
#define CHILD_ID_HUM 3
#define CHILD_ID_HUMTEMP 4

// Radio
#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10
#define MY_RF24_IRQ_PIN 4

#include <SPI.h>
#include <MySensors.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Bounce2.h>
#include <BH1750.h>
#include <Wire.h>
#include <DHT.h>

// Humidity sensor
#define DHT_DATA_PIN 7
#define SENSOR_TEMP_OFFSET 0

// Humidity sensor temp
float lastTemp;
float lastHum;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_HUMTEMP, V_TEMP);
DHT dht;


// 1-wire temp sensor
#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float lastTemperature;
MyMessage tempMsg(0, V_TEMP);


// Battery percentage sensor
int BATTERY_SENSE_PIN = A0;
int oldBatteryPcnt = 0;


// Ambient light sensor
BH1750 lightMeter;
MyMessage lightMsg(CHILD_ID_LIGHT, V_LEVEL);
uint16_t lastlux;


// Door sensor
#define BUTTON_PIN 2
MyMessage doorMsg(CHILD_ID_DOOR, V_TRIPPED);
Bounce debouncer = Bounce();
int oldDoorValue = -1;


// Generic global variables
uint8_t loopCounter = 0;


void before()
{
  // Startup up the OneWire library
  sensors.begin();
}

void setup()
{
  analogReference(INTERNAL);

  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);

  Wire.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);

  // Setup the button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // After setting up the button, setup debouncer
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(30);

  dht.setup(DHT_DATA_PIN);
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());
}

void presentation() {
  
  sendSketchInfo("Bedroom Ambient Sensor", "1.0");

  present(CHILD_ID_TEMP, S_TEMP);

  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_DOOR, S_DOOR);

  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_HUMTEMP, S_TEMP);

}


void measureBattery() {

  // get the battery Voltage
  int sensorValue = analogRead(BATTERY_SENSE_PIN);

  // 1M, 470K divider across battery and using internal ADC ref of 1.1V
  // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
  // ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
  // 3.44/1023 = Volts per bit = 0.003363075

  int batteryPcnt = sensorValue / 10;

  if (oldBatteryPcnt != batteryPcnt || loopCounter == FORCE_UPDATE_N_READS) {
    sendBatteryLevel(batteryPcnt);
    oldBatteryPcnt = batteryPcnt;
  }
}

void measureTemperature() {
  
  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  sleep(conversionTime);

 
  // Fetch and round temperature to one decimal
  float temperature = static_cast<float>(static_cast<int>( sensors.getTempCByIndex(CHILD_ID_TEMP)  * 10.)) / 10.;

  
  if ((lastTemperature != temperature && temperature != -127.00 && temperature != 85.00) || loopCounter == FORCE_UPDATE_N_READS) {

      // Send in the new temperature
      send(tempMsg.setSensor(CHILD_ID_TEMP).set(temperature, 1));
      // Save new temperatures for next compare
      lastTemperature = temperature;
   }
}

void measureDoor() {

  debouncer.update();
  int doorValue = debouncer.read();

  if (doorValue != oldDoorValue || loopCounter == FORCE_UPDATE_N_READS) {
    // Send in the new value
    send(doorMsg.set(doorValue == HIGH ? 1 : 0));
    oldDoorValue = doorValue;
  }

}

void measureLight() {
  uint16_t lux = lightMeter.readLightLevel();
  if (lux != lastlux || loopCounter == FORCE_UPDATE_N_READS) {
    send(lightMsg.set(lux));
    lastlux = lux;
  }
}


void measureHumidity() {

  // Force reading sensor, so it works also after sleep()
  dht.readSensor(true);

  float temperature = dht.getTemperature();
  if (!isnan(temperature) && (temperature != lastTemp || loopCounter == FORCE_UPDATE_N_READS)) {
    lastTemp = temperature;
   
    temperature += SENSOR_TEMP_OFFSET;
    send(msgTemp.set(temperature, 1));
  }

  
  float humidity = dht.getHumidity();
  if (!isnan(temperature) && (humidity != lastHum || loopCounter == FORCE_UPDATE_N_READS)) {
    lastHum = humidity;
   
    send(msgHum.set(humidity, 1));
  } 
}


void loop()
{
  
  measureTemperature();
  measureBattery();
  measureDoor();
  measureLight();
  measureHumidity();

  if (loopCounter == FORCE_UPDATE_N_READS) {
    loopCounter = 0;
  } else {
    loopCounter++;
  }

  // Sleep to conserve power between sensor reads,
  // except when the door sensor triggers - then, wake immediately
  sleep(digitalPinToInterrupt(BUTTON_PIN), CHANGE, SLEEP_TIME, false);
}

