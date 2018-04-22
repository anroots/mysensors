// Custom-built IoT sensor powered by MySensors (https://www.mysensors.org)
//
// Author Ando Roots <ando@sqroot.eu> 2018

//#define MY_DEBUG

#define MY_NODE_ID 02
unsigned long SLEEP_TIME = 900000;

// Force a sending of reading values after N measurements
// even if the values have not changed
static const uint8_t FORCE_UPDATE_N_READS = 8;

// Sensor ID-s
#define CHILD_ID_LIGHT 1
#define CHILD_ID_HUM 2
#define CHILD_ID_HUMTEMP 3

// Radio
#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10
#define MY_RF24_IRQ_PIN 4

#include <MySensors.h>
#include <BH1750.h>
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

// Battery percentage sensor
int BATTERY_SENSE_PIN = A0;
int oldBatteryPcnt = 0;


// Ambient light sensor
BH1750 lightMeter;
MyMessage lightMsg(CHILD_ID_LIGHT, V_LEVEL);
uint16_t lastlux;

// Generic global variables
uint8_t loopCounter = 0;


void setup()
{
  analogReference(INTERNAL);


  Wire.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);


  dht.setup(DHT_DATA_PIN);
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht.getMinimumSamplingPeriod());
}

void presentation() {
  
  sendSketchInfo("ATC Node02", "1.0");

  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
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
  
  measureBattery();
  measureLight();
  measureHumidity();

  if (loopCounter == FORCE_UPDATE_N_READS) {
    loopCounter = 0;
  } else {
    loopCounter++;
  }

  sleep(SLEEP_TIME);
}

