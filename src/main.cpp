#include <Arduino.h>

#include <SPI.h>
#include <BLEPeripheral.h>
#include <BLESerial.h>
#include <Wire.h>

#ifdef __cplusplus
extern "C" {
#endif
void TIMER2_IRQHandler(void);
#ifdef __cplusplus
}
#endif

// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// TimerOne library: https://code.google.com/p/arduino-timerone/
//#include <TimerOne.h>
// DHT library: https://github.com/adafruit/DHT-sensor-library
#include "DHT.h"

/* define pins (varies per shield/board)
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9 */

#define SENSOR_PIN 4
#define DHTTYPE   DHT22
#define DHTPIN    10

DHT dht(DHTPIN, DHTTYPE);

//BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
BLEPeripheral blePeripheral = BLEPeripheral();

BLEService tempService = BLEService("CCC0");
BLEFloatCharacteristic tempCharacteristic = BLEFloatCharacteristic("CCC1", BLERead | BLENotify);
BLEDescriptor tempDescriptor = BLEDescriptor("2901", "Temp Celsius");

BLEService humidityService = BLEService("DDD0");
BLEFloatCharacteristic humidityCharacteristic = BLEFloatCharacteristic("DDD1", BLERead | BLENotify);
BLEDescriptor humidityDescriptor = BLEDescriptor("2901", "Humidity Percent");


// create service
BLEService              readSensorService           = BLEService("19b10000e8f2537e4f6cd104768a1214");
// create switch characteristic to turn on sensor
BLECharCharacteristic   switchCharacteristic = BLECharCharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);

volatile bool readFromSensor = false;

float lastTempReading;
float lastHumidityReading;
/*
void TIMER2_IRQHandler(void)
{
  if ((NRF_TIMER2->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
  {
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;           //Clear compare register 0 event
    readFromSensor = true;
  }

  if ((NRF_TIMER2->EVENTS_COMPARE[1] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
  {
    NRF_TIMER2->EVENTS_COMPARE[1] = 0;           //Clear compare register 1 event
    //readFromSensor = false;
  }
}

void start_timer(void)
{
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Counter Mode
  NRF_TIMER2->TASKS_CLEAR = 1;               // clear the task first to be usable for later
  NRF_TIMER2->PRESCALER = 6;                             //Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz timer
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;     //Set counter to 16 bit resolution
  NRF_TIMER2->CC[0] = 25000;                             //Set value for TIMER2 compare register 0
  NRF_TIMER2->CC[1] = 1000;                                 //Set value for TIMER2 compare register 1

  // Enable interrupt on Timer 2, both for CC[0] and CC[1] compare match events
  NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos) | (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
  NVIC_EnableIRQ(TIMER2_IRQn);

  NRF_TIMER2->TASKS_START = 1;               // Start TIMER2
}
*/
boolean significantChange(float val1, float val2, float threshold) {
  return (abs(val1 - val2) >= threshold);
}

void setTempCharacteristicValue() {
  float reading = dht.readTemperature();
//  float reading = random(100);

  if (!isnan(reading) && significantChange(lastTempReading, reading, 0.5)) {
    tempCharacteristic.setValue(reading);

    Serial.print(F("Temperature: ")); Serial.print(reading); Serial.println(F("C"));

    lastTempReading = reading;
  }
}

void setHumidityCharacteristicValue() {
  float reading = dht.readHumidity();
//  float reading = random(100);

  if (!isnan(reading) && significantChange(lastHumidityReading, reading, 1.0)) {
    humidityCharacteristic.setValue(reading);

    Serial.print(F("Humidity: ")); Serial.print(reading); Serial.println(F("%"));

    lastHumidityReading = reading;
  }
}

void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
}

void switchCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print(F("Characteristic event, writen: "));

  if (switchCharacteristic.value()) {
    Serial.println(F("LED on"));
    digitalWrite(SENSOR_PIN, HIGH);
    setTempCharacteristicValue();
    setHumidityCharacteristicValue();
  } else {
    Serial.println(F("LED off"));
    digitalWrite(SENSOR_PIN, LOW);
  }
}

void setup() {
  Serial.begin(115200);
#if defined (__AVR_ATmega32U4__)
  delay(5000);  //5 seconds delay for enabling to see the start up comments on the serial board
#endif

  blePeripheral.setLocalName("nRF_Temp");

  blePeripheral.setAdvertisedServiceUuid(tempService.uuid());
  blePeripheral.addAttribute(tempService);
  blePeripheral.addAttribute(tempCharacteristic);
  blePeripheral.addAttribute(tempDescriptor);

  blePeripheral.setAdvertisedServiceUuid(humidityService.uuid());
  blePeripheral.addAttribute(humidityService);
  blePeripheral.addAttribute(humidityCharacteristic);
  blePeripheral.addAttribute(humidityDescriptor);

  
  blePeripheral.setAdvertisedServiceUuid(readSensorService.uuid());
  blePeripheral.addAttribute(readSensorService);
  blePeripheral.addAttribute(switchCharacteristic);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  blePeripheral.begin();
  
  // assign event handlers for characteristic
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);

  //start_timer();

  Serial.println(F("BLE Temperature Sensor Peripheral"));
}

void loop() {
  blePeripheral.poll();
  /*
  if (readFromSensor) {
    setTempCharacteristicValue();
    setHumidityCharacteristicValue();
    readFromSensor = false;
  } */
    
}


