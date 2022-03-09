// LoRa and Heltec
#include <heltec.h>
#include <LoRa.h>
// DHT Sensor -- temperature & humidity
#include <DHT.h>
#include <Adafruit_Sensor.h>
// GPS Module
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// MPU Library -- IMU device
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <node.hpp>

// MPU
Adafruit_MPU6050 mpu;
String mpu_data;
// Smoke Sensor
#define MQ2PIN (32)
float sensorValue;
bool Smoke = false;
// Temperature Sensor
String temp_string;
#define DHT11PIN 33
DHT dht(DHT11PIN, DHT11);
float currentTemp;
float currentHumidity;
// GPS
String gps_string;
double Lon, Lat;
TinyGPSPlus gps;
static const int RXPin = 17, TXPin = 2;
static const uint32_t GPSBaud = 9600;

Node transmitter_node;



void setup()
{

}

void loop()
{
  transmitter_node.CreateLoRaPacket();
  transmitter_node.SendLoRaPacket();
  delay(3000);
}