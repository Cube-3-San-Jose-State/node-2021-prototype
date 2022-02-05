#include "heltec.h"
#include "LoRa.h"
#include "HTTPClient.h"
#include "WiFiMulti.h"
#include "ArduinoJson.h"
#include "Adafruit_Sensor.h"

const char *AP_SSID = "kingdom2";
const char *AP_PWD = "22039622";

String Packet;
float temperature, humidity, mpu_accel_x, mpu_accel_y, mpu_accel_z, smoke, longitude, latitude;

WiFiMulti wifiMulti;

void LoraRecieve()
{
  // parse packet
  LoRa.parsePacket();
  if (LoRa.available())
  {
    Serial.println("Packet Received: ");
    Packet = LoRa.readString();
  }
  // Parse will need to be tested and verfied
  Serial.println(Packet);
  sscanf(Packet.c_str(), "%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n", &temperature, &humidity, &mpu_accel_x, &mpu_accel_y, &mpu_accel_z, &smoke, &longitude, &latitude);
  delay(10000);
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, false /*Serial Enable*/);
  Serial.println("LoRa Receiver");
  if (!LoRa.begin(915E6, 1))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  wifiMulti.addAP(AP_SSID, AP_PWD);
}

void loop()
{
  LoraRecieve();
}