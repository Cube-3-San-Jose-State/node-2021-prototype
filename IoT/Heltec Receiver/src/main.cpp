#include "heltec.h"
#include "LoRa.h"
#include "HTTPClient.h"
#include "WiFiMulti.h"
#include "ArduinoJson.h"
#include "Adafruit_Sensor.h"

const char *AP_SSID = "kingdom2";
const char *AP_PWD = "22039622";

String Packet, GPS_location;
float temperature, humidity;

WiFiMulti wifiMulti;

void LoraRecieve()
{
  // try to parse packet
  int packetSize = LoRa.parsePacket();

  if (LoRa.available())
  {
    Serial.println("Packet Received: ");
    Packet = LoRa.readString();
  }
  Serial.println(Packet);
  sscanf(Packet.c_str(), "%f\n%f\n%[^&]\n", &temperature, &humidity, &GPS_location);
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