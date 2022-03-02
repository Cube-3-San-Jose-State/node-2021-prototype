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

String GetGPS()
{
  while (Serial2.available())
  {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated())
    {
      Lat = gps.location.lat();
      Lon = gps.location.lng();
      gps_string = (String)Lat + ",\n" + (String)Lon;
      Serial.println(gps_string);
    }
  }
  return gps_string;
}

String GetMpu()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Accel Values
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  // Gyro Values
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  // Temo Values
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  mpu_data = (String)a.acceleration.x + ",\n" + (String)a.acceleration.y + ",\n" + (String)a.acceleration.z;
  return mpu_data;
}

bool GetSmoke()
{
  pinMode(MQ2PIN, INPUT);
  sensorValue = analogRead(MQ2PIN);
  Serial.print("Sensor Value: ");
  Smoke = (sensorValue > 750) ? Smoke = true : Smoke = false;
  (Smoke == true) ? Serial.println("Smoke detected!!!!\n") : Serial.println("Smoke not detected\n");
  return Smoke;
}

String GetTemp()
{
  currentHumidity = dht.readHumidity();
  currentTemp = dht.readTemperature();
  Serial.println("Temperature: " + (String)currentTemp + "°C");
  Serial.println("Temperature: " + (String)(1.8 * currentTemp + 32) + "°F");
  Serial.println("Humidity: " + (String)currentHumidity + "%\n");
  temp_string = (String)(1.8 * currentTemp + 32) + "°F" + ", " + (String)currentHumidity + "%";
  return temp_string;
}

void CompileSensors()
{
  // Printing
  GetTemp();
  GetMpu();
  GetSmoke();
  GetGPS();
  // LoRa output
  LoRa.println(GetTemp());
  LoRa.println(GetMpu());
  LoRa.println(GetSmoke());
  LoRa.println(GetGPS());
}

void SendLoRaPacket()
{
  LoRa.beginPacket();
  CompileSensors();
  LoRa.endPacket();
}

void displayOnBoard()
{
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Transceiver Operational");
  Heltec.display->display();
}

void setup()
{
  Serial.begin(115200);
  mpu.begin(115200);
  Serial2.begin(GPSBaud, SERIAL_8N1, TXPin, RXPin); // GPS Serial Baud-Rate
  Serial.println("Uploading GPS");
  dht.begin(115200);

  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, false /*Serial Enable*/);
  Serial.println("LoRa Sender starting...");
  if (!LoRa.begin(915E6, 1))
  { // Set frequency to 433, 868 or 915MHz
    Serial.println("Could not find a valid LoRa transmitter, check pins used and wiring!");
  }
  displayOnBoard();
}

void loop()
{
  SendLoRaPacket();
  delay(3000);
}