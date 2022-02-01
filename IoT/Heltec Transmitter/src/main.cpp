// LoRa and Heltec
#include <heltec.h>
#include <LoRa.h>
// DHT Sensor
#include <DHT.h>
#include <Adafruit_Sensor.h>
// GPS Module
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// MPU Library
#include <Adafruit_MPU6050.h>
#include <Wire.h>

// MPU
Adafruit_MPU6050 mpu;

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

// Reading and Outputting MPU data
void mpuData()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

String GetTemp(){
  currentHumidity = dht.readHumidity();
  currentTemp = dht.readTemperature();
  Serial.println("Temperature: " + (String)currentTemp + "°C");
  Serial.println("Temperature: " + (String)(1.8 * currentTemp + 32) + "°F");
  Serial.println("Humidity: " + (String)currentHumidity + "%\n");
  temp_string = (String)(1.8 * currentTemp + 32) + "°F" + ", " + (String)currentHumidity + "%";
  return temp_string;
}

// Compiling all sensors
void CompileSensors()
{
  GetTemp();
  GetGPS();
  LoRa.println(GetTemp());
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
  //DHT SENSOR
  String CtemperatureDisplay = "Temperature: " + (String)currentTemp + "°C";
  String FtemperatureDisplay = "Temperature: " + (String)(1.8 * currentTemp + 32) + "°F";
  String humidityDisplay = "Humidity: " + (String)currentHumidity + "%";

  Heltec.display->clear();
  // Prepare to display temperature C and F
  Heltec.display->drawString(0, 0, CtemperatureDisplay);
  Heltec.display->drawString(0, 12, FtemperatureDisplay);
  // Prepare to display humidity
  Heltec.display->drawString(0, 24, humidityDisplay);
  // Display the readings
  Heltec.display->display();
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(GPSBaud, SERIAL_8N1, TXPin, RXPin); // GPS Serial Baud-Rate
  Serial.println("Uploading GPS");
  dht.begin(9600);

  // Try to initialize!
  // if (!mpu.begin(115200))
  // {
  //   Serial.println("Failed to find MPU6050 chip");
  //   while (1)
  //   {
  //     delay(10);
  //   }
  // }
  // Serial.println("MPU6050 Found!");

  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, false /*Serial Enable*/);

  Serial.println("LoRa Sender starting...");

  if (!LoRa.begin(915E6, 1))
  { // Set frequency to 433, 868 or 915MHz
    Serial.println("Could not find a valid LoRa transmitter, check pins used and wiring!");
  }
}

void loop()
{
  SendLoRaPacket();
  delay(3000);
}