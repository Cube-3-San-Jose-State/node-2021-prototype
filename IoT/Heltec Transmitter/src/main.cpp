// LoRa and Heltec
#include <heltec.h>
#include <LoRa.h>
// DHT Sensor
#include <DHT.h>
#include <Adafruit_Sensor.h>
// GPS Library
#include <TinyGPS++.h>
// MPU Library
#include <Adafruit_MPU6050.h>
#include <Wire.h>

// MPU
Adafruit_MPU6050 mpu;

// Temperature Sensor
#define DHT11PIN 33
DHT dht(DHT11PIN, DHT11);
float currentTemp;
float currentHumidity;
// GPS Module
TinyGPSPlus gps;
static const int RXPin = 17, TXPin = 2;
static const uint32_t GPSBaud = 9600;

// Displays GPS module data using TinyGPS Library
void displayGPS()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

// Uses Serial port to acquire GPS data
void GPSInfo()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayGPS();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true)
      ;
  }
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

// Compiling all sensors
void CompileSensors()
{

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  currentHumidity = dht.readHumidity();
  currentTemp = dht.readTemperature();
  Serial.println("Temperature: " + (String)currentTemp + "°C");
  Serial.println("Temperature: " + (String)(1.8 * currentTemp + 32) + "°F");
  Serial.println("Humidity: " + (String)currentHumidity + "%\n");
  //mpuData();
  GPSInfo();
  delay(3000);

  // Sending the Data so it can be parsed by the reciever
  //  LoRa.println(1.8*currentTemp+32);
  //  LoRa.println(currentHumidity);
  //  LoRa.println(Lat);
  //  LoRa.println(Lon);
  //  LoRa.println(Smoke);

  // TEMP
  LoRa.println("Temperature: " + (String)currentTemp + "°C");
  LoRa.println("Temperature: " + (String)(1.8 * currentTemp + 32) + "°F");
  LoRa.println("Humidity: " + (String)currentHumidity + "%");
  // MPU
  LoRa.print("Acceleration X: ");
  LoRa.print(a.acceleration.x);
  LoRa.print(", Y: ");
  LoRa.print(a.acceleration.y);
  LoRa.print(", Z: ");
  LoRa.print(a.acceleration.z);
  LoRa.println(" m/s^2");
  // GPS
  LoRa.print(gps.location.lat(), 6);
  LoRa.print(F(","));
  LoRa.print(gps.location.lng(), 6);
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

// void loop()
// {
//   //
//   // CompileSensors();

//   GPSInfo();

//   //delay(1000);
// }

void loop()
{

  CompileSensors();
}