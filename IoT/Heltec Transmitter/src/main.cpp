#include <heltec.h>
#include <LoRa.h>
#include <DHT.h>

#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <SoftwareSerial.h>

//MPU
Adafruit_MPU6050 mpu;

//Temperature Sensor
#define DHT11PIN 33
DHT dht(DHT11PIN, DHT11);
float currentTemp;
float currentHumidity;

double Lon, Lat;
TinyGPSPlus gps;
static const int RXPin = 17, TXPin = 2;
static const uint32_t GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);


void wakeGPS(){
    while (gpsSerial.available()){
      Serial.println("GPS found");
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()){

      Lat = gps.location.lat();
      Lon = gps.location.lng();

      Serial.print("Latitude: " + (String)Lat + "\nLongitude: " + (String)Lon);
      //LoRa.print("Latitude: " + (String)Lat + "\nLatitude: " + (String)Lon);
    }
  }
}


void mpuData(){
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

void CompileSensors(){

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("Temperature: " + (String)currentTemp +  "°C");
  Serial.println("Temperature: " + (String)(1.8*currentTemp+32) + "°F");
  Serial.println("Humidity: " + (String)currentHumidity + "%\n");
  mpuData();
  wakeGPS();

  //Sending the Data so it can be parsed by the reciever
  // LoRa.println(1.8*currentTemp+32);
  // LoRa.println(currentHumidity);
  // LoRa.println(Lat);
  // LoRa.println(Lon);
  // LoRa.println(Smoke);

  //TEMP
  LoRa.println("Temperature: " + (String)currentTemp +  "°C");
  LoRa.println("Temperature: " + (String)(1.8*currentTemp+32) + "°F");
  LoRa.println("Humidity: " + (String)currentHumidity + "%");
  //MPU
  LoRa.print("Acceleration X: ");
  LoRa.print(a.acceleration.x);
  LoRa.print(", Y: ");
  LoRa.print(a.acceleration.y);
  LoRa.print(", Z: ");
  LoRa.print(a.acceleration.z);
  LoRa.println(" m/s^2");
  //GPS
  LoRa.println("Latitude: " + (String)Lat + "\nLatitude: " + (String)Lon);
  

  
}


void SendLoRaPacket(){
  LoRa.beginPacket();
  CompileSensors();
  LoRa.endPacket();
}


void displayOnBoard() {
   
  String CtemperatureDisplay ="Temperature: " + (String)currentTemp +  "°C";
  String FtemperatureDisplay ="Temperature: " + (String)(1.8*currentTemp+32) + "°F";
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
  Serial.begin(GPSBaud);
  dht.begin();

    // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, false /*Serial Enable*/);


  
  Serial.println("LoRa Sender starting...");

  if (!LoRa.begin(915E6, 1)) 
  { // Set frequency to 433, 868 or 915MHz
    Serial.println("Could not find a valid LoRa transceiver, check pins used and wiring!");
  }

}
 
    
void loop()
{
  // currentHumidity = dht.readHumidity();
  // currentTemp = dht.readTemperature();
   displayOnBoard();
  // SendLoRaPacket();

  //mpuData();
  //wakeGPS();
  delay(1000);
  //Serial.print("hello");
  
  

}