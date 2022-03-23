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

class Node : public TransmitterInterface
{
public:
    Node()
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
        pinMode(kMq2Pin, INPUT);
        DisplayOnBoard();
    };

    SensorInterface::SensorData GetSensorData() override
    {
        return transmitter_sensor_data_;
    }

    void PrintData() override
    {
        // TODO;
    }

    void TransmitData() override
    {
        CreateLoRaPacket();
        SendLoRaPacket();
    }

private:
    struct TransmitterSensorData : public SensorInterface::SensorData
    {
        string gps_coordinates = "";
        string acceleration = "";
        string rotation = "";
        string temperature = "";
        bool is_smoke_detected = "";
    };

    void CreateLoRaPacket()
    {
        LoRa.println(GetTemperatureData());
        LoRa.println(GetAccelerometerData());
        LoRa.println(GetSmokeData());
        LoRa.println(GetGpsData());
    }

    void SendLoRaPacket()
    {
        BeginLoRaPacket();
        CreateLoRaPacket();
        SendLoRaPacket();
    }

    void DisplayOnBoard()
    {
        Heltec.display->clear();
        Heltec.display->drawString(0, 0, "Transceiver Operational");
        Heltec.display->display();
    }

    void BeginLoRaPacket()
    {
        LoRa.beginPacket();
    }

    string GetGpsData()
    {
        while (Serial2.available())
        {
            gps.encode(Serial2.read());
            if (gps.location.isUpdated())
            {
                Lat = gps.location.lat();
                Lon = gps.location.lng();
                string gps_string = (String)Lat + ",\n" + (String)Lon;
                Serial.println(gps_string);
                return gps_string;
            }
        }
        return "";
    }

    string GetAccelerometerData()
    {
        sensors_event_t accelerometer, gyroscope, temperature;
        mpu.getEvent(&accelerometer, &gyroscope, &temperature);

        // Accelerometer Values
        Serial.print("Acceleration X: ");
        Serial.print(accelerometer.acceleration.x);
        Serial.print(", Y: ");
        Serial.print(accelerometer.acceleration.y);
        Serial.print(", Z: ");
        Serial.print(accelerometer.acceleration.z);
        Serial.println(" m/s^2");
        // Gyroscope Values
        Serial.print("Rotation X: ");
        Serial.print(gyroscope.gyro.x);
        Serial.print(", Y: ");
        Serial.print(gyroscope.gyro.y);
        Serial.print(", Z: ");
        Serial.print(gyroscope.gyro.z);
        Serial.println(" rad/s");
        // Temperature Values
        Serial.print("Temperature: ");
        Serial.print(temperature.temperature);
        Serial.println(" degC");

        string mpu_data = (String)accelerometer.acceleration.x + ",\n" + (String)accelerometer.acceleration.y + ",\n" + (String)accelerometer.acceleration.z;
        return mpu_data;
    }

    bool GetSmokeData()
    {
        float sensor_value = analogRead(kMq2Pin);
        Serial.print("Sensor Value: ");
        bool smoke = (sensorValue > 750) ? smoke = true : smoke = false; // ternary initialize
        (smoke) ? Serial.println("Smoke detected!!!!\n") : Serial.println("Smoke not detected\n");
        return smoke;
    }

    string GetTemperatureData()
    {
        current_humidity = dht.readHumidity();
        current_temperature = dht.readTemperature();
        Serial.println("Temperature: " + (String)(1.8 * current_temperature + 32) + "°F");
        Serial.println("Humidity: " + (String)current_humidity + "%\n");
        string temperature_string = (String)(1.8 * current_temperature + 32) + "°F" + ", " + (String)current_humidity + "%";
        return temperature_string;
    }

    int const kMq2Pin = 32;
    int const kDht11Pin = 33;
    static const int kRxPin = 17;
    static const int kTxPin = 2;
    static const uint32_t kGpsBaud = 9600;
    TransmitterSensorData transmitter_sensor_data_;

    Adafruit_MPU6050 mpu_;
    DHT dht_(dht11_pin_, DHT11);
    TinyGPSPlus gps_;
};
