#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long lastPublish = 0;
const int publishRate = 20;   // 50 Hz

void setup()
{
  Serial.begin(115200);

  Serial.println("Initializing BNO055...");

  if (!bno.begin())
  {
    Serial.println("BNO055 NOT DETECTED");

    while (1)
    {
      delay(100);
    }
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  Serial.println("BNO055 READY");
}

void loop()
{
  if (millis() - lastPublish >= publishRate)
  {
    sensors_event_t orientationData;
    sensors_event_t gyroData;
    sensors_event_t accelData;

    bno.getEvent(
      &orientationData,
      Adafruit_BNO055::VECTOR_EULER);

    bno.getEvent(
      &gyroData,
      Adafruit_BNO055::VECTOR_GYROSCOPE);

    bno.getEvent(
      &accelData,
      Adafruit_BNO055::VECTOR_ACCELEROMETER);

    float yaw =
      orientationData.orientation.x;

    float pitch =
      orientationData.orientation.y;

    float roll =
      orientationData.orientation.z;

    float gx =
      gyroData.gyro.x;

    float gy =
      gyroData.gyro.y;

    float gz =
      gyroData.gyro.z;

    float ax =
      accelData.acceleration.x;

    float ay =
      accelData.acceleration.y;

    float az =
      accelData.acceleration.z;

    Serial.print("I,");

    Serial.print(yaw, 4);
    Serial.print(",");

    Serial.print(pitch, 4);
    Serial.print(",");

    Serial.print(roll, 4);
    Serial.print(",");

    Serial.print(gx, 4);
    Serial.print(",");

    Serial.print(gy, 4);
    Serial.print(",");

    Serial.print(gz, 4);
    Serial.print(",");

    Serial.print(ax, 4);
    Serial.print(",");

    Serial.print(ay, 4);
    Serial.print(",");

    Serial.println(az, 4);

    lastPublish = millis();
  }
}