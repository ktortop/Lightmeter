#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    while (!Serial);
    if (!bno.begin()) {
      Serial.println("Failed to initialize IMU");
      while (1);
    }
    delay(1000);
      // Optional: reset sensor
    bno.setExtCrystalUse(true);
  }

}

void loop() {
  // reads the linear acceleration
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO085::VECTOR_LINEARACCEL);
  //individual acceleration componenets
  float ax = linAccel.x(); 
  float ay = linAccel.y();
  float az = linAccel.z();
  float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
  // reads the heading 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float heading = euler.x(); // yaw in degrees (0 degrees is north, 90 degrees east)
  // reads the gyro
    // ---- 3️⃣ Gyroscope ----
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //individual gyro components
  float gx = gyro.x();
  float gy = gyro.y();
  float gz = gyro.z();

  //print data
  Serial.print("Linear Accel (m/s^2): ");
  Serial.print(ax, 2); Serial.print(", ");
  Serial.print(ay, 2); Serial.print(", ");
  Serial.print(az, 2); Serial.print(" | Mag: ");
  Serial.println(accelMagnitude, 2);

  Serial.print("Heading (yaw): ");
  Serial.println(heading, 2);

  Serial.print("Gyro (deg/s): ");
  Serial.print(gx, 2); Serial.print(", ");
  Serial.print(gy, 2); Serial.print(", ");
  Serial.println(gz, 2);
  delay(100); // 10 Hz
}
