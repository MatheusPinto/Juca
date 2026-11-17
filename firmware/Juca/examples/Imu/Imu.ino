/*
 * Juca - Imu
 *
 * Starts the MPU6050 inertial sensor, calibrates it and prints the orientation of the robot as a
 * quaternion, about 10 times per second. Nothing moves. Serial Monitor at 115200 baud.
 *
 * IMPORTANT: put the robot on a flat, level surface and do not touch it until "IMU ready" appears.
 * The calibration assumes the sensor is still and level.
 *
 * Reading the numbers: w close to 1 and x, y, z close to 0 means "same orientation as during the
 * calibration". Turn the robot 90 degrees flat on the table: w and z both go to about 0.71.
 * z drifts slowly over time: this sensor has no compass.
 */

#include <Juca.h>

void setup()
{
  Serial.begin(115200);
  delay(2000);   // time to open the Serial Monitor

  Serial.println("Calibrating the IMU: keep the robot still and level...");
  if (!Juca.beginImu())
  {
    // ESP_FAIL = the sensor does not answer (wiring, power). ESP_ERR_TIMEOUT = the robot moved.
    Serial.printf("IMU failed to start: %s\n", esp_err_to_name(Juca.lastError()));
    while (true) { delay(1000); }
  }
  Serial.println("IMU ready.");
}

void loop()
{
  mpu6050_quaternion_t q;

  if (Juca.readQuaternion(q))
  {
    Serial.printf("Quat [w=%.3f x=%.3f y=%.3f z=%.3f]\n", q.w, q.x, q.y, q.z);
  }
  else if (Juca.lastError() != ESP_ERR_NOT_FOUND)
  {
    // ESP_ERR_NOT_FOUND only means "nothing new yet". Anything else is a real problem.
    Serial.printf("IMU read failed: %s\n", esp_err_to_name(Juca.lastError()));
  }

  delay(40);   // ask more often than the sensor produces (every 100 ms), so nothing is missed
}
