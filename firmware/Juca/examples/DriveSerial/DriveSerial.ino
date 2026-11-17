/*
 * Juca - DriveSerial
 *
 * Drives the robot from the Serial Monitor (115200 baud, line ending "New Line") and prints the
 * velocity the robot REALLY has, measured by the wheel encoders.
 *
 * A command is two numbers and ENTER:  <linear m/s> <angular rad/s>
 *     0.3 0     forward at 0.3 m/s
 *    -0.3 0     backward
 *     0 2       spin in place to the left (counter-clockwise) at 2 rad/s
 *     0.3 1     forward while turning left
 *     0 0       stop
 * Anything that is not two numbers stops the robot.
 *
 * THE ROBOT MOVES BY ITSELF: 3 s after start-up it runs a 4 s calibration at full power, and then
 * it follows every command. Put it on a stand with the wheels in the air, or give it 2 m of free
 * floor. During the calibration the overcurrent protection is paused.
 *
 * This example is OPEN LOOP: the library turns a speed into motor power and does not correct it.
 * Compare the "measured" line with what you typed: at low speed the robot goes slower than asked,
 * and below about 0.2 m/s the motors do not move at all. The DriveSpeedPid example fixes that.
 */

#include <Juca.h>

const uint32_t LOOP_PERIOD_MS = 50;   // odometry every 50 ms (20 Hz)
const int PRINT_EVERY_LAPS = 5;       // print one line every 5 laps (250 ms)

bool faultReported = false;

void setup()
{
  Serial.begin(115200);
  delay(2000);   // time to open the Serial Monitor

  // Wheels + differential drive, with the pins and the geometry of the Juca. Nothing moves yet.
  if (!Juca.beginDrive())
  {
    Serial.printf("Drive failed to start: %s\n", esp_err_to_name(Juca.lastError()));
    while (true) { delay(1000); }
  }

  Serial.println("Calibration in 3 s: THE ROBOT WILL MOVE for 4 s. Clear its path!");
  delay(3000);

  // Both wheels at the same power; the encoders measure the real top speed and the left/right
  // difference. The result is kept in RAM only, so it is repeated at every start.
  float maxWheelRadS = 0.0f;
  if (Juca.calibrate(4000, &maxWheelRadS))
  {
    Serial.printf("Calibrated: the slower wheel does %.2f rad/s at full power.\n", maxWheelRadS);
  }
  else
  {
    Serial.printf("Calibration failed (%s). Going on with the default estimate.\n", esp_err_to_name(Juca.lastError()));
  }

  Serial.println("Type: <linear m/s> <angular rad/s> and ENTER, for example: 0.3 0");
}

void loop()
{
  static uint32_t lastLoopMs = 0;
  static int lap = 0;

  // ---- 1. A new command from the Serial Monitor? ----
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      float linear = 0.0f;
      float angular = 0.0f;
      if (sscanf(line.c_str(), "%f %f", &linear, &angular) != 2 || !isfinite(linear) || !isfinite(angular))
      {
        // Do not guess what was meant: stop.
        linear = 0.0f;
        angular = 0.0f;
        Serial.printf("Invalid input '%s': stopping.\n", line.c_str());
      }

      // After an overcurrent fault, a typed command means "I checked the robot, go on".
      if (Juca.faultActive())
      {
        Juca.clearFault();
        faultReported = false;
        Serial.println("Fault cleared.");
      }

      if (Juca.setTwist(linear, angular))
      {
        Serial.printf("Command: %.2f m/s, %.2f rad/s\n", linear, angular);
      }
      else
      {
        Serial.printf("Command refused: %s\n", esp_err_to_name(Juca.lastError()));
      }
    }
  }

  // ---- 2. Overcurrent protection (a blocked wheel): report it once ----
  if (Juca.faultActive() && !faultReported)
  {
    faultReported = true;
    Serial.println("OVERCURRENT: the robot is stopped. Free it, then type a new command.");
  }

  // ---- 3. Odometry, every LOOP_PERIOD_MS ----
  uint32_t nowMs = millis();
  if (nowMs - lastLoopMs >= LOOP_PERIOD_MS)
  {
    lastLoopMs = nowMs;

    float measuredLinear = 0.0f;
    float measuredAngular = 0.0f;
    if (Juca.getTwist(measuredLinear, measuredAngular) && ++lap >= PRINT_EVERY_LAPS)
    {
      lap = 0;
      Serial.printf("Measured: %.3f m/s, %.3f rad/s\n", measuredLinear, measuredAngular);
    }
  }
}
