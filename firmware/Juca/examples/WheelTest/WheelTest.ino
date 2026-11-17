/*
 * Juca - WheelTest
 *
 * Bring-up test of the two wheels, WITHOUT the drive layer: spins both wheels forward and backward
 * at half power and prints the encoder counts and the raw current readings. Serial Monitor at
 * 115200 baud.
 *
 * THE WHEELS SPIN AS SOON AS THE SKETCH STARTS: put the robot on a stand, wheels in the air.
 *
 * What to check:
 *   - Both wheels turn FORWARD when the log says "forward". If one goes backward, swap that
 *     motor's two PWM pins (or its two wires).
 *   - Both encoder counts go UP while going forward. If one goes down, swap that encoder's A and B.
 *   - The current readings are above zero while the wheels turn.
 */

#include <Juca.h>

const int32_t TEST_POWER_TICKS = 200;       // half of the 400 ticks full scale
const uint32_t DIRECTION_PERIOD_MS = 3000;  // time in each direction
const uint32_t PRINT_PERIOD_MS = 500;

void setup()
{
  Serial.begin(115200);
  delay(2000);   // time to open the Serial Monitor

  if (!Juca.beginWheels())
  {
    Serial.printf("Wheels failed to start: %s\n", esp_err_to_name(Juca.lastError()));
    while (true) { delay(1000); }
  }
  Serial.println("Wheels ready. Keep the wheels in the air!");
}

void loop()
{
  static bool forward = false;
  static uint32_t lastDirectionMs = 0;
  static uint32_t lastPrintMs = 0;
  uint32_t nowMs = millis();

  // Change direction every DIRECTION_PERIOD_MS (and start the first time through).
  if (lastDirectionMs == 0 || nowMs - lastDirectionMs >= DIRECTION_PERIOD_MS)
  {
    lastDirectionMs = nowMs;
    forward = !forward;
    int32_t power = forward ? TEST_POWER_TICKS : -TEST_POWER_TICKS;
    wheel_set_power(Juca.leftWheel(), power);
    wheel_set_power(Juca.rightWheel(), power);
    Serial.println(forward ? "Going forward" : "Going backward");
  }

  if (nowMs - lastPrintMs >= PRINT_PERIOD_MS)
  {
    lastPrintMs = nowMs;
    int leftCount = 0, rightCount = 0, leftCurrent = 0, rightCurrent = 0;
    wheel_get_encoder_count(Juca.leftWheel(), &leftCount);
    wheel_get_encoder_count(Juca.rightWheel(), &rightCount);
    wheel_get_current_raw(Juca.leftWheel(), &leftCurrent);
    wheel_get_current_raw(Juca.rightWheel(), &rightCurrent);
    Serial.printf("Left: encoder=%d current=%d | Right: encoder=%d current=%d\n",
                  leftCount, leftCurrent, rightCount, rightCurrent);
  }

  delay(10);
}
