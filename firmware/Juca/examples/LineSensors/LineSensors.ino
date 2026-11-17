/*
 * Juca - LineSensors
 *
 * Reads the five infrared line sensors and prints where the line is, twice per second.
 * Nothing moves: slide the robot sideways over the line by hand and watch the Serial Monitor
 * (115200 baud).
 *
 * The bar is drawn as the robot sees it, far left sensor first: '#' = over the line, '.' = floor.
 *
 * Do this check before running a line follower:
 *   1. All sensors over the plain floor must print [.....]. If it prints [#####], the polarity is
 *      inverted: call Juca.beginLine(1) instead of Juca.beginLine().
 *   2. A strip of line under the far LEFT sensor must light the FIRST character.
 *   3. Each sensor must switch cleanly when the line passes under it (adjust its trimmer if not).
 */

#include <Juca.h>

void setup()
{
  Serial.begin(115200);
  delay(2000);   // time to open the Serial Monitor

  if (!Juca.beginLine())
  {
    Serial.printf("Line sensors failed to start: %s\n", esp_err_to_name(Juca.lastError()));
    while (true) { delay(1000); }
  }
}

void loop()
{
  uint8_t mask = Juca.readLineMask();

  char bar[IR_LINE_SENSOR_COUNT + 1];
  for (int i = 0; i < IR_LINE_SENSOR_COUNT; i++)
  {
    bar[i] = (mask & (1U << i)) ? '#' : '.';
  }
  bar[IR_LINE_SENSOR_COUNT] = '\0';

  float position;
  if (Juca.readLinePosition(position))
  {
    // -2 = far left, 0 = centered, +2 = far right
    Serial.printf("[%s] position %+.1f\n", bar, position);
  }
  else
  {
    Serial.printf("[%s] no line\n", bar);
  }

  delay(500);
}
