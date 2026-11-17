/*
 * Juca - Sonar
 *
 * Reads the three HC-SR04 ultrasonic sensors and prints the nearest object, twice per second.
 * Nothing moves. Move your hand in front of each sensor and watch the Serial Monitor (115200 baud).
 *
 * The three sensors fire together and the answer is the NEAREST object: its distance and which
 * sensor saw it. With nothing in front of the robot the sensors still answer, after about 40 ms,
 * and the library reports "nothing in range".
 */

#include <Juca.h>

const char *const SIDE_NAMES[] = { "left", "middle", "right" };

void setup()
{
  Serial.begin(115200);
  delay(2000);   // time to open the Serial Monitor

  if (!Juca.beginSonar())
  {
    Serial.printf("Sonar failed to start: %s\n", esp_err_to_name(Juca.lastError()));
    while (true) { delay(1000); }
  }
  Serial.println("Sonar ready.");
}

void loop()
{
  JucaSonarSide side;
  float distanceCm = Juca.readSonarCm(&side);

  if (distanceCm < 0)
  {
    // Either nothing within 4 m, or no sensor answered at all (check wiring and the 5 V supply).
    if (Juca.lastError() == ESP_ERR_TIMEOUT)
    {
      Serial.println("No echo: check the wiring and the 5 V supply of the sensors.");
    }
    else
    {
      Serial.println("Nothing in range.");
    }
  }
  else
  {
    Serial.printf("Nearest object: %s sensor, %.1f cm\n", SIDE_NAMES[side], distanceCm);
  }

  delay(500);   // do not go below 60 ms: the echoes of one trigger must die out before the next
}
