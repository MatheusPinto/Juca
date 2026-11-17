/*
 * Juca - DriveSpeedPid
 *
 * Same as DriveSerial, with a SPEED PID on top: every 50 ms the sketch compares the speed you asked
 * for with the speed the encoders measured, and corrects the command it sends to the library.
 *
 *     your command -> limit to reachable speed -> acceleration ramp -> reference --+--> (+) --> Juca.setTwist()
 *                                                      |     ^
 *                                    reference - measured    | correction
 *                                                      +-> PID
 *                              measured  <--  Juca.getTwist()
 *
 * What changes compared with DriveSerial:
 *   - Low speeds work: "0.2 0" moves the robot (the integral part overcomes the motor dead zone).
 *   - The measured speed settles on the command, instead of below it.
 *   - Straight lines stay straight: the angular PID corrects a left/right difference.
 *
 * A command above what the wheels can reach (for example "50 0") is reduced to the top speed
 * before it reaches the PID, keeping its direction; the sketch prints a warning when it does.
 *
 * Commands (Serial Monitor, 115200 baud, "New Line"):  <linear m/s> <angular rad/s>, e.g. "0.3 0".
 *
 * THE ROBOT MOVES BY ITSELF (calibration 3 s after start-up, then every command).
 *
 * THE GAINS BELOW ARE STARTING VALUES FROM A SIMPLE SIMULATION. THEY WERE NOT TUNED ON THE ROBOT.
 * Test first with the wheels in the air. To tune one axis: set KI to 0 and raise KP until the speed
 * oscillates, use half of that; then raise KI until the speed reaches the command in about 1 s.
 *
 * The encoders must count UP when the robot goes forward (check with the WheelTest example). With
 * one encoder inverted the correction has the wrong sign and that wheel runs away.
 */

#include <Juca.h>

// ---- Gains: one controller for linear (m/s), one for angular (rad/s) ----
const float LINEAR_KP = 0.5f;     // correction per unit of error, right now
const float LINEAR_KI = 2.0f;     // correction per unit of error, per second (removes the steady error)
const float LINEAR_KD = 0.0f;     // 0: the encoder speed is too noisy at 20 Hz for a derivative
const float ANGULAR_KP = 0.5f;
const float ANGULAR_KI = 2.0f;
const float ANGULAR_KD = 0.0f;

// ---- Largest correction each controller may add to the command ----
const float LINEAR_LIMIT_M_S = 0.4f;       // must cover the dead zone (about 0.2 m/s)
const float ANGULAR_LIMIT_RAD_S = 2.5f;    // must cover the dead zone (about 1.3 rad/s)

// ---- Acceleration ramp of the reference ----
const float ACCEL_LINEAR_M_S2 = 0.5f;
const float ACCEL_ANGULAR_RAD_S2 = 1.5f;

const uint32_t LOOP_PERIOD_MS = 50;   // control period (20 Hz)
const int PRINT_EVERY_LAPS = 5;       // print one line every 5 laps (250 ms)

// One PID controller: gains, limit, and its memory between two steps.
struct SpeedPid
{
  float kp;
  float ki;
  float kd;
  float outputLimit;     // the output, and the integral part alone, stay within +/- this value
  float integral;        // accumulated integral part of the output
  float previousError;   // error of the previous step, for the derivative
};

SpeedPid linearPid = { LINEAR_KP, LINEAR_KI, LINEAR_KD, LINEAR_LIMIT_M_S, 0.0f, 0.0f };
SpeedPid angularPid = { ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, ANGULAR_LIMIT_RAD_S, 0.0f, 0.0f };

float commandLinear = 0.0f;      // what you typed, brought inside the reachable speeds
float commandAngular = 0.0f;
float reachableMS = 0.0f;        // fastest speed a wheel surface can reach, in m/s (set after the calibration)
float referenceLinear = 0.0f;    // the command after the acceleration ramp: what the PID follows
float referenceAngular = 0.0f;
bool faultReported = false;
uint32_t lastLoopMs = 0;           // time of the previous control step

// Forgets the past of a controller (when the robot stops, and after a fault).
void pidReset(SpeedPid &pid)
{
  pid.integral = 0.0f;
  pid.previousError = 0.0f;
}

// One step: from the error (wanted - measured) to the correction.
// The integral is limited ("anti-windup"): if the wheels cannot deliver what is asked, the error
// never reaches zero, and without the limit the integral would grow without end.
float pidStep(SpeedPid &pid, float error, float dtSeconds)
{
  pid.integral = constrain(pid.integral + pid.ki * error * dtSeconds, -pid.outputLimit, pid.outputLimit);
  float output = pid.kp * error + pid.integral + pid.kd * (error - pid.previousError) / dtSeconds;
  pid.previousError = error;
  return constrain(output, -pid.outputLimit, pid.outputLimit);
}

// Moves a value towards a target by at most maxStep.
float rampTowards(float current, float target, float maxStep)
{
  return current + constrain(target - current, -maxStep, maxStep);
}

// Brings a command inside the speeds the wheels can reach, keeping its direction (same curve, lower
// speed). The faster wheel of (linear, angular) turns at |linear| + |angular| * L/2 m/s.
// WHY: the library limits what it sends to the motors, but not the reference of this PID. Without
// this, "50 0" would make the reference ramp towards 50 m/s, and after "0 0" it would need as long
// to come back down: the robot would keep running at full speed for all that time.
// Returns true if the command was reduced.
bool limitToReachable(float &linear, float &angular)
{
  float demand = fabsf(linear) + fabsf(angular) * JUCA_HALF_TRACK_WIDTH_M;
  if (demand <= reachableMS)
  {
    return false;
  }
  float scale = reachableMS / demand;
  linear *= scale;
  angular *= scale;
  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);   // time to open the Serial Monitor

  // The acceleration ramp is done in this sketch (on the reference), so the ramp of the library is
  // turned off: two ramps in a row would delay the corrections and make the speed overshoot.
  diff_drive_config_t config = Juca.defaultDriveConfig();
  config.max_accel_linear_m_s2 = 0.0f;
  config.max_accel_angular_rad_s2 = 0.0f;
  if (!Juca.beginDrive(&config))
  {
    Serial.printf("Drive failed to start: %s\n", esp_err_to_name(Juca.lastError()));
    while (true) { delay(1000); }
  }

  Serial.println("Calibration in 3 s: THE ROBOT WILL MOVE for 4 s. Clear its path!");
  delay(3000);
  float maxWheelRadS = 0.0f;
  if (Juca.calibrate(4000, &maxWheelRadS))
  {
    Serial.printf("Calibrated: the slower wheel does %.2f rad/s at full power.\n", maxWheelRadS);
  }
  else
  {
    maxWheelRadS = config.max_wheel_rad_s;   // the estimate of the configuration
    Serial.printf("Calibration failed (%s). Going on with the default estimate.\n", esp_err_to_name(Juca.lastError()));
  }

  // Wheel speed at full duty x wheel radius x the fraction of the duty the motors may use.
  reachableMS = maxWheelRadS * JUCA_WHEEL_RADIUS_M * ((float)JUCA_MOTOR_MAX_POWER_TICKS / (float)WHEEL_PWM_DUTY_TICK_MAX);
  Serial.printf("Reachable: %.2f m/s straight, %.2f rad/s in place.\n", reachableMS, reachableMS / JUCA_HALF_TRACK_WIDTH_M);

  Serial.println("Type: <linear m/s> <angular rad/s> and ENTER, for example: 0.3 0");
  lastLoopMs = millis();
}

void loop()
{
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
        linear = 0.0f;
        angular = 0.0f;
        Serial.printf("Invalid input '%s': stopping.\n", line.c_str());
      }

      // After an overcurrent fault, a typed command means "I checked the robot, go on". Start again
      // from a stop: the reference ramps up from zero and the controllers forget the blocked wheel.
      if (Juca.faultActive())
      {
        Juca.clearFault();
        faultReported = false;
        referenceLinear = 0.0f;
        referenceAngular = 0.0f;
        pidReset(linearPid);
        pidReset(angularPid);
        Serial.println("Fault cleared.");
      }

      if (limitToReachable(linear, angular))
      {
        Serial.printf("Command above what the wheels can reach: limited to %.3f m/s, %.3f rad/s.\n", linear, angular);
      }
      commandLinear = linear;
      commandAngular = angular;
      Serial.printf("Command: %.2f m/s, %.2f rad/s\n", commandLinear, commandAngular);
    }
  }

  // ---- 2. Overcurrent protection: report it once, and do nothing else while it lasts ----
  if (Juca.faultActive())
  {
    if (!faultReported)
    {
      faultReported = true;
      Serial.println("OVERCURRENT: the robot is stopped. Free it, then type a new command.");
    }
    lastLoopMs = millis();   // so the first step after the fault does not see a huge time interval
    return;
  }

  // ---- 3. The control step, every LOOP_PERIOD_MS ----
  uint32_t nowMs = millis();
  if (nowMs - lastLoopMs < LOOP_PERIOD_MS)
  {
    return;
  }
  float dtSeconds = (float)(nowMs - lastLoopMs) / 1000.0f;
  lastLoopMs = nowMs;

  // 3a. How fast is the robot really going?
  float measuredLinear = 0.0f;
  float measuredAngular = 0.0f;
  if (!Juca.getTwist(measuredLinear, measuredAngular))
  {
    return;   // no odometry in this lap: keep the previous command
  }

  // 3b. The reference moves towards the command, limited by the acceleration.
  referenceLinear = rampTowards(referenceLinear, commandLinear, ACCEL_LINEAR_M_S2 * dtSeconds);
  referenceAngular = rampTowards(referenceAngular, commandAngular, ACCEL_ANGULAR_RAD_S2 * dtSeconds);

  float sentLinear = 0.0f;
  float sentAngular = 0.0f;
  if (referenceLinear == 0.0f && referenceAngular == 0.0f)
  {
    // 3c. Asked to stand still: send exactly zero (the library brakes the wheels) and clear the
    //     controllers, so encoder noise at rest cannot slowly wind them up.
    pidReset(linearPid);
    pidReset(angularPid);
  }
  else
  {
    // 3d. Send the reference (what open loop would send) PLUS the correction of each controller.
    sentLinear = referenceLinear + pidStep(linearPid, referenceLinear - measuredLinear, dtSeconds);
    sentAngular = referenceAngular + pidStep(angularPid, referenceAngular - measuredAngular, dtSeconds);
  }
  Juca.setTwist(sentLinear, sentAngular);

  // "Sent" above "reference" means the PID is pushing harder to make up for a slow robot.
  if (++lap >= PRINT_EVERY_LAPS)
  {
    lap = 0;
    Serial.printf("Reference: %.3f m/s %.3f rad/s | Measured: %.3f m/s %.3f rad/s | Sent: %.3f m/s %.3f rad/s\n",
                  referenceLinear, referenceAngular, measuredLinear, measuredAngular, sentLinear, sentAngular);
  }
}
