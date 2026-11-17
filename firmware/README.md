# Juca firmware

Software that runs on **Juca**, a small two-wheeled, differential-drive robot built on an **ESP32-S3**. This folder holds two versions of it, for the same robot and with the same drivers:

| Folder | What it is | Start here |
|---|---|---|
| [`juca_firmware/`](juca_firmware) | **ESP-IDF project.** The reference implementation: the drivers as ESP-IDF components and one example FreeRTOS task per subsystem | [`juca_firmware/README.md`](juca_firmware/README.md) |
| [`arduino/`](arduino) | **Arduino library** `Juca`, ported from the ESP-IDF project, with an Arduino-style API and seven example sketches | [`arduino/README.md`](arduino/README.md) |

```
firmware/
├── README.md                this file
├── juca_firmware/           ESP-IDF project
│   ├── main/                app_main(), board and platform headers, example tasks
│   ├── components/          wheel, diff_drive, threeeyes, ir_line, mpu6050
│   └── docs/                measurements and reports
└── arduino/
    ├── README.md            how the Arduino version is organised and kept in step
    └── Juca/                the Arduino library (library.properties, src/, examples/)
```

## Which one to use

| | ESP-IDF (`juca_firmware`) | Arduino (`arduino/Juca`) |
|---|---|---|
| Tooling | ESP-IDF v5.3 (v5.3.6 recommended), with VS Code or `idf.py` | Arduino IDE 2 with the esp32 core 3.x |
| Program structure | FreeRTOS tasks, chosen with `#define` in `main/main.c` | `setup()` and `loop()`, one sketch per example |
| API | Component functions: `diff_drive_set_twist()`, `threeeyes_trigger_and_wait()`, ... | The `Juca` object: `Juca.setTwist()`, `Juca.readSonarCm()`, ... (the component functions are also available) |
| Good for | Full control, real-time work, developing and testing the drivers | Classes, quick experiments, tools built on Arduino (for example block programming) |

Both control the same hardware in the same way: a velocity command (`linear_x` in m/s, `angular_z` in rad/s) goes in, the measured velocity comes out of the wheel odometry, and the same calibration, acceleration ramp and overcurrent protection run underneath.

## What the robot has

| Subsystem | Part | Driver |
|---|---|---|
| Controller | ESP32-S3 | |
| Motors | 2 DC gear motors on an L298N H-bridge, 25 kHz PWM | `wheel` |
| Encoders | Quadrature, 900 counts per wheel turn | `wheel` |
| Motor current | Shunt and amplifier on each motor, read by the ADC | `wheel` |
| Differential drive | Kinematics, odometry, calibration, ramp, overcurrent protection | `diff_drive` |
| Distance | 3 x HC-SR04 ultrasonic sensors, fired together | `threeeyes` |
| Line | 5 digital infrared sensors | `ir_line` |
| Orientation | MPU6050 IMU on I2C, with its on-chip DMP | `mpu6050` |

The pins are defined once per version, with the same values: [`juca_firmware/main/juca_board.h`](juca_firmware/main/juca_board.h) and [`arduino/Juca/src/juca_board.h`](arduino/Juca/src/juca_board.h). The chassis and motor parameters (wheel radius, track width, encoder resolution, duty cap) are in the `juca_platform.h` next to each of them. With other motors or another chassis, change `juca_platform.h` and calibrate again.

## Quick start

**ESP-IDF**, from `juca_firmware/`:

```
idf.py set-target esp32s3        # only the first time
idf.py build flash monitor
```

Choose the example task in `main/main.c`. Details in [`juca_firmware/README.md`](juca_firmware/README.md).

**Arduino:** install the esp32 core 3.x, install the `Juca` library from `arduino/Juca`, select *ESP32S3 Dev Module* and open *File > Examples > Juca*. Details in [`arduino/Juca/README.md`](arduino/Juca/README.md).

In both versions, **run the sensor examples first** (wheels with the robot lifted, line sensors, ultrasonic sensors, IMU) to check the wiring, and only then the ones that drive the robot. The examples that move the robot start with a calibration at full power: lift the wheels, or give the robot at least 2 m of free floor.

## How the two versions relate

The **ESP-IDF project is the reference.** The drivers are written and tested there first; the Arduino library carries copies of them in `arduino/Juca/src/components/`, with small adaptations to the Arduino environment (the IMU uses the `Wire` library, the current sensors use `analogRead()`, and the build options come from a header instead of `menuconfig`).

So a change to a driver is made in `juca_firmware/components/` and then copied to the Arduino library. The procedure, and the list of the adaptations file by file, are in [`arduino/README.md`](arduino/README.md#keeping-the-two-versions-in-step).

## Documentation

| Document | What it covers |
|---|---|
| [`juca_firmware/README.md`](juca_firmware/README.md) | The ESP-IDF project: hardware, architecture, components, getting started, configuration, conventions |
| [`juca_firmware/main/example_tasks/README.md`](juca_firmware/main/example_tasks/README.md) | The ESP-IDF example tasks and how to run them safely |
| `juca_firmware/components/*/README.md` | One README per driver (`diff_drive`, `mpu6050`, `threeeyes`, `ir_line`) |
| [`juca_firmware/docs/wheel_open_loop_tests.md`](juca_firmware/docs/wheel_open_loop_tests.md) | Measurements with the wheels in the air: speed curves, dead zone, electrical tests, current-sensor calibration |
| [`arduino/README.md`](arduino/README.md) | The Arduino version in the repository: setup, relation to ESP-IDF, keeping in step, making a zip |
| [`arduino/Juca/README.md`](arduino/Juca/README.md) | The Arduino library for its users: API, examples, pins, limitations |

## Known limitations

Shared by both versions, because they use the same drivers:

- **The drive is open loop.** The encoders feed the odometry and the start-up calibration, not the motors. At low speed the robot is slower than commanded, and below about 0.2 m/s the motors do not turn. Both versions include an example that closes the loop with a speed PID on top of the driver.
- **The calibration is not stored** and runs at every start, with the robot moving.
- **The overcurrent protection does not see a stall at low duty**, because the current sensors read the PWM-chopped average.
- **The ultrasonic sensors report the nearest object only**, and the IMU yaw drifts slowly (no magnetometer).
