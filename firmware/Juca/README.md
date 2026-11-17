# Juca for Arduino

This folder holds the **Arduino version** of the Juca firmware: a library, `Juca`, that runs the robot from the Arduino IDE on the same ESP32-S3 board. It is a port of the ESP-IDF project in [`../juca_firmware`](../juca_firmware): the same drivers, with a small Arduino-style class on top and seven example sketches.

- How to **use** the library (API, examples, pins, limitations): [`Juca/README.md`](Juca/README.md).
- This page: how the Arduino version is organised in the repository, how to work on it, and how to keep it in step with the ESP-IDF project.

```
firmware/
├── juca_firmware/            ESP-IDF project (the reference)
└── arduino/
    ├── README.md             this file
    └── Juca/                 the Arduino library
        ├── library.properties
        ├── keywords.txt
        ├── README.md         user documentation of the library
        ├── src/
        │   ├── Juca.h, Juca.cpp          the Juca object (Arduino-style API)
        │   ├── juca_board.h              pins of the board
        │   ├── juca_platform.h           chassis and motor parameters
        │   └── components/               drivers, ported from ../juca_firmware/components
        └── examples/                     WheelTest, LineSensors, Sonar, Imu,
                                          DriveSerial, DriveSpeedPid, LineAvoid
```

## ESP-IDF or Arduino?

Both run on the same robot and use the same drivers. Pick by who is going to program it:

| | ESP-IDF (`juca_firmware`) | Arduino (`arduino/Juca`) |
|---|---|---|
| Tooling | ESP-IDF v5.3 and VS Code (or `idf.py`) | Arduino IDE 2 with the esp32 core 3.x |
| Program structure | FreeRTOS tasks, one per example, chosen with `#define` in `main.c` | `setup()` and `loop()`, one sketch per example |
| API | Component functions (`diff_drive_set_twist()`, ...) | The `Juca` object (`Juca.setTwist()`, ...), plus the same component functions |
| Build options | `idf.py menuconfig` | `src/components/juca_config.h` |
| Good for | Full control, real-time work, the reference implementation | Classes, quick experiments, block-programming tools built on Arduino |

## Setting up a development copy

### 1. Board package

In the Arduino IDE 2: *Boards Manager*, install **esp32 by Espressif Systems**, version **3.x**. The library does not build with the 2.x core.

Board settings under *Tools*:

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | **Enabled** when the cable is on the native USB port of the ESP32-S3 (the same port the ESP-IDF console uses). Disabled if it is on a USB-to-UART port |
| USB Mode | Hardware CDC and JTAG |
| Core Debug Level | None (set Info to see the messages of the drivers) |

### 2. Install the library

Either way works:

- **From a zip:** make a zip of the `Juca` folder (or use one from a release, see [Making a zip for others](#making-a-zip-for-others)) and add it with *Sketch > Include Library > Add .ZIP Library*.
- **By copying the folder:** copy `arduino/Juca` into the `libraries` folder of your sketchbook. The sketchbook is the *Sketchbook location* of *File > Preferences*.

Restart the IDE. The examples appear under *File > Examples > Juca*.

### 3. First run

1. Upload *WheelTest* with the wheels in the air. Check that both wheels turn forward on "Going forward" and that both encoder counts go up.
2. Then *LineSensors*, *Sonar* and *Imu*, which do not move the robot.
3. Only then the sketches that drive it: *DriveSerial*, *DriveSpeedPid*, *LineAvoid*.

Serial Monitor: **115200 baud**, line ending **New Line**.

The examples in the library folder are part of the repository. To try changes without touching them, use *File > Save As* to save a copy of the sketch in your own sketchbook.

### Suggestion: work on the library straight from your clone

Not needed to use the library. It helps if you are going to **change** it: with the zip or a copied folder, the IDE uses a copy, and every edit in the repository means installing it again. A link from the `libraries` folder to your clone avoids that: the IDE then reads the files of the repository directly.

Remove any `Juca` folder already in the `libraries` folder first, then create the link, replacing the two placeholders with your own paths.

Windows (*Command Prompt*):

```
mklink /J "<sketchbook>\libraries\Juca" "<clone>\firmware\arduino\Juca"
```

Linux or macOS:

```
ln -s "<clone>/firmware/arduino/Juca" "<sketchbook>/libraries/Juca"
```

`<sketchbook>` is the *Sketchbook location* of *File > Preferences*, and `<clone>` is the folder of your copy of this repository. Restart the IDE afterwards.

## How the port relates to the ESP-IDF project

The files in `Juca/src/components/` are **copies** of the components of `juca_firmware`, so the two versions behave the same. Most are unchanged; a few needed small adaptations to the Arduino environment:

| Arduino file | Comes from | Differences |
|---|---|---|
| `diff_drive.c`, `diff_drive.h` | `components/diff_drive/` | `#include "juca_port.h"`; `TAG` marked `__attribute__((unused))` |
| `threeeyes.c`, `threeeyes.h` | `components/threeeyes/` | Same two lines |
| `ir_line.c`, `ir_line.h` | `components/ir_line/` | Same two lines |
| `mpu6050init.c`, `mpu6050init.h` | `components/mpu6050/` | Same two lines, plus `#include "esp_log.h"` |
| `mpu6050_dmp_firmware.h` | `components/mpu6050/include/` | None |
| `mpu6050.c`, `mpu6050.h` | `components/mpu6050/` | The I2C layer goes through the Arduino `Wire` library (`mpu6050_port_*` functions, in `mpu6050_wire.cpp`) instead of the ESP-IDF I2C driver: the two cannot be mixed in one program. `mpu6050_init()` no longer installs a driver |
| `wheel.c`, `wheel.h` | `components/wheel/` | The current sensor is read with `analogRead()`; `wheel_config_t` has `current_sense_gpio` instead of `adc_handle` and `adc_channel`. The `CONFIG_WHEEL_*` options come from `juca_config.h` |
| `bdc_motor*.c`, `bdc_motor*.h`, `bdc_motor_LICENSE.txt` | `managed_components/espressif__bdc_motor/` (0.1.0) | None. Copied because Arduino has no component manager. Apache-2.0, Espressif |

Files that exist only in the Arduino version:

| File | What it is |
|---|---|
| `src/Juca.h`, `src/Juca.cpp` | The `Juca` object: each subsystem set up with the pins of the board, and an Arduino-style API |
| `src/juca_board.h` | Same pins as `juca_firmware/main/juca_board.h`, except that the current sensors are pins (`JUCA_PIN_CURRENT_*`) instead of ADC channels, and `JUCA_MCPWM_GROUP_MOTORS` is added |
| `src/juca_platform.h` | Same as `juca_firmware/main/juca_platform.h` |
| `src/components/juca_config.h` | The `CONFIG_WHEEL_*` options of the wheel Kconfig menu, with the same defaults |
| `src/components/juca_port.h` | Includes the Arduino HAL, so the C sources get `analogRead()` and the Arduino log macros |
| `src/components/mpu6050_wire.cpp` | The I2C transport of the MPU6050 over `Wire` |

Under the Arduino core 3.3.12 the drivers run on ESP-IDF v5.5.5, which already contains the fix of the pulse-counter read-out race of ESP-IDF v5.3.1 to v5.3.3.

## Keeping the two versions in step

The ESP-IDF project is the reference. When a component changes there:

1. Copy the changed file to `Juca/src/components/` (headers from `include/` go to the same flat folder).
2. Re-apply the differences of the table above for that file. For most files that is only the `#include "juca_port.h"` line and the `__attribute__((unused))` on `TAG`.
3. If the public API changed, update `src/Juca.h`, `src/Juca.cpp`, the examples and `Juca/README.md`.
4. Compile all the examples (next section).

To see what differs between the two copies of a file, from the `firmware` folder:

```
git diff --no-index --ignore-cr-at-eol juca_firmware/components/diff_drive/diff_drive.c arduino/Juca/src/components/diff_drive.c
```

`--ignore-cr-at-eol` is needed because the ESP-IDF sources use CRLF line endings and the Arduino copies use LF. Only the adaptations listed above should show up.

When a component of the ESP-IDF project gets a new pin or a new platform constant, update `src/juca_board.h` or `src/juca_platform.h` here too.

## Checking that everything builds

In the IDE, open each example and press *Verify*. With [arduino-cli](https://arduino.github.io/arduino-cli/), from the `firmware/arduino` folder, all the examples at once:

```
arduino-cli compile --fqbn "espressif:esp32:esp32s3:CDCOnBoot=cdc" --warnings all Juca/examples/WheelTest
```

and the same for `LineSensors`, `Sonar`, `Imu`, `DriveSerial`, `DriveSpeedPid` and `LineAvoid`. All seven compile without warnings with the esp32 core 3.3.12.

Also compile `DriveSpeedPid` once with `PLOT_OUTPUT 1` and `PLOT_ANGULAR 1`, the code behind those switches is not built otherwise.

## Making a zip for others

The library is distributed as a zip that users install with *Sketch > Include Library > Add .ZIP Library*. The zip is a build product: do not commit it. Create it from the repository when needed, with the `Juca` folder at its top level. From the root of the repository:

```
git archive --format=zip --prefix=Juca/ -o Juca.zip HEAD:firmware/arduino/Juca
```

This packs the last commit, so commit your changes first. Bump `version=` in `Juca/library.properties` for each zip you hand out, and attach the zip to a release (for example a GitHub Release) instead of committing it.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `The Juca library needs the Arduino-ESP32 core version 3.0 or newer` | The esp32 core 2.x is installed: update it in the Boards Manager |
| Nothing at all in the Serial Monitor | Press the RST button after the upload (with the native USB the board can stay in download mode). Check *USB CDC On Boot* against the port the cable is on, and *Tools > Port* (the number can change after a reset) |
| The IDE shows an old version of an example | An older copy of the library is installed: remove it and install the current one (or use the link of the suggestion above) |
| `multiple definition` or duplicate `Juca` library warnings | Two copies of the library in the libraries folder |
| The Serial Plotter shows nothing | The sketch prints text. In `DriveSpeedPid` set `PLOT_OUTPUT` to 1 |

## Status

The library and the examples compile without warnings for the ESP32S3 Dev Module with the esp32 core 3.3.12. Communication with the robot was checked; the behaviour of the examples on the robot is being validated (the PID gains of `DriveSpeedPid` and the numbers of `LineAvoid` are still starting values).
