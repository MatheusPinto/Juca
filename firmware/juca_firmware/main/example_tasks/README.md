# Test tasks

Each file in this folder is a small, self-contained **example task** for the Juca robot (ESP32-S3, ESP-IDF v5.3.1). A task exercises **one piece of hardware** or **one component** from [`components/`](../../components), logs what it measures on the serial monitor, and nothing more.

They exist for two reasons:

- **Bring-up and bench tests.** After assembling or rewiring the robot, run the task of the part you touched and check that it behaves.
- **Reference code.** Each task shows the intended way to use its component: how to configure it, how to call it, and how to read the results.

These are **not** the final robot application. They are meant to be read, run, modified and thrown away.

## Contents

Each task has its own README in this folder, with what it does, what to expect in the log and how to interpret it. Start there after choosing a task.

| Task | Source | Enable with (in `main.c`) | Hardware | Component | Moves the robot? | Task README |
|---|---|---|---|---|---|---|
| Differential drive | [`diff_drive_task.c`](diff_drive_task.c) | `DIFF_DRIVE_TASK` | Motors, encoders, current sensors | [`diff_drive`](../../components/diff_drive) | **Yes** | [README_diff_drive.md](README_diff_drive.md) |
| Wheel | [`wheel_task.c`](wheel_task.c) | `WHEEL_CTRL_TASK` | Motors, encoders, current sensors | [`wheel`](../../components/wheel) | **Yes** | [README_wheel.md](README_wheel.md) |
| IMU | [`imu_task.c`](imu_task.c) | `IMU_TASK` | MPU6050 | [`mpu6050`](../../components/mpu6050) | No | [README_imu.md](README_imu.md) |
| Ultrasonic sensors | [`threeeyes_task.c`](threeeyes_task.c) | `THREE_EYES_TASK` | 3 x HC-SR04 | [`threeeyes`](../../components/threeeyes) | No | [README_threeeyes.md](README_threeeyes.md) |
| Line follower | [`ir_line_task.c`](ir_line_task.c) | `IR_LINE_TASK` | 5 infrared sensors | None (see [`ir_line.h`](../ir_line.h)) | No | [README_ir_line.md](README_ir_line.md) |

Pins of the motors, encoders, current sensors and IMU are defined **once**, in [`juca_board.h`](../juca_board.h). The pins of the ultrasonic sensors are in [`threeeyes.h`](../../components/threeeyes/include/threeeyes.h) and those of the line follower in [`ir_line.h`](../ir_line.h).

## How to use

### 1. Choose the task

Open [`main/main.c`](../main.c). Near the top there is one `#define` per task, and only the ones **without** `//` in front of them are started by `app_main()`:

```c
//#define THREE_EYES_TASK
#define IMU_TASK              // <- enabled
//#define WHEEL_CTRL_TASK
//#define IR_LINE_TASK
//#define DIFF_DRIVE_TASK
```

To run the differential drive example, for instance, comment out `IMU_TASK` and remove the `//` from `DIFF_DRIVE_TASK`.

Rules for combining tasks:

- **`WHEEL_CTRL_TASK` and `DIFF_DRIVE_TASK` must never be enabled together.** Both take the same motors, encoders, MCPWM group and ADC unit, so the second one to start would fail.
- The other tasks use different pins (there is no GPIO shared between any two of them), so they do not conflict on the board. Their log lines will be interleaved, though, so **enable one task at a time** when you want to read the output.

### 2. Choose the wheel variant (only for `WHEEL_CTRL_TASK`)

[`wheel_task.h`](wheel_task.h) offers three mutually exclusive variants (`WHEEL_EXAMPLE_TWO_WHEELS`, `WHEEL_EXAMPLE_SINGLE_BASIC` and `WHEEL_EXAMPLE_ENCODER_TEST`). Leave **exactly one** `#define` active: with none, `wheel_ctrl_task` does not exist and the link fails; with two, it is defined twice. The variants are described in [README_wheel.md](README_wheel.md).

### 3. Build, flash and monitor

From the project root, with ESP-IDF v5.3.1 loaded in the shell:

```
idf.py set-target esp32s3        # only the first time
idf.py build flash monitor
```

If the board is not found automatically, add the serial port: `idf.py -p <PORT> flash monitor`.

### 4. Before running a task that moves the robot

`wheel_task` spins the motors immediately. `diff_drive_task` does it after a 2 s pause and a warning in the log (the calibration), and again with every command you type.

- Put the robot **on a stand with the wheels in the air**, or on the floor with **at least 2 m of free path** in front of it.
- Make sure the **motor supply** (battery) is on, otherwise the wheels will not turn.
- The first time, be ready to cut the motor power.

## Adding a new task

1. Create `<name>_task.c` and `<name>_task.h` in this folder. The header declares the entry function with `portTASK_FUNCTION(<name>_task, arg);`.
2. Take pins and physical constants from [`juca_board.h`](../juca_board.h); do not hard-code GPIOs in the task.
3. Add the `.c` file to `SRCS` in [`main/CMakeLists.txt`](../CMakeLists.txt) and, if the task needs a new component, to `REQUIRES`.
4. In [`main.c`](../main.c), include the header, add a `//#define <NAME>_TASK` line and an `#ifdef` block that calls `xTaskCreate()`, like the existing ones.
5. Create `README_<name>.md` in this folder and add a row to the table above.