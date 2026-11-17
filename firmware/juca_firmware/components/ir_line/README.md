# ir_line component

Reader for a **bar of 5 digital infrared line sensors** on ESP-IDF. It reads the five GPIOs and returns one bit per sensor, already corrected for the polarity: 1 means "this sensor is over the line".

Developed for ESP-IDF v5.3 on an ESP32-S3 (the Juca robot). It has no dependency on other components of this repository.

**Contents**

1. [What it does (and does not)](#1-what-it-does-and-does-not)
2. [Quick start](#2-quick-start)
3. [Wiring](#3-wiring)
4. [Configuration reference](#4-configuration-reference)
5. [API reference](#5-api-reference)
6. [The mask and the patterns](#6-the-mask-and-the-patterns)
7. [From the mask to a line position](#7-from-the-mask-to-a-line-position)
8. [Error codes](#8-error-codes)
9. [Limitations and gotchas](#9-limitations-and-gotchas)
10. [Files](#10-files)

---

## 1. What it does (and does not)

| It does | It does not |
|---|---|
| Configure five GPIOs as inputs | Read analog sensors: the modules must have a digital output |
| Return one bit per sensor, in a fixed order (bit 0 = far left) | Compute a line position or a steering command (see [7](#7-from-the-mask-to-a-line-position)) |
| Hide the polarity: the application always sees 1 = line | Filter or debounce: each call is one instantaneous reading |
| Name the patterns a single line produces (`IR_LINE_PATTERN_*`) | Support more than one bar, or a different number of sensors, without editing `IR_LINE_SENSOR_COUNT` |

The component is a **singleton** and knows nothing about the board: pins and polarity arrive in `ir_line_config_t`. Component dependencies (`CMakeLists.txt`): `esp_driver_gpio`, `log`.

---

## 2. Quick start

```c
#include "ir_line.h"
#include "esp_log.h"

static const char *TAG = "app";

void line_demo(void)
{
    /* Pins from the far LEFT sensor to the far RIGHT one. These are the Juca robot's. */
    const ir_line_config_t config = {
        .sensor_gpios = { GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_8 },
        .line_level = 0,    /* these sensors output 0 over a black line */
    };
    ESP_ERROR_CHECK(ir_line_init(&config));

    for (;;) {
        uint8_t mask = 0;
        ir_line_read_mask(&mask);

        if (mask == IR_LINE_PATTERN_MIDDLE) {
            ESP_LOGI(TAG, "line centered");
        } else if (mask == IR_LINE_PATTERN_NONE) {
            ESP_LOGI(TAG, "no line");
        } else {
            ESP_LOGI(TAG, "mask = 0x%02x", mask);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

A complete, runnable version is the example task `ir_line_task.c`, described in `README_ir_line.md` next to it (in `main/test_tasks/` or `main/example_tasks/`, depending on your tree).

---

## 3. Wiring

Each sensor module has three pins: supply, ground and a digital output. The output goes to one GPIO input. The component enables no pull-up and no pull-down, because the modules drive their output actively.

If the modules are powered from 5 V, their output is 5 V too, and the ESP32-S3 inputs are 3.3 V: power them from 3.3 V, or use a level shifter or a resistor divider.

---

## 4. Configuration reference

| Field | Type | Meaning |
|---|---|---|
| `sensor_gpios[5]` | `gpio_num_t` | Input pins, from the far **left** sensor (index 0) to the far **right** one (index 4), as seen from the robot looking forward |
| `line_level` | `uint32_t` | Level (0 or 1) a sensor outputs when it is **over the line** |

`line_level` depends on two things: the sensor module (some output 0 when they see a reflection, others 1) and the track (dark line on a light floor, or the opposite). Do not guess it: measure it, as described in the example's README.

**No Kconfig.** Keep the pins of your robot in your application (the Juca project keeps them in `main/juca_board.h` as `JUCA_PIN_IR_LINE_*` and `JUCA_IR_LINE_LEVEL_ON_LINE`).

---

## 5. API reference

| Function | What it does | Notes |
|---|---|---|
| `ir_line_init(&config)` | Validates the configuration and sets the five pins as inputs | Can be called again to change the configuration |
| `ir_line_read_mask(&mask)` | Reads the five sensors into one byte | Does not block; a few microseconds. Any task |

---

## 6. The mask and the patterns

Bit `i` of the mask belongs to sensor `i`:

| Bit | 4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|
| Sensor | far right | right | middle | left | far left |

Written in binary, the **left** sensor is therefore the **rightmost** digit. A line normally covers one sensor, or two neighbours when it is between them, which gives nine patterns:

| Constant | Binary | Line is |
|---|---|---|
| `IR_LINE_PATTERN_FAR_LEFT` | `00001` | under the far left sensor |
| `IR_LINE_PATTERN_LEFT_FAR_LEFT` | `00011` | between far left and left |
| `IR_LINE_PATTERN_LEFT` | `00010` | under the left sensor |
| `IR_LINE_PATTERN_LEFT_MIDDLE` | `00110` | between left and middle |
| `IR_LINE_PATTERN_MIDDLE` | `00100` | centered |
| `IR_LINE_PATTERN_RIGHT_MIDDLE` | `01100` | between middle and right |
| `IR_LINE_PATTERN_RIGHT` | `01000` | under the right sensor |
| `IR_LINE_PATTERN_RIGHT_FAR_RIGHT` | `11000` | between right and far right |
| `IR_LINE_PATTERN_FAR_RIGHT` | `10000` | under the far right sensor |

Two more are named: `IR_LINE_PATTERN_NONE` (`00000`, no sensor sees the line) and `IR_LINE_PATTERN_ALL` (`11111`, a crossing, a wide mark, or, with sensors that read "dark" when nothing reflects, the robot lifted off the floor). Any other value means sensors that are not neighbours, or three or four at once: a junction, a sharp corner, dirt, or a sensor out of adjustment.

---

## 7. From the mask to a line position

The component stops at the mask on purpose: what to do with it is a decision of the application. A common choice, used by the `line_avoid` example, is the **average position of the sensors that see the line**:

```c
/* -2 (far left) ... 0 (centered) ... +2 (far right); returns false if no sensor sees the line */
static bool line_position(uint8_t mask, float *position)
{
    int sum = 0, count = 0;
    for (int i = 0; i < IR_LINE_SENSOR_COUNT; i++) {
        if (mask & (1U << i)) { sum += i - 2; count++; }
    }
    if (count == 0) { return false; }
    *position = (float)sum / (float)count;
    return true;
}
```

One sensor gives -2, -1, 0, +1 or +2, and two neighbours give the half steps: nine positions from five sensors.

---

## 8. Error codes

| Function | `ESP_ERR_INVALID_ARG` | `ESP_ERR_INVALID_STATE` | Others |
|---|---|---|---|
| `ir_line_init` | NULL config; a pin that is not a valid GPIO; `line_level` not 0 or 1 | | error of the GPIO driver |
| `ir_line_read_mask` | NULL output pointer | called before `ir_line_init` (the mask is set to 0) | |

---

## 9. Limitations and gotchas

- **The order of the pins is the meaning of the bits.** If left and right come out swapped, the order in `sensor_gpios` is wrong, not the wiring.
- **Wrong polarity looks like "all sensors" on the floor and "no line" over the line.** Flip `line_level`.
- **Digital sensors have a threshold, set by a trimmer on each module.** A sensor that never changes, or flickers, needs its trimmer adjusted; the component cannot compensate.
- **No filtering.** A reading taken exactly at the edge of the line can flicker between two patterns; debounce in the application if it matters.
- **Not validated on the robot in this revision.** The code builds with ESP-IDF v5.3.1 and v5.3.6.

---

## 10. Files

```
components/ir_line/
  CMakeLists.txt        REQUIRES esp_driver_gpio, log
  include/ir_line.h     public API, configuration struct, pattern constants
  ir_line.c             implementation
  README.md             this file
```
