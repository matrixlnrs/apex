# APEX — Jump-height meter on STM32

**How high did you just jump?** APEX answers that with a small box you hold while jumping. It has no camera, no GPS and no idea where the ground is. It only has an accelerometer, a timer and a bit of physics.

Set a target height with a potentiometer, jump, and the 7-segment display shows your **height** and **time in the air**. Beat your target and a **vibration motor** celebrates with you.

> Microprocessor Systems project — ISEN Toulon.
> STM32 NUCLEO-L152RE · X-NUCLEO-IKS01A3 sensor shield · ISEN32 I/O board.

---

## The idea: measure time, not height

You can't measure a height directly without seeing the ground. But you can measure how long you stay in the air.

- **Take-off.** As soon as your feet leave the ground, the box is in free fall. The accelerometer reads close to **0 g**.
- **Landing.** The impact produces a sharp **acceleration spike**.
- **Time to height.** Going up takes as long as coming down (`t/2` each way), so the flight time `t` gives the height:

```
h = ½ · g · (t/2)²  =  g · t² / 8        (g = 9.81 m/s², t in seconds)
```

A 500 ms flight gives about **30.7 cm**.

---

## How it works

The firmware is a four-state machine:

```
SETUP ──[button]──► IDLE ──[free fall]──► RUNNING ──[impact]──► RESULT ──► SETUP
```

| State | What happens |
|---|---|
| **SETUP** | Turn the potentiometer to choose a target (in cm). It updates live on the display. Confirm with a button. |
| **IDLE** | Ready. The accelerometer is read continuously, and take-off is detected when `|a|` stays below the free-fall threshold for at least 15 ms. |
| **RUNNING** | In the air. A 1 ms hardware timer counts the flight. The display stays frozen on `----`, so no SPI traffic disturbs the sampling. |
| **RESULT** | The impact stops the timer. The height is computed, ambient humidity is read, and **height / time / humidity** are shown in turn. If the target is reached, the motor vibrates three times. |

### Detection details

- **Thresholds on the acceleration magnitude `|a|`** (in mg):
  - take-off when `|a| < 300 mg` for at least **15 ms**. The start time is then backdated, so the confirmation delay does not shorten the measured flight;
  - landing on an impact peak `|a| > 1800 mg`;
  - flights outside **80–2000 ms** are rejected as noise.
- **Fast sampling.** The accelerometer runs at **416 Hz** over **400 kHz I²C** and is read on every loop iteration, so neither the free-fall dip nor the impact peak is missed.
- **Timing.** TIM6 runs with `PSC = 31` and `ARR = 999`: 32 MHz / 32 / 1000 gives an interrupt **every 1 ms**.
- **Button debouncing** relies on `HAL_GetTick()` (SysTick), independent of TIM6.

All the thresholds are `#define`s at the top of [`Core/Src/main.c`](Core/Src/main.c).

---

## Hardware and peripherals

| Peripheral | Role | Pins | Mode |
|---|---|---|---|
| **GPIO + EXTI** | Confirm buttons, LEDs | PC6, PC5, PA11, PA12 · PB1/2/10–15 | interrupt |
| **TIM6** | Flight timer, 1 ms tick | internal | interrupt |
| **ADC** | Potentiometer (target height) | PA0 | polling |
| **I²C1** | Shield sensors | PB8 (SCL) / PB9 (SDA) | polling |
| **SPI1** | MAX7219 7-segment display | PA5/PA6/PA7, PA8 (CS) | polling |
| **USART2** | Serial logs to the PC, 115200 baud | PA2/PA3 | — |
| **GPIO out** | Vibration motor | PB4 | — |

| Sensor (IKS01A3) | I²C address | Used for |
|---|---|---|
| **LSM6DSO** accelerometer | `0xD6` | Take-off / landing detection, height computation |
| **HTS221** humidity sensor | `0xBE` | Ambient conditions shown with the result |

---

## Debugging stories

Getting a reliable jump detection took three investigations.

1. **The wrong suspect.** The buttons did nothing. Pull-ups and debouncing looked guilty but were fine. The real cause was a timer configured with `ARR = 0`: the millisecond counter never moved, and the debouncing depended on it.
2. **The ghost address.** An I²C scan returned silence on all 127 addresses. Power, wiring and configuration were all correct. The firmware was talking on PB6/PB7 while the shield is wired to **PB8/PB9**. The fix came from reading the shield's schematic, not just the MCU datasheet.
3. **The missed moment.** Only one jump in three was detected. UART logs showed that sampling every 10 ms was too slow to catch both the free-fall dip and the impact peak. Moving to 416 Hz, reading on every loop and freezing the display during flight fixed it.

The method each time: **isolate, measure, prove.**

---

## Build and flash

**STM32CubeIDE (recommended)**

1. *File → Open Projects from File System…* and select this folder.
2. Build, then *Run* to flash the NUCLEO board through the on-board ST-LINK.
3. Open a serial terminal at **115200 baud, 8N1** on the ST-LINK virtual COM port to follow the logs.

**Command line** (with the CubeIDE ARM toolchain in your `PATH`):

```bash
cd Debug
make -j4 all
```

The firmware is generated as `Debug/APEX_PROJECT.elf`.

---

## Project layout

```
Core/Src/main.c               application: state machine, sensors, detection, display
Core/Src/stm32l1xx_it.c       interrupt handlers (TIM6, button EXTI)
Core/Src/max7219_Yncrea2.c    7-segment display driver (SPI)
Core/Inc/main.h               pin definitions and state enum
APEX_PROJECT.ioc              STM32CubeMX configuration
Documentation/                project brief and presentation slides
```

---

## Team

Built by **Julien Gomis**, **Mathis Lenorais-Clere** and **Neil Ben-Othman** for the Microprocessor Systems course at ISEN Toulon.
