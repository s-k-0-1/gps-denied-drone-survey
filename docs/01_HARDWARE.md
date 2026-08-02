# 01 — Hardware

Physical structure of the system, every component, how they are wired, and how to use each part.

---

## 1. System overview

The system has three physical units that talk to each other over one WiFi network:

```
        ┌────────────────────────────────────────────┐
        │  DRONE (ASCEND)                            │
        │  • Tarot 650 frame · Emax 935 KV ×4        │
        │    1045 props · BLHeli_S 30 A ESCs ×4      │
        │  • Pixhawk Cube Orange+  (flight control)  │
        │  • Jetson Orin Nano  (via 75 W buck → 12 V)│
        │  • RealSense D455  ·  MTF-01 optical flow  │
        │  • 4S Li-ion + BMS → 2-pin charging pads   │
        └──────────┬─────────────────────────────────┘
                   │  MAVLink (USB) + photos saved on the Jetson
                   ▼
        ┌──────────────────────────────┐        WiFi        ┌───────────────────────┐
        │  JETSON (onboard computer)   │◄──────────────────►│  GROUND PC            │
        │  • captures photos + pose    │   mavlink-router   │  • dashboard (:8000)  │
        │  • runs mavlink-router       │   TCP 5760         │  • runs the pipeline  │
        └──────────────────────────────┘                    └───────────┬───────────┘
                                                                        │ HTTP
                                                            ┌───────────▼───────────┐
                                                            │  BASE STATION (ESP32) │
                                                            │  • docking rods       │
                                                            │  • charging + voltage │
                                                            └───────────────────────┘
```

---

## 2. Component list

### 2.1 Drone

| # | Component | Part / model | Qty | Purpose |
|---|---|---|---|---|
| 1 | Frame | **Tarot TL65B01 Iron Man 650** foldable quadcopter | 1 | Airframe (650 mm, folding arms) |
| 2 | Flight controller | **Pixhawk Cube Orange+** (PX4 firmware) | 1 | Flight control, IMU, telemetry |
| 3 | Companion computer | **NVIDIA Jetson Orin Nano** | 1 | Photo capture, pose logging, MAVLink routing |
| 4 | Camera | **Intel RealSense D455** (downward, 1280×720) | 1 | Survey imagery |
| 5 | Optical flow / rangefinder | **MTF-01** | 1 | GPS-free position hold (VIO input) |
| 6 | Motors | **Emax 935 KV** brushless | 4 | Propulsion |
| 7 | Propellers | **1045** (10 × 4.5) | 4 | Thrust |
| 8 | ESCs | **LittleBee BLHeli_S 30 A** | 4 | Motor speed control |
| 9 | Battery | **4S Li-ion pack**, 16.8 V full charge | 1 | Flight power + charged at the dock |
| 10 | BMS | 4S BMS with **2-pin** balance/charge connection | 1 | Balance charging + pack protection |
| 11 | Buck converter | **75 W step-down**, output 12 V | 1 | Powers the Jetson Orin Nano from the pack |
| 12 | Charging pads | 2 contact pads (Pad A / Pad B) on the landing gear | 2 | Docking contact + charge input |
| 13 | Telemetry link | WiFi (shared network) | 1 | Live telemetry + file transfer |

> **No GPS module is used for localization** — the rulebook prohibits GNSS. Position comes
> from the camera + Pixhawk VIO / optical flow.

### 2.2 Base station (docking + charging)

| # | Component | Part / model | Purpose |
|---|---|---|---|
| 1 | Microcontroller | ESP32 DevKit (WROOM) | Runs docking + charging logic, WiFi server |
| 2 | Stepper drivers ×4 | A4988 | Drive the two docking rods |
| 3 | Stepper motors ×4 | NEMA 17 | Push the drone into the charging pads |
| 4 | Motor driver / charger | BTS7960 H-bridge | Switches charge current, handles either polarity |
| 5 | Voltage sense | 2 × resistor divider → ESP32 ADC | Reads pad voltage, detects contact + polarity |
| 6 | Power supply | 24 volt  | Charge source |
| 7 | Landing platform | 2  copper pads  | Mechanical alignment + pads |

### 2.3 Ground

| # | Component | Purpose |
|---|---|---|
| 1 | Laptop / PC (Linux or Windows + WSL2) | Dashboard + pipeline processing |
| 2 | WiFi router or phone hotspot (SSID `LUMA`) | Connects Jetson, ESP32 and PC |

### 2.4 Propulsion

| Item | Spec |
|---|---|
| Frame | Tarot TL65B01 Iron Man 650, foldable quad, 650 mm wheelbase |
| Motors | Emax **935 KV**, ×4 |
| Propellers | **1045** (10 inch diameter, 4.5 inch pitch), ×4 — 2 CW + 2 CCW |
| ESCs | LittleBee **BLHeli_S 30 A**, ×4 |
| Pack | 4S Li-ion (14.8 V nominal / 16.8 V full) |

**Why this combination:** a 935 KV motor on 4S spins a large, low-pitch 1045 prop — high static
thrust at modest RPM, which is what a slow survey platform needs (stable hover, long endurance,
low vibration for clean imagery). The 30 A ESC leaves comfortable headroom over the motor's
continuous draw.

**Motor / ESC wiring**

```
  Battery (4S) ──► PDB / power rail ──┬──► ESC 1 ──► Motor 1 (front-right, CCW)
                                      ├──► ESC 2 ──► Motor 2 (front-left,  CW )
                                      ├──► ESC 3 ──► Motor 3 (rear-left,   CCW)
                                      └──► ESC 4 ──► Motor 4 (rear-right,  CW )

  Each ESC signal wire ──► Pixhawk Cube Orange+ MAIN OUT 1-4
  ESC ground ──► common ground with the flight controller
```

- Motor order and rotation direction must match the PX4 **Quadrotor X** layout. Verify with
  QGroundControl → **Actuators / Motor Test** before ever fitting propellers.
- BLHeli_S ESCs need one-time calibration, or configure DShot in QGroundControl
  (`DSHOT_CONFIG`) — do this with props removed.
- Fit propellers only after motor order and direction are confirmed correct.

### 2.5 Power distribution

One 4S pack powers everything. The Jetson cannot take pack voltage directly, so a buck converter
steps it down.

```
                    ┌─────────────────────────────────────────────┐
   4S Li-ion pack ──┤                                             │
   (14.8–16.8 V)    │                                             │
        │           ├──► ESCs ×4 ──► motors            (pack V)   │
        │           │                                             │
        │           ├──► Pixhawk Cube Orange+ (via power module)  │
        │           │                                             │
        │           └──► 75 W BUCK CONVERTER ──► 12 V ──► Jetson  │
        │                                            Orin Nano    │
        │                                                         │
        └──► BMS ──► 2-pin charge connection ──► charging pads ───┘
                                                 (Pad A / Pad B)
```

**Buck converter (75 W, 12 V out) — powering the Jetson**

| Item | Value / note |
|---|---|
| Input | 14.8–16.8 V (direct from the 4S pack) |
| Output | **12 V**, set and verified with a multimeter **before** connecting the Jetson |
| Rating | 75 W ≈ 6 A at 12 V — comfortably above the Orin Nano's draw with the D455 attached |
| Wiring | Pack **+** → buck `IN+`, pack **−** → buck `IN−`; buck `OUT+/OUT−` → Jetson barrel jack |
| Cautions | Set the output voltage with **no load connected first**. Keep the converter ventilated — it warms up under sustained load. Use a common ground with the flight controller. |

> ⚠️ Never connect the Jetson before confirming the buck output reads 12 V. A miswired or
> unadjusted converter passing 16 V will destroy the board.

**BMS — balance charging over 2 pins**

The pack has a 4S BMS. Charging current and cell balancing both happen through the BMS, so the
drone only needs **two** external contacts (pack **+** and pack **−**) — the same two pads used
for docking.

| Aspect | Detail |
|---|---|
| Connection | **2-pin**: charge **+** and charge **−** from the BMS to the landing-gear pads (Pad A / Pad B) |
| Balancing | Handled internally by the BMS; no separate balance lead is exposed |
| Protection | BMS provides over-charge, over-discharge and over-current cut-off for the pack |
| Charge source | Base-station 24 V supply, switched by the BTS7960 H-bridge (see §3.2) |

**Why 2-pin matters for docking:** with only two contacts the drone can seat in either
orientation. The ESP32 measures both pads, works out which one is positive, and drives the
H-bridge accordingly — so no mechanical keying or manual alignment is required. Details in
[02 — Docking & Charging](02_DOCKING_CHARGING.md).

> The BMS is the last line of defence, but the base-station firmware has **no automatic charge
> cut-off** in this build — supervise charging and end it by removing the drone or pressing
> **STOP**.

---

## 3. Wiring — ESP32 base station

All pin numbers below are **exactly** what the firmware
(`esp32_firmware/full_base_station_wifi.ino`) uses. Change the code if you rewire.

### 3.1 Stepper motors (A4988 drivers)

| Signal | ESP32 pin | Goes to |
|---|---|---|
| Motor 1 STEP | **GPIO 16** | A4988 #1 `STEP` |
| Motor 1 DIR | **GPIO 17** | A4988 #1 `DIR` |
| Motor 2 STEP | **GPIO 18** | A4988 #2 `STEP` |
| Motor 2 DIR | **GPIO 19** | A4988 #2 `DIR` |

A4988 wiring (per driver):

```
  VMOT ── motor supply (8–35 V)   +  100 µF capacitor across VMOT/GND (REQUIRED)
  GND  ── supply ground  (common with ESP32 GND)
  VDD  ── 3.3 V from ESP32
  1A 1B 2A 2B ── stepper motor coils
  ENABLE ── GND (always enabled)   |   MS1/MS2/MS3 ── set microstepping
  RESET ── SLEEP (tie together)
```

- **Set the current limit** with the A4988 trim-pot (VREF) before running the motors, or the
  driver overheats. Follow the standard `VREF = I_max × 8 × R_sense` procedure for your board.
- Step rate is generated in **hardware (LEDC)** at `STEP_HZ = 400` steps/s — both motors run at
  exactly the same speed with no CPU jitter. Lower this value for **more torque**, raise for speed.

### 3.2 Charging (BTS7960 H-bridge)

| Signal | ESP32 pin | Goes to |
|---|---|---|
| RPWM | **GPIO 26** | BTS7960 `RPWM` |
| LPWM | **GPIO 27** | BTS7960 `LPWM` |
| Enable (both) | **GPIO 25** | BTS7960 `R_EN` **and** `L_EN` |

```
  B+ / B- ── charge power supply
  M+ / M- ── charging pads (Pad A / Pad B)
  VCC ── 3.3 V (logic)     GND ── common ground
```

The H-bridge exists so the charger works **whichever way round the drone lands** — the firmware
detects which pad is positive and drives the bridge in the matching direction.

### 3.3 Voltage sensing (contact + battery voltage)

| Signal | ESP32 pin | Goes to |
|---|---|---|
| Pad A sense | **GPIO 32** (ADC1_CH4) | Divider on Pad A |
| Pad B sense | **GPIO 33** (ADC1_CH5) | Divider on Pad B |

Each pad has its own divider:

```
   PAD ──[ R1 ]──┬──[ R2 ]── GND
                 │
                 └────────── ESP32 ADC pin (32 or 33)

   DIVIDER_RATIO = (R1 + R2) / R2      ← must match the firmware constant
```

⚠️ **Critical:** the ESP32 ADC is only linear up to **≈ 2.4 V**. Your **maximum** pack voltage
must land **below** that at the pin, otherwise the reading clips and reports a falsely low voltage.

| Resistors | Ratio | 17 V pack → pin | Verdict |
|---|---|---|---|
| 47 k + 10 k | 5.7 | 2.98 V | ❌ clips |
| 47 k + 6.8 k | 7.9 | 2.15 V | ✅ safe |
| 100 k + 15 k | 7.7 | 2.21 V | ✅ safe |

If you change resistors, update `DIVIDER_RATIO` in the firmware to match.

**Calibration:** the firmware reads voltage with `analogReadMilliVolts()` (uses the ESP32's
factory eFuse ADC calibration — much more accurate than raw `analogRead()`), then applies a
fine trim:

```
actual = (measured × CAL_SLOPE) + CAL_OFFSET      // start at 1.0 / 0.0
```

To calibrate: measure the real pad voltage with a multimeter at two different levels, compare
with the serial printout, and solve for slope/offset.

### 3.4 Grounding

**All grounds must be common:** ESP32 GND ↔ A4988 GND ↔ BTS7960 GND ↔ supply GND ↔ pad negative.
Without a shared ground the ADC readings are meaningless and the drivers behave erratically.

---

## 4. How to use the hardware

### 4.1 Power-up order (base station)

1. Connect the ESP32 to USB (or its 5 V supply) — it boots and joins WiFi `LUMA`.
2. Power the motor supply (A4988 VMOT).
3. Power the charge supply (BTS7960 B+ / B−).
4. Open the ESP32's IP in a browser (it is printed on serial, and auto-registered with the
   dashboard) — you get a manual control page with Forward/Reverse per motor, **LANDED** and **STOP**.

### 4.2 Manual controls (ESP32 web page)

| Button | Effect |
|---|---|
| Motor 1 / 2 — Forward, Reverse | Jog a rod while the button is held (for alignment/testing) |
| **LANDED** | Starts the full automatic docking + charging sequence |
| **STOP** | Immediately stops motors, disables charging, resets the state machine |

Same actions are available from the dashboard's *Docking & Charging* panel.

### 4.3 Normal automatic cycle

1. Drone lands on the platform.
2. Dashboard waits `DOCK_DELAY_S` (default 5 s), then calls the ESP32's `/landed`.
3. Rods drive forward until the pads make contact (details in
   [02 — Docking & Charging](02_DOCKING_CHARGING.md)).
4. Firmware confirms polarity, reports the pack voltage, counts down 10 s, then charges.

### 4.4 Safety checklist

**Drone**

- **Propellers off** for every bench test — motor order, direction and ESC calibration are all
  checked with props removed.
- Confirm the **QUAD X** motor order and rotation in Motor Test before the first flight.
- Set the **buck converter to 12 V with no load connected**, verify with a multimeter, *then*
  connect the Jetson. A converter left at pack voltage will destroy it.
- Check the BMS 2-pin polarity at the landing-gear pads before the first docking attempt.
- Keep the buck converter ventilated; it warms up under sustained load.

**Base station**

- Set A4988 current limits **before** the first run.
- Verify the divider ratio and confirm the ADC never exceeds ~2.4 V at maximum pack voltage.
- Confirm common ground everywhere.
- Keep the 100 µF capacitors across each A4988 VMOT.
- Charging in this firmware **runs continuously** — it has no automatic cut-off. The BMS protects
  the pack, but still supervise it, and end the cycle by removing the battery from the pads
  (loss of contact aborts to IDLE) or by pressing **STOP**.
- Never hot-plug the stepper motors while VMOT is live (destroys A4988 drivers).

---

## 5. Configuration values (firmware)

Edit these at the top of `esp32_firmware/full_base_station_wifi.ino`:

| Constant | Default | Meaning |
|---|---|---|
| `WIFI_SSID` / `WIFI_PASS` | `LUMA` / … | Network the ESP32 joins |
| `BASE_URL` | `http://<PC-IP>:8000` | Dashboard address (get PC IP with `hostname -I`) |
| `DOCK_TOKEN` | `lumadock` | Must match `IROC_TOKEN` on the dashboard |
| `DIVIDER_RATIO` | `5.7` | Your resistor divider ratio — **update to match your resistors** |
| `CAL_SLOPE` / `CAL_OFFSET` | `1.0` / `0.0` | Fine voltage calibration |
| `HIGH_THRESHOLD` / `LOW_THRESHOLD` | `3.0 V` / `0.5 V` | Contact / polarity decision levels |
| `STEP_HZ` | `400` | Stepper speed (lower = more torque) |
| `DOCKING_TIMEOUT_MS` | `30000` | Give up docking after 30 s without contact |
| `CONTACT_SETTLE_MS` | `1000` | Wait after first contact before stopping the rods |
| `PUSH_DELAY_MS` / `PUSH_TIME_MS` | `2000` / `1000` | Final seating push timing |
| `STABILIZE_MS` / `COUNTDOWN_MS` | `3000` / `10000` | Charging state-machine delays |

---

**Next:** [02 — Docking & Charging](02_DOCKING_CHARGING.md)
