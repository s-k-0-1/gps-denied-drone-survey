# 02 — Docking & Charging (ESP32)

How the drone is automatically seated on the charging pads, how the firmware detects contact,
polarity and battery voltage, and how it all appears on the dashboard.

**Firmware:** `esp32_firmware/full_base_station_wifi.ino`

---

## 1. The idea

The drone lands roughly on a platform. Two motor-driven rods then push it into a repeatable
position where two charging pads on the drone touch two pads on the station. Because the drone
can land facing either way, the station does not assume polarity — it **measures** which pad is
positive and drives an H-bridge in the matching direction.

Everything is done by one ESP32, which also mirrors its serial log to the dashboard over WiFi.

---

## 2. Full sequence (what happens, in order)

```
 landing detected ──► /landed ──► rods drive forward
                                        │
                        contact sensed (one pad high, other low)
                                        │
                          wait 1 s (settle) ──► stop rods
                                        │
                       wait 2 s ──► push forward 1 s ──► rods park (never reverse)
                                        │
              stabilize 3 s ──► confirm polarity + read pack voltage
                                        │
                            countdown 10 s ──► CHARGING (continuous)
```

| Step | Timing constant | Why it exists |
|---|---|---|
| Docking timeout | `DOCKING_TIMEOUT_MS` = 30 s | If contact never happens, stop the motors anyway |
| Contact settle | `CONTACT_SETTLE_MS` = 1 s | Ignore a momentary touch; let the drone fully seat |
| Push delay / time | `PUSH_DELAY_MS` = 2 s, `PUSH_TIME_MS` = 1 s | Final firm seating push after contact |
| Stabilize | `STABILIZE_MS` = 3 s | Let the voltage reading settle before trusting polarity |
| Countdown | `COUNTDOWN_MS` = 10 s | Human-visible warning before current flows |

> The rods **only ever move forward**. They stop where they seat and never reverse, so the drone
> stays clamped for the whole charge.

---

## 3. Contact and polarity detection

Both pads are measured continuously (every 20 ms, non-blocking):

```cpp
a_high = (V_padA > HIGH_THRESHOLD)   // 3.0 V
b_high = (V_padB > HIGH_THRESHOLD)
a_low  = (V_padA < LOW_THRESHOLD)    // 0.5 V
b_low  = (V_padB < LOW_THRESHOLD)

contactOK = (a_high && b_low) || (b_high && a_low);
```

Reading this table:

| Pad A | Pad B | Meaning |
|---|---|---|
| low | low | No contact (nothing connected) |
| **high** | **low** | Contact — **Pad A is positive** |
| **low** | **high** | Contact — **Pad B is positive** |
| high | high | Ambiguous / fault — not accepted as contact |

So a single condition proves *both* that contact exists *and* which way round the battery is.
The firmware stores that as `padA_positive` and sets the H-bridge direction accordingly:

```cpp
if (padA_positive) { RPWM = HIGH; LPWM = LOW;  }   // charge one way
else               { RPWM = LOW;  LPWM = HIGH; }   // charge the other way
digitalWrite(EN_PIN, HIGH);                        // then enable
```

> **Important detail:** direction is re-applied *every* time charging is enabled. Enabling the
> bridge with both PWM pins LOW is an undefined state that made the wires heat up — that bug is
> fixed by `chargingEnable()` always calling `setChargeDirection()` first.

---

## 4. Voltage measurement

```cpp
uint32_t mv = 0;
for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(pin);   // 4-sample average
float v_pin = (mv / 4.0f) / 1000.0f;      // volts at the ADC pin
float v     = v_pin * DIVIDER_RATIO;      // volts at the pad
if (v < 0.3f) return 0.0f;                // genuinely disconnected
return (v * CAL_SLOPE) + CAL_OFFSET;      // fine calibration
```

Why it is done this way:

- **`analogReadMilliVolts()`** uses the ESP32's factory eFuse ADC calibration, which corrects most
  of the chip's non-linearity — far more accurate than `analogRead() * 3.3/4095`.
- **4-sample average** steadies the reading without blocking the step timing.
- **The `< 0.3 V` guard** stops the calibration offset from turning "no contact" into a fake voltage.
- **Divider ratio** must keep the pin below ~2.4 V at maximum pack voltage — see
  [01 — Hardware §3.3](01_HARDWARE.md#33-voltage-sensing-contact--battery-voltage).

The measured pack voltage at the start of charging is reported as `batteryVoltageAtStart` and
printed to serial + the dashboard:

```
POLARITY : Pad A = POSITIVE
BATTERY VOLTAGE: 14.82 V
```

Live pad voltages are also printed twice a second, including the **raw pin millivolts** so you can
spot ADC clipping:

```
Pad A: 14.82 V | Pad B: 0.01 V  [pinA=2600mV pinB=2mV]
```

> If `pinA` stops rising above ~2400 mV while the real pack voltage keeps increasing, the ADC is
> clipping → change the divider resistors.

---

## 5. Charging state machine

```
CS_IDLE ──contact──► CS_STABILIZING ──3 s──► CS_POLARITY_CONFIRMED
                                                     │
                                              CS_COUNTDOWN (10 s)
                                                     │
                                              CS_CHARGING  ← runs continuously
```

| State | What it does | Abort condition |
|---|---|---|
| `CS_IDLE` | Waiting for `contactMade` | — |
| `CS_STABILIZING` | 3 s settle before trusting the reading | contact lost → back to IDLE |
| `CS_POLARITY_CONFIRMED` | Decides positive pad, records start voltage, sets bridge direction | contact lost → back to IDLE |
| `CS_COUNTDOWN` | Prints "charging starts in N s" each second | contact lost → back to IDLE |
| `CS_CHARGING` | Charger enabled; prints elapsed time every 5 s | — (see note) |
| `CS_COMPLETE` / `CS_ERROR` | Disables the charger, prints a final message | — |

**Charging has no automatic cut-off in this build.** It starts after the countdown and runs
continuously. To end it: remove the battery from the pads (contact loss aborts to IDLE) or press
**STOP** (`/dockstop`), which disables the bridge and resets everything.

> If you want an automatic stop, add a check in `CS_CHARGING` comparing the pad voltage against
> your pack's full-charge ceiling (4S Li-ion = 4 × 4.2 V = **16.8 V**) and transition to
> `CS_COMPLETE`. Verify the ceiling against your specific cell's datasheet first.

---

## 6. Motor control detail (why it is smooth)

Originally the step pulses were made in `loop()` with `delayMicroseconds()`. WiFi and the web
server kept interrupting it, so the pulses were uneven — one motor ran fast, the other slow, and
both were noisy.

Now the ESP32's **LEDC hardware peripheral** outputs a rock-steady square wave on each STEP pin at
the same frequency (`STEP_HZ = 400`):

```cpp
ledcAttach(stepPin1, STEP_HZ, 8);      // hardware square wave, zero CPU jitter
ledcWrite(stepPin1, on ? 128 : 0);     // 50 % duty = run, 0 = stop
```

`loop()` only switches the wave on/off to follow the existing `motorXMoving` flags. Both motors
therefore run at *exactly* the same speed. Torque is set by the A4988 VREF pot, not by the code.

Also: `WiFi.setSleep(false)` is essential. In station mode the ESP32 otherwise parks its radio
between router beacons, and the wake-ups stall the CPU for a few milliseconds — enough to make
steppers buzz or stall.

---

## 7. Network integration with the dashboard

### 7.1 ESP32 → dashboard

| What | Endpoint called on the dashboard | When |
|---|---|---|
| Register its own IP | `POST /api/dock_register?token=…&ip=…` | On connect, then every 30 s |
| Mirror serial log | `POST /api/dock_log?token=…` (text body) | Whenever it prints |

A background FreeRTOS task on **core 0** does the HTTP work, so blocking network calls can never
disturb motor timing in `loop()` (core 1). Log lines are queued and sent in batches.

Because the ESP32 self-registers, the ground PC always knows its address even with DHCP — no
fragile static IP needed.

### 7.2 Dashboard → ESP32

| Dashboard action | ESP32 route | Effect |
|---|---|---|
| Drone landed (auto, after `DOCK_DELAY_S`) | `GET /landed` | Start docking |
| **Start Docking** button | `GET /landed` | Same, manually |
| **STOP** button | `GET /dockstop` | Stop motors, disable charging, reset |
| (status polling) | `GET /status` | Docking active, contact, pad voltages, charge state |

All machine-to-machine calls carry the shared token (`DOCK_TOKEN` on the ESP32 must equal
`IROC_TOKEN` on the dashboard, default `lumadock`) so they bypass the dashboard's password.

### 7.3 What you see on the dashboard

The **Docking & Charging** panel streams the ESP32's own log lines live:

```
>>> LANDED received. Docking started. <<<
>>> Contact detected. Settling for 1 second... <<<
>>> Settled. Docking motors stopped. <<<
>>> Final seat push (forward)... <<<
POLARITY : Pad A = POSITIVE
BATTERY VOLTAGE: 14.82 V
>>> Charging starts in: 10 s
>>> CHARGING STARTED <<<
[CHARGING] total time: 35 s
```

Battery voltage therefore reaches the dashboard **directly from the ESP32** — the vision pipeline
is not involved in it at all.

---

## 8. Flashing the firmware

1. Install the **Arduino IDE** and add ESP32 board support
   (Boards Manager URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`).
2. Open `esp32_firmware/full_base_station_wifi.ino`.
3. Edit at the top: `WIFI_SSID`, `WIFI_PASS`, `BASE_URL` (your PC's IP), `DOCK_TOKEN`,
   and `DIVIDER_RATIO` for your resistors.
4. Select board **ESP32 Dev Module**, choose the serial port, click **Upload**.
5. Open Serial Monitor at **115200** baud — it prints the IP address once connected.

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Voltage reads low and stops rising | ADC clipping above ~2.4 V | Increase divider ratio (e.g. 47 k + 6.8 k), update `DIVIDER_RATIO` |
| Voltage reads slightly off | Calibration | Set `CAL_SLOPE` / `CAL_OFFSET` from two multimeter points |
| Contact never detected | Thresholds or wiring | Check pad continuity, common ground, `HIGH/LOW_THRESHOLD` |
| Motors buzz / stall | Current limit or WiFi sleep | Set A4988 VREF; ensure `WiFi.setSleep(false)`; lower `STEP_HZ` |
| One motor faster than the other | Old software stepping | Confirm the LEDC path is used (`stepperSetup()`) |
| Wires heat up after a pause | Both PWM low with EN high | Already fixed — `chargingEnable()` re-applies direction |
| Logs missing on the dashboard | Token / URL mismatch | `DOCK_TOKEN` must equal `IROC_TOKEN`; check `BASE_URL` IP |
| Docking stops after 30 s | Timeout with no contact | Re-align the platform, check pad heights |

---

**Next:** [03 — Data Transfer](03_DATA_TRANSFER.md)
