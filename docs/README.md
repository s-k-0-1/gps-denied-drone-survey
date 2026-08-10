# Documentation

Everything about this system, written for someone seeing it for the first time.

---

> 🔎 **Looking for something specific?** → **[INDEX — where to find anything](INDEX.md)**
> (A–Z topics · every file explained · parameter lookup · error-message lookup)

---

## Start here

| I want to… | Read |
|---|---|
| **Set the whole thing up from nothing** | [00 — Install Everything](00_INSTALL_EVERYTHING.md) |
| **Fly it** (commands, tests, checklists) | [13 — Operations Runbook](13_OPERATIONS.md) |
| **Understand how it works** | [How It Works](../HOW_IT_WORKS.md) |
| **Fix something that's broken** | [08 — Troubleshooting](08_TROUBLESHOOTING.md) |
| **Tune a parameter** | [07 — Stage Guide](07_STAGE_GUIDE.md) · [Parameters Guide](../PARAMETERS_GUIDE.md) |

---

## By subsystem

### 🚁 The drone

| Doc | What's inside |
|---|---|
| [09 — Drone Software](09_DRONE_SOFTWARE.md) | The ROS 2 stack: architecture, every node, the autonomous missions, yellow-boundary safety |
| [10 — VIO & Localization](10_VIO_LOCALIZATION.md) | Optical flow + RTAB-Map, the gated handover, loop closure, what happens when the camera fails |
| [11 — Pixhawk & PX4](11_PIXHAWK_PX4.md) | Wiring, MAVROS config, the PX4 parameters for GPS-denied flight, tuning order |

### 💻 Ground processing

| Doc | What's inside |
|---|---|
| [04 — Pipeline / Feature Detection](04_PIPELINE.md) | Stage-by-stage detection, and what every file does |
| [07 — Stage Guide](07_STAGE_GUIDE.md) | Per stage: what runs, which file, the few parameters that matter, how to fix it |
| [05 — Setup](05_SETUP.md) | Ground-PC installation walkthrough |

### 🔌 Base station

| Doc | What's inside |
|---|---|
| [01 — Hardware](01_HARDWARE.md) | Build photos, components, **complete wiring tables**, power, safety |
| [02 — Docking & Charging](02_DOCKING_CHARGING.md) | Docking sequence, contact + polarity detection, voltage, charging state machine |

### 🔗 How they connect

| Doc | What's inside |
|---|---|
| [03 — Data Transfer](03_DATA_TRANSFER.md) | Photos and telemetry drone → Jetson → PC; MAVLink routing; file layout |
| [12 — End-to-End Automation](12_END_TO_END_AUTOMATION.md) | What happens by itself after touchdown: docking, transfer, pipeline |

### 🛠 Practical

| Doc | What's inside |
|---|---|
| [00 — Install Everything](00_INSTALL_EVERYTHING.md) | All three machines from zero, in the order you use them |
| [13 — Operations Runbook](13_OPERATIONS.md) | Every command, with a test after every step |
| [06 — Git & GitHub](06_GIT_GITHUB.md) | Complete beginner's guide to git |
| [08 — Troubleshooting](08_TROUBLESHOOTING.md) | Every symptom → fix, in one table |

---

## Reference (repository root)

| Doc | What's inside |
|---|---|
| [How It Works](../HOW_IT_WORKS.md) | Full algorithm explanation and the design decisions behind them |
| [Parameters Guide](../PARAMETERS_GUIDE.md) | Every tunable parameter, when to change it and why |
| [Run Guide](../RUN_GUIDE.md) | Run + validation checklist |
| [64×64 Mode](../PARAMETERS_64x64.md) | Tuning for the alternate 64×64 matching mode |

---

## Two paths through the docs

**Building this from scratch**

```
00 Install  →  01 Hardware  →  13 Operations (bench tests)
            →  11 Pixhawk   →  13 Operations (first flights)
            →  09 Drone Software  →  survey flight
            →  04/07 Pipeline     →  results
```

**Something is wrong**

```
08 Troubleshooting  →  the stage-specific doc it points to
```

---

## Images

`images/` holds the build photos, result screenshots and diagrams used across these docs —
including the [system architecture](images/architecture.svg) and the
[RTAB-Map loop closure explainer](images/rtabmap_loop_closure.svg).
