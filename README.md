# Skycart Delivery Rack — ESP32 Firmware
**Embedded Software Engineering Internship** · Skycart · Oct 2025 – Mar 2026

ESP32 firmware written in C++ for an autonomous drone delivery rack system. The rack uses a leadscrew-driven stepper motor to deploy packages and a teeter-totter balance system to keep the rack level relative to the aircraft during flight.

---

## Projects

### 1. Stepper Motor Characterization (`stepper_test`)
Developed test firmware to characterize leadscrew lift performance under a 42 lb load. Ran a test matrix across voltage, speed, and load conditions to establish validated safe operating limits.

**Hardware**
- ESP32 microcontroller
- NEMA 23 stepper motor (23HS16-0884S)
- ISD04 stepper driver
- 4-start leadscrew, 2 mm pitch (lead = 8.0 mm/rev)
- 1/8 microstepping → 5,080 steps/inch

**Validated Results**
| Voltage | Max Safe Speed | Linear Speed |
|---------|---------------|--------------|
| 12V     | 5,000 Hz      | ~0.49 in/s   |
| 24V     | 10,000 Hz     | ~0.98 in/s   |

- Verified leadscrew holds 42 lb load with no observed creep over 3 minutes
- Validated positioning accuracy to within ±0.1 inches across repeated test runs
- Identified thermal failure modes — motor must be disabled when not in motion to prevent overheating

---

### 2. Teeter-Totter Balance System (`teeter_totter`)
Developed center-of-gravity firmware for a teeter-totter style balance system. The rack's support point (where the drone holds the rack) must align with the loaded rack's center of gravity. Two stepper motors slide box carriages left and right to achieve this.

**Approach — Pure Feedforward (no sensor)**
Box masses are entered before flight. The firmware solves for exact carriage positions using the CG formula, then moves the motors once. No IMU or load cells needed.

```
xcg = (Σ mᵢxᵢ + mᵣxᵣ) / (Σmᵢ + mᵣ)
```

When a box is delivered mid-flight, the operator marks it as dropped and the firmware automatically recomputes and rebalances.

---

## Tech Stack
- **Language:** C++
- **Platform:** ESP32 (PlatformIO + Arduino framework)
- **Library:** FastAccelStepper
- **Tools:** VS Code, PlatformIO, Git

---

## Repo Structure
```
src/
├── stepper_test/       # Stepper characterization firmware
├── teeter_totter/      # Balance system firmware
└── shared/
    └── motion_config.h # Shared hardware constants (pins, steps/inch, rack geometry)
archive/                # Previous firmware iterations
platformio.ini          # Two build environments, one per firmware
```

## Flashing
1. Open the project in VS Code
2. In the PlatformIO toolbar at the bottom, click the environment name and select either `stepper_test` or `teeter_totter`
3. Hit the upload button (→)
