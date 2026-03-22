```markdown
# ABS ECU Simulation

A realistic, modular, C++17 simulation of an Anti-lock Braking System (ABS) Electronic Control Unit (ECU) for automotive embedded systems. The project demonstrates a closed‑loop control system with sensor input, ABS logic, and actuator output, running in a real‑time loop (20 ms cycle) with a simplified but physically consistent vehicle and wheel dynamics model.

## Overview

This project simulates an ABS ECU that monitors four wheel speeds, estimates vehicle speed, calculates wheel slip, and modulates brake pressure to prevent wheel lock during hard braking. It follows automotive embedded design principles:

- **Modular OOP architecture** – separation of sensor, control, and actuator layers.
- **Real‑time loop** – 20 ms cycle using `std::chrono` and `std::thread`.
- **Slip‑ratio‑based control** – three‑state brake pressure regulation (APPLY, HOLD, RELEASE).
- **Simplified physics** – vehicle deceleration proportional to average brake pressure, wheel deceleration/recovery influenced by brake pressure and road friction.
- **Sensor noise** – Gaussian noise added to wheel speed readings for realism.
- **CSV logging** – records all relevant data for post‑processing and analysis.

---

## Features

- **Four independent wheel sensors** – each with individual noise.
- **Vehicle speed estimation** – average of wheel speeds (simple but effective).
- **Slip calculation** – `(V_veh - V_wheel) / V_veh`.
- **ABS control logic** – applies, holds, or releases brake pressure based on slip thresholds.
- **Pressure ramping** – smooth pressure changes (step‑wise ±10% per cycle).
- **Physics update** – wheel speed changes due to brake pressure and road friction; vehicle speed updates based on average brake pressure.
- **Real‑time console output** – updated every 200 ms to avoid clutter.
- **CSV logging** – writes a complete history of the simulation for later analysis (e.g., in Excel, Python).
- **Configurable constants** – all key parameters are defined at the top of the source file for easy tuning.

---

## Requirements

- **C++17** compiler (e.g., `g++`, `clang++`).
- **CMake** (optional – can be compiled manually).
- **Standard C++ libraries** (no external dependencies).

---

## Building

### Using g++ directly
```bash
g++ -std=c++17 -O2 main.cpp -o abs_ecu_sim
```

## Running

Simply execute the compiled binary:
```bash
./abs_ecu_sim
```

The simulation will start with an initial vehicle speed of 30 m/s (~108 km/h) and will run until the vehicle comes to a stop (speed < 0.5 m/s). During the run, you will see console output every 200 ms and a CSV log file `abs_log.csv` will be created in the same directory.

---

## Architecture

The code is divided into several logical classes:

- **`BrakeActuator`** – models a hydraulic brake actuator. It stores the current pressure and a `BrakeState` (APPLY, HOLD, RELEASE). The `command()` method modifies pressure stepwise.
- **`WheelSpeedSensor`** – encapsulates a wheel speed sensor. It holds the true speed (from physics) and adds Gaussian noise when `read()` is called.
- **`Vehicle`** – simple vehicle model that updates its speed based on the average brake pressure and a maximum deceleration constant.
- **`ABSController`** – the ECU core. It manages four sensors and four actuators, estimates vehicle speed, calculates slip for each wheel, and issues brake commands. It also handles CSV logging.
- **`main()`** – orchestrates the real‑time simulation loop. It repeatedly calls `ABSController::control_cycle()` for the ECU logic and then updates the physics (wheel and vehicle speeds) for the next time step.

### Control Flow
1. `main()` runs a loop while vehicle speed > 0.5 m/s.
2. For each cycle:
   - `ABSController::control_cycle()` is called (ECU logic):
     - Reads all wheel sensors (with noise).
     - Estimates vehicle speed as average of wheel speeds.
     - For each wheel, calculates slip and decides next brake state.
     - Commands actuators accordingly.
     - Logs data to CSV.
   - Physics update:
     - For each wheel: compute deceleration due to brake pressure; apply recovery if state is RELEASE; clamp to vehicle speed.
     - Update vehicle speed using average pressure.
   - Sleep to maintain real‑time 20 ms cycle.
   - Print console output every 200 ms.

---

## Customization

All simulation parameters are defined as `constexpr` at the top of `abs_ecu_sim.cpp`:

| Constant                | Description                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| `DT`                    | Control cycle time (seconds). Default: 0.020 (20 ms).                       |
| `INITIAL_SPEED`         | Initial vehicle speed (m/s). Default: 30.0.                                 |
| `SLIP_THRESHOLD`        | Slip above which ABS releases pressure (e.g., 0.20 = 20%).                 |
| `LOW_SLIP_THRESHOLD`    | Slip below which ABS reapplies pressure (e.g., 0.05 = 5%).                 |
| `PRESSURE_STEP`         | Pressure change per cycle (%). Default: 10.0.                               |
| `MAX_PRESSURE`          | Maximum brake pressure (%). Default: 100.0.                                 |
| `MAX_WHEEL_DECEL`       | Maximum wheel deceleration when brake is fully applied (m/s²). Default: 25.0. |
| `RECOVERY_RATE`         | Wheel acceleration when brake is released (m/s²). Default: 15.0.            |
| `MAX_VEH_DECEL`         | Maximum vehicle deceleration (m/s²). Default: 8.0 (dry road).               |

You can also adjust the noise level inside the `WheelSpeedSensor::read()` method.

---

## Output

### Console Output
Every 200 ms the simulation prints:
- Simulation time.
- Current vehicle speed and estimated vehicle speed.
- For each wheel: measured speed, current brake pressure, and ABS state (APPLY, HOLD, RELEASE).

Example:
```
t = 0.20 s | Vehicle: 29.84 m/s | Est. Veh: 29.90 m/s
  Wheel 0: 29.82 m/s |  30.00% | APPLY 
  Wheel 1: 29.85 m/s |  30.00% | APPLY 
  Wheel 2: 29.83 m/s |  30.00% | APPLY 
  Wheel 3: 29.86 m/s |  30.00% | APPLY 
------------------------------------------------------------
```

### CSV Log File (`abs_log.csv`)
The log file contains the following columns:
- `Time(s)` – simulation time.
- `Veh_Speed` – true vehicle speed.
- `Est_Veh_Speed` – estimated vehicle speed (average of wheel sensor readings).
- `W0_Speed` … `W3_Speed` – true wheel speeds (before noise).
- `P0` … `P3` – brake pressures.
- `S0` … `S3` – ABS active flag per wheel (1 = RELEASE, 0 otherwise).
- `ABS_Active` – overall ABS active flag (1 if any wheel is releasing).

This file can be opened in spreadsheet software or analyzed with tools like Python (pandas, matplotlib) to visualize the ABS intervention.

---

## Example Console Output

```
============================================================
          ABS ECU SIMULATION STARTED (C++17)
          Initial speed: 30.00 m/s (~108 km/h)
          Cycle time: 20.00 ms
          Slip threshold: 20.00%
============================================================

t = 0.20 s | Vehicle: 29.84 m/s | Est. Veh: 29.90 m/s
  Wheel 0: 29.82 m/s |  30.00% | APPLY 
  Wheel 1: 29.85 m/s |  30.00% | APPLY 
  Wheel 2: 29.83 m/s |  30.00% | APPLY 
  Wheel 3: 29.86 m/s |  30.00% | APPLY 
------------------------------------------------------------
t = 0.40 s | Vehicle: 29.68 m/s | Est. Veh: 29.80 m/s
  Wheel 0: 29.64 m/s |  40.00% | APPLY 
  Wheel 1: 29.67 m/s |  40.00% | APPLY 
  Wheel 2: 29.65 m/s |  40.00% | APPLY 
  Wheel 3: 29.68 m/s |  40.00% | APPLY 
------------------------------------------------------------
...
t = 2.00 s | Vehicle: 23.45 m/s | Est. Veh: 21.10 m/s
  Wheel 0: 18.76 m/s |  80.00% | RELEASE
  Wheel 1: 18.80 m/s |  80.00% | RELEASE
  Wheel 2: 18.78 m/s |  80.00% | RELEASE
  Wheel 3: 18.82 m/s |  80.00% | RELEASE
------------------------------------------------------------
...
```

---

## Future Enhancements

The project is designed to be easily extended. Possible improvements include:

- **More advanced ABS algorithms** – PID control, adaptive target slip, road friction estimation.
- **Enhanced physics model** – load transfer, different road surfaces per wheel, tire relaxation length, separate vehicle longitudinal dynamics.
- **CAN bus simulation** – emulate communication between ECU and other vehicle modules.
- **Fault injection** – simulate sensor failures or actuator stuck conditions.
- **Graphical user interface** – real‑time plotting of speeds, pressures, and slip.
- **Hardware‑in‑the‑loop (HIL)** – interface with real ECU hardware via CAN/Ethernet.
- **Unit testing** – integrate Google Test or Catch2 for regression testing.
- **Multi‑threading** – separate sensor reading, control, and logging into parallel threads.

  
**Repository**: (https://github.com/ulvinamazow/embedded_abs_ecu.git)
