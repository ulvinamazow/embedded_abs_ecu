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
- **Physics update** – wheel speed changes due to brake pressure and road friction; vehicle speed updates based on average pressure.
- **Real‑time console output** – updated every 200 ms to avoid clutter.
- **CSV logging** – writes a complete history of the simulation for later analysis.
- **Configurable constants** – all key parameters defined as `constexpr` for easy tuning.

---

## Requirements

- **C++17** compiler (e.g., `g++`, `clang++`).
- **Standard C++ libraries** (no external dependencies).

---

### Simulation Results

Vehicle speed over time:  
![Vehicle Speed](output/vehicle_speed.png)

Brake pressures per wheel:  
![Brake Pressure](output/brake_pressure.png)

ABS activation timeline:  
![ABS Active](output/abs_active_timeline.png)

---

## Building

### Using g++ directly
```bash
g++ main.cpp src/*.cpp -Iinclude -std=c++17 -O2 -Wall -o abs_ecu_sim


Using CMake (optional)
Create CMakeLists.txt in the project root.
Configure and build using standard CMake workflow.
Running

Simply execute the compiled binary:

./abs_ecu_sim

The simulation starts with an initial vehicle speed of 30 m/s (~108 km/h) and runs until the vehicle stops (speed < 0.5 m/s). During the run, the console updates every 200 ms, and a CSV log file abs_log.csv is created in the project directory.

CSV Log File (abs_log.csv)

The CSV contains the following columns:

Column	Description
Time(s)	Simulation time in seconds.
Veh_Speed	True vehicle speed.
Est_Veh_Speed	Estimated vehicle speed (average of wheel sensors).
W0_Speed … W3_Speed	True wheel speeds (before noise).
P0 … P3	Brake pressures for each wheel.
S0 … S3	Wheel ABS active flag (1 = RELEASE, 0 = APPLY/HOLD).
ABS_Active	Overall ABS active flag (1 if any wheel is releasing).

Example CSV snippet:

Time(s),Veh_Speed,Est_Veh_Speed,W0_Speed,W1_Speed,W2_Speed,W3_Speed,P0,P1,P2,P3,S0,S1,S2,S3,ABS_Active
0.000,30.000,29.992,30.000,30.000,30.000,30.000,10.000,10.000,10.000,10.000,0,0,0,0,0
0.020,29.984,29.879,29.950,29.950,29.950,29.950,20.000,20.000,20.000,20.000,0,0,0,0,0
Console Output

Every 200 ms, the simulation prints:

Simulation time.
Vehicle speed and estimated vehicle speed.
For each wheel: measured speed, brake pressure, and ABS state (APPLY, HOLD, RELEASE).

Example:

t = 0.20 s | Vehicle: 29.84 m/s | Est. Veh: 29.90 m/s
  Wheel 0: 29.82 m/s |  30.00% | APPLY 
  Wheel 1: 29.85 m/s |  30.00% | APPLY 
  Wheel 2: 29.83 m/s |  30.00% | APPLY 
  Wheel 3: 29.86 m/s |  30.00% | APPLY 
------------------------------------------------------------
t = 2.00 s | Vehicle: 23.45 m/s | Est. Veh: 21.10 m/s
  Wheel 0: 18.76 m/s |  80.00% | RELEASE
  Wheel 1: 18.80 m/s |  80.00% | RELEASE
  Wheel 2: 18.78 m/s |  80.00% | RELEASE
  Wheel 3: 18.82 m/s |  80.00% | RELEASE
------------------------------------------------------------
Architecture
BrakeActuator – models hydraulic brake actuator with current pressure and BrakeState (APPLY, HOLD, RELEASE).
WheelSpeedSensor – models a wheel speed sensor, applies Gaussian noise on read().
Vehicle – updates vehicle speed based on average brake pressure.
ABSController – manages sensors and actuators, estimates speed, calculates slip, commands brakes, and logs CSV.
main() – runs real‑time loop: ECU logic, physics update, and terminal logging.
Simulation Parameters (constexpr)
Constant	Description
DT	Control cycle (seconds). Default: 0.020.
INITIAL_SPEED	Initial vehicle speed (m/s). Default: 30.0.
SLIP_THRESHOLD	Slip above which ABS releases pressure. Default: 0.20 (20%).
LOW_SLIP_THRESHOLD	Slip below which ABS reapplies pressure. Default: 0.05 (5%).
PRESSURE_STEP	Pressure change per cycle (%). Default: 10.0.
MAX_PRESSURE	Maximum brake pressure (%). Default: 100.0.
MAX_WHEEL_DECEL	Max wheel deceleration (m/s²). Default: 25.0.
RECOVERY_RATE	Wheel recovery rate when brake is released (m/s²). Default: 15.0.
MAX_VEH_DECEL	Max vehicle deceleration (m/s²). Default: 8.0.
Future Enhancements
Advanced ABS algorithms (PID, adaptive target slip).
Enhanced physics (load transfer, tire relaxation, per-wheel road friction).
CAN bus simulation.
Sensor or actuator fault injection.
Graphical real-time visualization.
Hardware-in-the-loop (HIL) testing.
Unit testing with Google Test or Catch2.
Multi-threading for separate sensor, control, and logging threads.

Repository: https://github.com/ulvinamazow/embedded_abs_ecu