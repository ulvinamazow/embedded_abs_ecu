````markdown
# ABS ECU Simulation

A realistic, modular, C++17 simulation of an Anti-lock Braking System (ABS) Electronic Control Unit (ECU) for automotive embedded systems. The project demonstrates a closed‑loop control system with sensor input, ABS logic, and actuator output, running in a 20 ms real-time loop.

---

## Overview

This project simulates an ABS ECU that:

- Monitors four wheel speeds
- Estimates vehicle speed
- Calculates wheel slip
- Modulates brake pressure to prevent wheel lock

Design principles:

- Modular OOP architecture (sensor → control → actuator)
- Real-time loop: 20 ms cycle using `std::chrono` and `std::thread`
- Slip-ratio-based control: APPLY / HOLD / RELEASE
- Simplified physics
- Gaussian sensor noise
- CSV logging

---

## Features

- Four independent wheel sensors with individual noise
- Vehicle speed estimation (average of wheel speeds)
- Slip calculation: `(V_veh - V_wheel)/V_veh`
- ABS control logic
- Pressure ramping (±10% per cycle)
- Physics update
- Real-time console output every 200 ms
- CSV logging
- Configurable constants (`constexpr`)

---

## Building

### Using g++
```bash
g++ main.cpp src/*.cpp -Iinclude -std=c++17 -O2 -Wall -o abs_ecu_sim
./abs_ecu_sim
````

### Using CMake (optional)

* Create `CMakeLists.txt` in the project root
* Configure and build using standard CMake workflow

---

## Running

* Starts at 30 m/s (~108 km/h)
* Updates console every 200 ms
* Generates `abs_log.csv` in project directory

---

## CSV Log (`abs_log.csv`)

| Column              | Description               |
| ------------------- | ------------------------- |
| Time(s)             | Simulation time           |
| Veh_Speed           | True vehicle speed        |
| Est_Veh_Speed       | Estimated vehicle speed   |
| W0_Speed … W3_Speed | Wheel speeds before noise |
| P0 … P3             | Brake pressures per wheel |
| S0 … S3             | Wheel ABS active flags    |
| ABS_Active          | Overall ABS flag          |

**Example:**

```csv
Time(s),Veh_Speed,Est_Veh_Speed,W0_Speed,W1_Speed,W2_Speed,W3_Speed,P0,P1,P2,P3,S0,S1,S2,S3,ABS_Active
0.000,30.000,29.992,30.000,30.000,30.000,30.000,10.000,10.000,10.000,10.000,0,0,0,0,0
0.020,29.984,29.879,29.950,29.950,29.950,29.950,20.000,20.000,20.000,20.000,0,0,0,0,0
```

---

## Console Output (every 200 ms)

```text
t = 0.20 s | Vehicle: 29.84 m/s | Est. Veh: 29.90 m/s
  Wheel 0: 29.82 m/s | 30.00% | APPLY
  Wheel 1: 29.85 m/s | 30.00% | APPLY
  Wheel 2: 29.83 m/s | 30.00% | APPLY
  Wheel 3: 29.86 m/s | 30.00% | APPLY
------------------------------------------------------------
t = 2.00 s | Vehicle: 23.45 m/s | Est. Veh: 21.10 m/s
  Wheel 0: 18.76 m/s | 80.00% | RELEASE
  Wheel 1: 18.80 m/s | 80.00% | RELEASE
  Wheel 2: 18.78 m/s | 80.00% | RELEASE
  Wheel 3: 18.82 m/s | 80.00% | RELEASE
```

---

## Architecture

* **BrakeActuator** – hydraulic actuator with BrakeState (APPLY/HOLD/RELEASE)
* **WheelSpeedSensor** – wheel speed sensor with Gaussian noise
* **Vehicle** – updates vehicle speed based on brake pressures
* **ABSController** – manages sensors/actuators, calculates slip, commands brakes, logs CSV
* **main()** – real-time loop: ECU logic, physics update, console logging

---

## Simulation Parameters (`constexpr`)

| Constant           | Description                                                  |
| ------------------ | ------------------------------------------------------------ |
| DT                 | Control cycle (s), default 0.020                             |
| INITIAL_SPEED      | Start speed (m/s), default 30.0                              |
| SLIP_THRESHOLD     | Slip above which ABS releases pressure, default 0.20         |
| LOW_SLIP_THRESHOLD | Slip below which ABS reapplies pressure, default 0.05        |
| PRESSURE_STEP      | Pressure change per cycle (%), default 10.0                  |
| MAX_PRESSURE       | Max brake pressure (%), default 100.0                        |
| MAX_WHEEL_DECEL    | Max wheel deceleration (m/s²), default 25.0                  |
| RECOVERY_RATE      | Wheel recovery rate when brake released (m/s²), default 15.0 |
| MAX_VEH_DECEL      | Max vehicle deceleration (m/s²), default 8.0                 |

---

## Simulation Results

Vehicle speed over time:
![Vehicle Speed](figures/vehicle_speed.png)

Brake pressures per wheel:
![Brake Pressure](figures/brake_pressure.png)

ABS activation timeline:
![ABS Active](figures/abs_active_timeline.png)

---

## Future Enhancements

* Advanced ABS algorithms (PID, adaptive slip)
* Enhanced physics (load transfer, per-wheel friction)
* CAN bus simulation
* Sensor/actuator fault injection
* Real-time visualization
* HIL testing
* Unit testing with Google Test or Catch2
* Multi-threaded sensor/control/logging threads

---

## Repository

[https://github.com/ulvinamazow/embedded_abs_ecu](https://github.com/ulvinamazow/embedded_abs_ecu)
