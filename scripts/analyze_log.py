# analyze_abs_log.py
# Python 3.x required
# Install dependencies: pip install pandas matplotlib

import pandas as pd
import matplotlib.pyplot as plt

log_file = "abs_log.csv"
df = pd.read_csv(log_file)

time = df["Time(s)"]
veh_speed = df["Veh_Speed"]
est_speed = df["Est_Veh_Speed"]
pressures = df[["P0", "P1", "P2", "P3"]]
abs_active = df["ABS_Active"]

print("=== Vehicle Speed Statistics ===")
print(df["Veh_Speed"].describe())
print("\n=== ABS Active Summary ===")
print(f"ABS activated {abs_active.sum()} times out of {len(abs_active)} cycles")
print(f"Activation percentage: {100 * abs_active.sum() / len(abs_active):.2f}%")

plt.figure(figsize=(12,6))
plt.plot(time, veh_speed, label="Vehicle Speed (true)")
plt.plot(time, est_speed, label="Estimated Vehicle Speed", linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Vehicle Speed vs Time")
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(12,6))
for i in range(4):
    plt.plot(time, pressures[f"P{i}"], label=f"Wheel {i} Brake Pressure")
plt.xlabel("Time (s)")
plt.ylabel("Brake Pressure (%)")
plt.title("Brake Pressure vs Time")
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(12,3))
plt.plot(time, abs_active, label="ABS Active (1=Yes)", color='red')
plt.xlabel("Time (s)")
plt.ylabel("ABS Active")
plt.title("ABS Activation Timeline")
plt.grid(True)
plt.show()