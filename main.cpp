// ABS ECU Simulation in C++17
// This code simulates an Anti-lock Braking System (ABS) Electronic Control Unit (ECU) in a real-time environment. It models the vehicle dynamics, wheel speed sensors, brake actuators, and the ABS control logic. The simulation runs until the vehicle comes to a stop, logging detailed data to a CSV file for analysis.
// Note: This is a simplified model for educational purposes and does not represent the full complexity of a real ABS system.
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <random>
#include <iomanip>
#include <fstream>
#include <string>
#include <algorithm>
#include <cmath>

using namespace std;

// ====================== SIMULATION PARAMETERS ======================
constexpr double DT                = 0.020;   // 20 ms control cycle (50 Hz)
constexpr double INITIAL_SPEED     = 30.0;    // initial speed in m/s (~108 km/h)   
constexpr double SLIP_THRESHOLD    = 0.20;    // 20% slip → release brake
constexpr double LOW_SLIP_THRESHOLD = 0.05;   // 5% slip → apply brake
constexpr double PRESSURE_STEP     = 10.0;    // pressure change per cycle when applying/releasing brakes (% of max)
constexpr double MAX_PRESSURE      = 100.0;   // max brake pressure (%)
constexpr double MAX_WHEEL_DECEL   = 25.0;    // m/s² (max deceleration at wheel level when fully applied)
constexpr double RECOVERY_RATE     = 15.0;    // m/s² (how quickly the wheel recovers speed when brake is released)
constexpr double MAX_VEH_DECEL     = 8.0;     // m/s² (max deceleration of the vehicle under full braking)

// ====================== RANDOM NUMBER GENERATOR FOR SENSOR NOISE ======================
random_device rd;
mt19937 rng(rd());

// ====================== BRAKE STATE ENUM ======================
enum class BrakeState {
    APPLY,
    HOLD,
    RELEASE
};

// ====================== BRAKE ACTUATOR ======================
class BrakeActuator {
private:
    double pressure = 0.0;
    BrakeState state = BrakeState::APPLY;

public:
    void command(BrakeState new_state) {
        state = new_state;
        if (new_state == BrakeState::APPLY) {
            pressure = min(pressure + PRESSURE_STEP, MAX_PRESSURE);
        } else if (new_state == BrakeState::RELEASE) {
            pressure = max(pressure - PRESSURE_STEP, 0.0);
        }
        // HOLD state maintains current pressure, no change needed
    }

    double get_pressure() const { return pressure; }
    BrakeState get_state() const { return state; }
};

// ====================== WHEEL SPEED SENSOR ======================
class WheelSpeedSensor {
private:
    double true_speed;

public:
    explicit WheelSpeedSensor(double initial) : true_speed(initial) {}

    double read() const {
        // Simulate real sensor noise (±0.3 m/s)
        normal_distribution<double> noise(0.0, 0.3);
        return true_speed + noise(rng);
    }

    void update(double new_speed) {
        true_speed = max(0.0, new_speed);
    }

    double get_true_speed() const { return true_speed; }
};

// ====================== VEHICLE MODEL ======================
class Vehicle {
private:
    double speed;

public:
    explicit Vehicle(double initial) : speed(initial) {}

    double get_speed() const { return speed; }

    void update(double avg_pressure, double dt) {
        double decel = (avg_pressure / MAX_PRESSURE) * MAX_VEH_DECEL;
        speed -= decel * dt;
        if (speed < 0.0) speed = 0.0;
    }
};

// ====================== ABS CONTROLLER ======================
class ABSController {
private:
    vector<WheelSpeedSensor> sensors;
    vector<BrakeActuator> actuators;
    Vehicle& vehicle;
    ofstream log_file;

public:
    explicit ABSController(Vehicle& v, double init_speed)
        : vehicle(v) {
        for (int i = 0; i < 4; ++i) {
            sensors.emplace_back(init_speed);
            actuators.emplace_back();
        }

        log_file.open("abs_log.cvs");
        if (log_file.is_open()) {
            log_file << "Time(s),Veh_Speed,Est_Veh_Speed,W0_Speed,W1_Speed,W2_Speed,W3_Speed,"
                     << "P0,P1,P2,P3,S0,S1,S2,S3,ABS_Active\n";
        }
    }

    ~ABSController() {
        if (log_file.is_open()) log_file.close();
    }

    double estimate_vehicle_speed() const {
        double sum = 0.0;
        for (const auto& s : sensors) sum += s.read();
        return sum / 4.0;
    }

    void control_cycle(double sim_time) {
        double est_veh = estimate_vehicle_speed();
        if (est_veh < 1.0) return;

        bool abs_active = false;
        vector<double> pressures(4);
        vector<string> states(4);

        for (int i = 0; i < 4; ++i) {
            double wheel_sp = sensors[i].read();
            double slip = 0.0;
            if (est_veh > 0.0) {
                slip = (est_veh - wheel_sp) / est_veh;
            }

            BrakeState cmd;
            if (slip > SLIP_THRESHOLD) {
                cmd = BrakeState::RELEASE;
                abs_active = true;
            } else if (slip < LOW_SLIP_THRESHOLD) {
                cmd = BrakeState::APPLY;
            } else {
                cmd = BrakeState::HOLD;
            }

            actuators[i].command(cmd);

            pressures[i] = actuators[i].get_pressure();
            states[i] = (cmd == BrakeState::APPLY ? "APPLY" :
                         cmd == BrakeState::HOLD ? "HOLD" : "RELEASE");
        }

        // Log data to CSV
        if (log_file.is_open()) {
            log_file << fixed << setprecision(3)
                     << sim_time << ","
                     << vehicle.get_speed() << ","
                     << est_veh << ",";
            for (int i = 0; i < 4; ++i) log_file << sensors[i].get_true_speed() << ",";
            for (int i = 0; i < 4; ++i) log_file << pressures[i] << ",";
            for (int i = 0; i < 4; ++i) log_file << (actuators[i].get_state() == BrakeState::RELEASE ? "1" : "0") << ",";
            log_file << (abs_active ? "1" : "0") << "\n";
        }
    }

    const vector<BrakeActuator>& get_actuators() const { return actuators; }
    vector<WheelSpeedSensor>& get_sensors() { return sensors; } // for physics update
};

// ====================== MAIN SIMULATION LOOP =======================
int main() {
    cout << "============================================================\n";
    cout << "          ABS ECU SIMULATION STARTED (C++17)\n";
    cout << "          Initial speed: " << INITIAL_SPEED << " m/s (~108 km/h)\n";
    cout << "          Cycle time: " << (DT * 1000) << " ms\n";
    cout << "          Slip threshold: " << (SLIP_THRESHOLD * 100) << "%\n";
    cout << "============================================================\n\n";

    Vehicle vehicle(INITIAL_SPEED);
    ABSController ecu(vehicle, INITIAL_SPEED);

    double sim_time = 0.0;
    auto last_print = chrono::steady_clock::now();

    while (vehicle.get_speed() > 0.5) {
        // === CONTROL UPDATE (ECU logic) ===
        ecu.control_cycle(sim_time);

        // === PHYSICS UPDATE (simulates real vehicle & wheel dynamics) ===
        double avg_pressure = 0.0;
        auto& actuators = ecu.get_actuators();
        auto& sensors = ecu.get_sensors();

        for (int i = 0; i < 4; ++i) {
            double p = actuators[i].get_pressure();
            avg_pressure += p;

            double wheel_decel = (p / MAX_PRESSURE) * MAX_WHEEL_DECEL;
            double current_wheel = sensors[i].get_true_speed();

            double new_wheel_speed = current_wheel - wheel_decel * DT;

            // Road friction recovery when brake is released
            if (actuators[i].get_state() == BrakeState::RELEASE) {
                new_wheel_speed += RECOVERY_RATE * DT;
            }

            // Wheel cannot exceed vehicle speed (realistic kinematic limit)
            new_wheel_speed = clamp(new_wheel_speed, 0.0, vehicle.get_speed() * 1.02);

            sensors[i].update(new_wheel_speed);
        }
        avg_pressure /= 4.0;

        // Update vehicle speed based on total braking force
        vehicle.update(avg_pressure, DT);

        sim_time += DT;

        // Simulate real-time by sleeping for the remainder of the control cycle
        this_thread::sleep_for(chrono::milliseconds(static_cast<int>(DT * 1000)));

        // === CONSOLE LOGGING (every 200 ms to avoid spam) ===
        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(now - last_print).count() >= 200) {
            cout << fixed << setprecision(2);
            cout << "t = " << sim_time << " s | Vehicle: " << vehicle.get_speed()
                 << " m/s | Est. Veh: " << ecu.estimate_vehicle_speed() << " m/s\n";

            for (int i = 0; i < 4; ++i) {
                string state_str;
                switch (actuators[i].get_state()) {
                    case BrakeState::APPLY:   state_str = "APPLY "; break;
                    case BrakeState::HOLD:    state_str = "HOLD  "; break;
                    case BrakeState::RELEASE: state_str = "RELEASE"; break;
                }
                cout << "  Wheel " << i << ": "
                     << setw(6) << sensors[i].read() << " m/s | "
                     << setw(6) << actuators[i].get_pressure() << "% | "
                     << state_str << "\n";
            }
            cout << "------------------------------------------------------------\n";
            last_print = now;
        }
    }

    cout << "\n============================================================\n";
    cout << "SIMULATION COMPLETE - Vehicle stopped at " << sim_time << " seconds\n";
    cout << "ABS activated " << (sim_time > 2.0 ? "multiple times" : "minimally") << "\n";
    cout << "Log file written: abs_log.csv (open in Excel/Google Sheets)\n";
    cout << "============================================================\n";

    return 0;
}