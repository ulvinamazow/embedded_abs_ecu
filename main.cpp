#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <algorithm>

#include "ABSController.hpp"

using namespace std;

constexpr double DT = 0.020;
constexpr double INITIAL_SPEED = 30.0;

constexpr double MAX_PRESSURE = 100.0;
constexpr double MAX_WHEEL_DECEL = 25.0;
constexpr double RECOVERY_RATE = 15.0;

int main()
{
    cout << "ABS ECU Simulation Started\n";

    Vehicle vehicle(INITIAL_SPEED);
    ABSController ecu(vehicle, INITIAL_SPEED);

    double sim_time = 0;

    while (vehicle.get_speed() > 0.5)
    {
        // ================= ECU CONTROL =================
        ecu.control_cycle(sim_time);

        // ================= PHYSICS UPDATE =================
        double avg_pressure = 0;

        auto& actuators = ecu.get_actuators();
        auto& sensors = ecu.get_sensors();

        for (int i = 0; i < 4; i++)
        {
            double p = actuators[i].get_pressure();

            avg_pressure += p;

            double wheel_decel =
                (p / MAX_PRESSURE) * MAX_WHEEL_DECEL;

            double new_speed =
                sensors[i].get_true_speed()
                - wheel_decel * DT;

            if (actuators[i].get_state()
                == BrakeState::RELEASE)
            {
                new_speed += RECOVERY_RATE * DT;
            }

            new_speed =
                clamp(new_speed,
                      0.0,
                      vehicle.get_speed() * 1.02);

            sensors[i].update(new_speed);
        }

        avg_pressure /= 4.0;

        vehicle.update(avg_pressure, DT);

        sim_time += DT;

        this_thread::sleep_for(
            chrono::milliseconds(20)
        );

        // ================= TERMINAL OUTPUT =================
        cout << fixed << setprecision(2);

        cout << "t = " << sim_time
             << " s | Vehicle: "
             << vehicle.get_speed()
             << " m/s | Est: "
             << ecu.estimate_vehicle_speed()
             << " m/s\n";

        for (int i = 0; i < 4; i++)
        {
            string state_str;

            switch (actuators[i].get_state())
            {
                case BrakeState::APPLY:
                    state_str = "APPLY";
                    break;

                case BrakeState::HOLD:
                    state_str = "HOLD";
                    break;

                case BrakeState::RELEASE:
                    state_str = "RELEASE";
                    break;
            }

            cout << "Wheel " << i
                 << ": "
                 << sensors[i].read()
                 << " m/s | "
                 << actuators[i].get_pressure()
                 << "% | "
                 << state_str
                 << "\n";
        }

        cout << "---------------------------\n";
    }

    cout << "\nSimulation finished\n";

    return 0;
}