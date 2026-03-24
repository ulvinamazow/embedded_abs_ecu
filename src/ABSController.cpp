#include "ABSController.hpp"
#include <iomanip>
#include <algorithm>

using namespace std;

constexpr double SLIP_THRESHOLD = 0.20;
constexpr double LOW_SLIP_THRESHOLD = 0.05;

constexpr double MAX_PRESSURE = 100.0;
constexpr double MAX_VEH_DECEL = 8.0;

Vehicle::Vehicle(double initial)
    : speed(initial) {}

double Vehicle::get_speed() const {

    return speed;
}

void Vehicle::update(double avg_pressure, double dt) {

    double decel = (avg_pressure / MAX_PRESSURE) * MAX_VEH_DECEL;

    speed -= decel * dt;

    if (speed < 0.0)
        speed = 0.0;
}

ABSController::ABSController(Vehicle& v, double init_speed)
    : vehicle(v) {

    for (int i = 0; i < 4; i++) {

        sensors.emplace_back(init_speed);
        actuators.emplace_back();
    }

    log_file.open("logs/abs_log.csv");

    if (log_file.is_open()) {
    log_file << "Time(s),Veh_Speed,Est_Veh_Speed,"
             << "W0_Speed,W1_Speed,W2_Speed,W3_Speed,"
             << "P0,P1,P2,P3,ABS_Active\n";
    }



}

ABSController::~ABSController() {

    if (log_file.is_open())
        log_file.close();
}

double ABSController::estimate_vehicle_speed() const {

    double sum = 0;

    for (const auto& s : sensors)
        sum += s.read();

    return sum / 4.0;
}

void ABSController::control_cycle(double sim_time)
{
    double est_veh = estimate_vehicle_speed();

    if (est_veh < 1.0)
        return;

    bool abs_active = false;

    vector<double> pressures(4);

    for (int i = 0; i < 4; i++)
    {
        double wheel_sp = sensors[i].read();

        double slip = (est_veh - wheel_sp) / est_veh;

        BrakeState cmd;

        if (slip > SLIP_THRESHOLD)
        {
            cmd = BrakeState::RELEASE;
            abs_active = true;
        }
        else if (slip < LOW_SLIP_THRESHOLD)
            cmd = BrakeState::APPLY;
        else
            cmd = BrakeState::HOLD;

        actuators[i].command(cmd);

        pressures[i] = actuators[i].get_pressure();
    }

    // CSV logging
    if (log_file.is_open())
    {
        log_file << fixed << setprecision(3)
                 << sim_time << ","
                 << vehicle.get_speed() << ","
                 << est_veh << ",";

        for (int i = 0; i < 4; i++)
            log_file << sensors[i].get_true_speed() << ",";

        for (int i = 0; i < 4; i++)
            log_file << pressures[i] << ",";

        log_file << (abs_active ? "1" : "0") << "\n";
    }
}

const vector<BrakeActuator>& ABSController::get_actuators() const {

    return actuators;
}

vector<WheelSpeedSensor>& ABSController::get_sensors() {

    return sensors;
}