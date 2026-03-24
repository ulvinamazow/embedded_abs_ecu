#pragma once

#include <vector>
#include <fstream>
#include "Sensor.hpp"
#include "Wheel.hpp"

class Vehicle {

private:
    double speed;

public:
    explicit Vehicle(double initial);

    double get_speed() const;

    void update(double avg_pressure, double dt);
};

class ABSController {

private:
    std::vector<WheelSpeedSensor> sensors;
    std::vector<BrakeActuator> actuators;

    Vehicle& vehicle;

    std::ofstream log_file;

public:
    ABSController(Vehicle& v, double init_speed);

    ~ABSController();

    double estimate_vehicle_speed() const;

    void control_cycle(double sim_time);

    const std::vector<BrakeActuator>& get_actuators() const;

    std::vector<WheelSpeedSensor>& get_sensors();
};