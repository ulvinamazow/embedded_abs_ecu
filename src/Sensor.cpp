#include "Sensor.hpp"
#include <random>
#include <algorithm>

using namespace std;

static random_device rd;
static mt19937 rng(rd());

WheelSpeedSensor::WheelSpeedSensor(double initial)
    : true_speed(initial) {}

double WheelSpeedSensor::read() const {

    normal_distribution<double> noise(0.0, 0.3);
    return true_speed + noise(rng);
}

void WheelSpeedSensor::update(double new_speed) {

    true_speed = max(0.0, new_speed);
}

double WheelSpeedSensor::get_true_speed() const {

    return true_speed;
}