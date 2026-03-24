#include "Wheel.hpp"
#include <algorithm>

using namespace std;

constexpr double PRESSURE_STEP = 10.0;
constexpr double MAX_PRESSURE  = 100.0;

BrakeActuator::BrakeActuator()
    : pressure(0.0), state(BrakeState::APPLY) {}

void BrakeActuator::command(BrakeState new_state) {

    state = new_state;

    if (new_state == BrakeState::APPLY)
        pressure = min(pressure + PRESSURE_STEP, MAX_PRESSURE);

    else if (new_state == BrakeState::RELEASE)
        pressure = max(pressure - PRESSURE_STEP, 0.0);
}

double BrakeActuator::get_pressure() const {

    return pressure;
}

BrakeState BrakeActuator::get_state() const {

    return state;
}