#pragma once

enum class BrakeState {
    APPLY,
    HOLD,
    RELEASE
};

class BrakeActuator {

private:
    double pressure;
    BrakeState state;

public:
    BrakeActuator();

    void command(BrakeState new_state);

    double get_pressure() const;
    BrakeState get_state() const;
};