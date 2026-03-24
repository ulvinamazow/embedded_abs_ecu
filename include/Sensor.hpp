#pragma once

class WheelSpeedSensor {
private:
    double true_speed;

public:
    explicit WheelSpeedSensor(double initial);

    double read() const;
    void update(double new_speed);
    double get_true_speed() const;
};