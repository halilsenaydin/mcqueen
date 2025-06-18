#include "mcqueen_hw/concrete/PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, double dt)
: _kp(kp), _ki(ki), _kd(kd), _dt(dt), _prevError(0.0), _integral(0.0)
{}

PIDController::~PIDController() = default;

double PIDController::calculate(double setpoint, double measured)
{
    double error = setpoint - measured;
    double derivative = (error - _prevError) / _dt;

    _integral += error * _dt;
    _prevError = error;

    return _kp * error + _ki * _integral + _kd * derivative;
}

void PIDController::reset()
{
    _prevError = 0.0;
    _integral = 0.0;
}
