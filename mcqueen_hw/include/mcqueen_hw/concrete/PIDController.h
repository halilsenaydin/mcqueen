#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "mcqueen_hw/abstract/IController.h"

class PIDController : public IController {
public:
    explicit PIDController(double kp, double ki, double kd, double dt);
    ~PIDController();

    double calculate(double setpoint, double measured) override;
    void reset() override;

private:
    double _kp;
    double _ki;
    double _kd;
    double _dt;

    double _prevError;
    double _integral;
};

#endif // PID_CONTROLLER_H
