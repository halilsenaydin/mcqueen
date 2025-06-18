#ifndef ICONTROLLER_H
#define ICONTROLLER_H

class IController {
public:
    virtual ~IController();

    virtual double calculate(double setpoint, double measured) = 0;
    virtual void reset() = 0;
};

#endif // ICONTROLLER_H
