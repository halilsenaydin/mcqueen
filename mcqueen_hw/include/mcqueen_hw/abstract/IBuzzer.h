#ifndef IBUZZER_H
#define IBUZZER_H

class IBuzzer {
public:
    virtual ~IBuzzer();

    virtual void on() = 0;
    virtual void off() = 0;
};

#endif // IBUZZER_H
