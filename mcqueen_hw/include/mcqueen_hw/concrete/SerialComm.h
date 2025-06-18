#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "mcqueen_hw/abstract/ICommDevice.h"

#include <string>

class SerialComm : public ICommDevice {
public:
    explicit SerialComm(const std::string& portName, unsigned int baudRate = 9600);
    ~SerialComm();

    bool open() override;
    void close() override;
    bool write(const std::string& data, const std::string& target = "") override;
    std::string read(size_t maxBytes = 256, const std::string& target = "") override;

private:
    std::string _portName;
    unsigned int _baudRate;
    int _serialFd;

    bool configurePort();
};

#endif // SERIAL_COMM_H
