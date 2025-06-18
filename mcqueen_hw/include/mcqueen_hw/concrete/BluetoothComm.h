#ifndef BLUETOOTH_COMM_H
#define BLUETOOTH_COMM_H

#include "mcqueen_hw/abstract/ICommDevice.h"

#include <cstdint>
#include <string>

class BluetoothComm : public ICommDevice {
public:
    explicit BluetoothComm(const std::string& address, uint8_t channel = 1);
    ~BluetoothComm();

    bool open() override;
    void close() override;
    bool write(const std::string& data, const std::string& target = "") override;
    std::string read(size_t maxBytes = 256, const std::string& target = "") override;

private:
    std::string _address;
    uint8_t _channel;
    int _sock;
};

#endif // BLUETOOTH_COMM_H
