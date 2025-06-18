#ifndef BLUETOOTH_AUDIO_COMM_H
#define BLUETOOTH_AUDIO_COMM_H

#include "mcqueen_hw/abstract/ICommDevice.h"

#include <string>

class BluetoothAudioComm : public ICommDevice {
public:
    explicit BluetoothAudioComm(const std::string& macAddress);
    ~BluetoothAudioComm();

    bool open() override;
    void close() override;
    bool write(const std::string& data, const std::string& target = "") override;
    std::string read(size_t maxBytes = 256, const std::string& target = "") override;

private:
    std::string _macAddress;

    bool runBluetoothctlCommand(const std::string& command);
    std::string execCommand(const std::string& cmd);
    bool isConnected();
};

#endif // BLUETOOTH_AUDIO_COMM_H
