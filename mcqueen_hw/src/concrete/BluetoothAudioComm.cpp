#include "mcqueen_hw/concrete/BluetoothAudioComm.h"

#include <cstdlib>
#include <iostream>
#include <sstream>

BluetoothAudioComm::BluetoothAudioComm(const std::string& macAddress)
    : _macAddress(macAddress) {}

BluetoothAudioComm::~BluetoothAudioComm() {}

bool BluetoothAudioComm::open() {
    runBluetoothctlCommand("pair " + _macAddress);
    runBluetoothctlCommand("trust " + _macAddress);

    return runBluetoothctlCommand("connect " + _macAddress);
}

void BluetoothAudioComm::close() {
    runBluetoothctlCommand("disconnect " + _macAddress);
}

bool BluetoothAudioComm::write(const std::string& data, const std::string& target) {
    if (!isConnected()) {
        std::cerr << "Bluetooth speaker not connected.\n";
        return false;
    }

    std::ostringstream cmd;
    cmd << "mpg123 \"" << data << "\"";

    return std::system(cmd.str().c_str()) == 0;
}

std::string BluetoothAudioComm::read(size_t maxBytes, const std::string& target) {
    // No need
    return "";
}

bool BluetoothAudioComm::runBluetoothctlCommand(const std::string& command) {
    std::ostringstream cmd;
    cmd << "echo -e '" << command << "' | bluetoothctl";

    return std::system(cmd.str().c_str()) == 0;
}

std::string BluetoothAudioComm::execCommand(const std::string& cmd) {
    std::string result;
    char buffer[128];

    FILE* pipe = popen(cmd.c_str(), "r");

    if (!pipe) return "ERROR";

    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    pclose(pipe);

    return result;
}

bool BluetoothAudioComm::isConnected() {
    std::string output = execCommand("bluetoothctl info " + _macAddress + " | grep Connected");
    
    return output.find("yes") != std::string::npos;
}
