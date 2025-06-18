#include "mcqueen_hw/concrete/BluetoothComm.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <cstring>
#include <iostream>
#include <unistd.h>

BluetoothComm::BluetoothComm(const std::string& address, uint8_t channel)
    : _address(address), _channel(channel), _sock(-1) {}

BluetoothComm::~BluetoothComm() {
    close();
}

bool BluetoothComm::open() {
    struct sockaddr_rc addr {};
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = 1;
    str2ba(_address.c_str(), &addr.rc_bdaddr);
    _sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    if (_sock < 0) {
        std::cerr << "Failed to create Bluetooth socket\n";
        return false;
    }

    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = this->_channel;
    str2ba(_address.c_str(), &addr.rc_bdaddr);

    std::cout << "Connecting to Bluetooth device " << _address << "...\n";

    if (::connect(_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Bluetooth connection failed: " << strerror(errno) << "\n";
        ::close(_sock);
        _sock = -1;

        return false;
    }

    std::cout << "Bluetooth connection established.\n";
    return true;
}

void BluetoothComm::close() {
    if (_sock >= 0) {
        ::close(_sock);
        _sock = -1;
    }
}

bool BluetoothComm::write(const std::string& data, const std::string& target) {
    if (_sock < 0) return false;

    ssize_t sent = ::send(_sock, data.c_str(), data.size(), 0);

    return sent == (ssize_t)data.size();
}

std::string BluetoothComm::read(size_t maxBytes, const std::string& target) {
    if (_sock < 0) return "";

    char buffer[1024] = {0};
    ssize_t received = ::recv(_sock, buffer, std::min(maxBytes, sizeof(buffer)), 0);

    if (received > 0) {
        return std::string(buffer, received);
    }
    
    return "";
}
