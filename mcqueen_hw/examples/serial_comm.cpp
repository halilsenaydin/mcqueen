#include "mcqueen_hw/abstract/ICommDevice.h"
#include "mcqueen_hw/concrete/SerialComm.h"

#include <iostream>
#include <memory>

int main() {
    std::string portName = "/dev/cu.usbserial-12345"; 
    unsigned int baudRate = 9600;

    std::unique_ptr<ICommDevice> comm = std::make_unique<SerialComm>(portName, baudRate);

    if (!comm->open()) {
        std::cerr << "Failed to open serial port\n";
        return 1;
    }

    // Example message to send
    std::string messageToSend = "Hello Serial Device!\n";

    if (!comm->write(messageToSend)) {
        std::cerr << "Failed to write to serial port\n";
    } else {
        std::cout << "Sent: " << messageToSend;
    }

    // Read response from the device
    std::string received = comm->read(256);

    if (!received.empty()) {
        std::cout << "Received: " << received << "\n";
    } else {
        std::cout << "No data received (timeout or empty)\n";
    }

    return 0;
}
