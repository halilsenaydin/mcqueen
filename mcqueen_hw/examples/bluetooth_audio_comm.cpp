#include "mcqueen_hw/abstract/ICommDevice.h"
#include "mcqueen_hw/concrete/BluetoothAudioComm.h"

#include <iostream>
#include <memory>

int main() {
    std::unique_ptr<ICommDevice> comm = std::make_unique<BluetoothAudioComm>("00:1A:7D:DA:71:13");

    if (!comm->open()) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    comm->write("/home/halil/music/senaydin.mp3");
    comm.close();

    return 0;
}
