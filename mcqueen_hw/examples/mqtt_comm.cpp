#include "mcqueen_hw/abstract/ICommDevice.h"
#include "mcqueen_hw/concrete/MqttComm.h"

#include <iostream>
#include <memory>

int main() {
    std::unique_ptr<ICommDevice> comm = std::make_unique<MqttComm>("tcp://192.168.0.28:1883", "client_id");

    if (!comm->open()) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    if (auto mqttCommPtr = dynamic_cast<MqttComm*>(comm.get())) {
        mqttCommPtr->subscribe("sensors/temp");
        mqttCommPtr->subscribe("alerts");
    }

    comm->write("23.5", "sensors/temp");
    comm->write("Fire!", "alerts");

    std::string temp = comm->read(256, "sensors/temp");
    std::string alert = comm->read(256, "alerts");

    std::cout << "Temp: " << temp << "\n";
    std::cout << "Alert: " << alert << "\n";

    return 0;
}
