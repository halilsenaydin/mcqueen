#ifndef ICOMM_DEVICE_H
#define ICOMM_DEVICE_H

#include <string>

class ICommDevice {
public:
    virtual ~ICommDevice();

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool write(const std::string& data, const std::string& target = "") = 0;
    virtual std::string read(size_t maxBytes = 256, const std::string& target = "") = 0;
};

#endif // ICOMM_DEVICE_H
