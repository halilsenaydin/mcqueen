#include "mcqueen_hw/concrete/SerialComm.h"

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

SerialComm::SerialComm(const std::string& portName, unsigned int baudRate)
    : _portName(portName), _baudRate(baudRate), _serialFd(-1) {}

SerialComm::~SerialComm() {
    close();
}

bool SerialComm::open() {
    _serialFd = ::open(_portName.c_str(), O_RDWR | O_NOCTTY);
    
    if (_serialFd < 0) {
        std::cerr << "Failed to open serial port " << _portName << std::endl;
        return false;
    }

    return configurePort();
}

void SerialComm::close() {
    if (_serialFd >= 0) {
        ::close(_serialFd);
        _serialFd = -1;
    }
}

bool SerialComm::configurePort() {
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(_serialFd, &tty) != 0) {
        std::cerr << "Error from tcgetattr\n";
        return false;
    }

    speed_t baud;
    switch (_baudRate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default:
            std::cerr << "Unsupported baud rate, defaulting to 9600\n";
            baud = B9600;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 bit data
    tty.c_cflag &= ~CRTSCTS; // Hardware flow control closed
    tty.c_cflag |= CREAD | CLOCAL; // Read open, local connection

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1 second timeout

    if (tcsetattr(_serialFd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr\n";
        return false;
    }

    return true;
}

bool SerialComm::write(const std::string& data, const std::string& target) {
    if (_serialFd < 0) return false;

    ssize_t bytesWritten = ::write(_serialFd, data.c_str(), data.size());

    return bytesWritten == (ssize_t)data.size();
}

std::string SerialComm::read(size_t maxBytes, const std::string& target) {
    if (_serialFd < 0) return "";

    char buf[256];
    ssize_t bytesRead = ::read(_serialFd, buf, maxBytes > sizeof(buf) ? sizeof(buf) : maxBytes);

    if (bytesRead > 0) {
        return std::string(buf, bytesRead);
    }
    return "";
}
