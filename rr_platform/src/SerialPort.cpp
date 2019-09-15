#include <fcntl.h>
#include <rr_platform/SerialPort.h>
#include <termio.h>
#include <zconf.h>
#include <cstring>
#include <iostream>

using namespace std;

SerialPort::~SerialPort() {
    Close();
}

bool SerialPort::Open(const std::string &device, const unsigned int baud) {
    port_handle_ = open(device.c_str(), O_RDWR | O_NOCTTY);

    SetProperties(baud);

    return (port_handle_ != -1);
}

void SerialPort::Close() {
    if (port_handle_ != -1) {
        // Flush port to ensure any last minute writes make it to the hardware
        tcflush(port_handle_, TCIFLUSH);
        close(port_handle_);
    }
}

void SerialPort::Write(std::string message) {
    ssize_t n_written = 0;
    ssize_t spot = 0;
    const char *msg_data = message.c_str();
    do {
        n_written = write(port_handle_, &msg_data[spot], message.size() - n_written);
        spot += n_written;
    } while (spot < message.size() && n_written > 0);
}

std::string SerialPort::ReadLine() {
    string line;
    ssize_t n_read = 0;
    char in;
    do {
        n_read = read(port_handle_, &in, 1);
        if (in != '\n')
            line += in;
    } while (in != '\n' && n_read > 0);
    return line;
}

void SerialPort::SetProperties(unsigned int baud) {
    struct termios tty;
    struct termios tty_old;
    memset(&tty, 0, sizeof tty);

    /* Error Handling */
    if (tcgetattr(port_handle_, &tty) != 0) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB;  // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;        // no flow control
    tty.c_cc[VMIN] = 1;             // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush(port_handle_, TCIFLUSH);
    if (tcsetattr(port_handle_, TCSANOW, &tty) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
}
