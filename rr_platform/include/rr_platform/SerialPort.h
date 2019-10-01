#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>

class SerialPort {
  public:
    SerialPort() = default;

    ~SerialPort();

    bool Open(const std::string &device, const unsigned int baud);

    void Close();

    void Write(std::string message);

    std::string ReadLine();

  private:
    int port_handle_;

    void SetProperties(unsigned int baud);
};

#endif  // SERIALPORT_H
