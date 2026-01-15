#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <mutex>
#include <cstdint>

#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];   // 帧头：0xAA, 0x55
    int8_t data;         // 数据：-5到5
    uint8_t footer[2];   // 帧尾：0x0D, 0x0A (CR LF)
} DartDataFrame;
#pragma pack(pop)

class SerialPort {
private:
    int serial_fd_;
    std::string port_name_;
    bool is_connected_;
    mutable std::mutex mutex_;
    
public:
    SerialPort();
    ~SerialPort();
    
    bool connect(const std::string& port_name = "/dev/ttyUSB0", int baud_rate = 115200);
    void disconnect();
    bool isConnected() const;
    bool sendDataFrame(int8_t data_value);
    std::string getPortName() const;
};

#endif