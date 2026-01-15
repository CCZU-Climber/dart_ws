#include "SerialPort.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

template<typename T>
T clamp_value(T value, T min_val, T max_val) {
    return (value < min_val) ? min_val : ((value > max_val) ? max_val : value);
}

SerialPort::SerialPort() : serial_fd_(-1), is_connected_(false) {}

SerialPort::~SerialPort() {
    disconnect();
}

bool SerialPort::connect(const std::string& port_name, int baud_rate) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (is_connected_) disconnect();
    port_name_ = port_name;
    
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        std::cerr << "无法打开串口设备 " << port_name_ << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "获取串口配置失败: " << strerror(errno) << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    speed_t speed;
    switch (baud_rate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:
            std::cerr << "不支持的波特率: " << baud_rate << std::endl;
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    
    tcflush(serial_fd_, TCIOFLUSH);
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "设置串口配置失败: " << strerror(errno) << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    is_connected_ = true;
    std::cout << "串口连接成功: " << port_name_ << " @ " << baud_rate << " bps" << std::endl;
    return true;
}

void SerialPort::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
    is_connected_ = false;
    std::cout << "串口已断开" << std::endl;
}

bool SerialPort::isConnected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_connected_;
}

bool SerialPort::sendDataFrame(int8_t data_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_connected_ || serial_fd_ < 0) {
        std::cerr << "串口未连接，无法发送数据" << std::endl;
        return false;
    }
    
    DartDataFrame frame;
    frame.header[0] = 0xAA;
    frame.header[1] = 0x55;
    frame.data = clamp_value(data_value, static_cast<int8_t>(-5), static_cast<int8_t>(5));
    frame.footer[0] = 0x0D;
    frame.footer[1] = 0x0A;
    
    ssize_t bytes_written = write(serial_fd_, &frame, sizeof(DartDataFrame));
    
    if (bytes_written == sizeof(DartDataFrame)) {
        static int debug_counter = 0;
        if (debug_counter++ % 20 == 0) {
            std::cout << "发送串口数据(5字节): data=" << static_cast<int>(frame.data) 
                      << " (0x" << std::hex << (frame.data & 0xFF) << std::dec << ")" << std::endl;
        }
        tcdrain(serial_fd_);
        return true;
    } else {
        std::cerr << "发送串口数据失败: 预期" << sizeof(DartDataFrame) 
                  << "字节，实际" << bytes_written << "字节" << std::endl;
        return false;
    }
}

std::string SerialPort::getPortName() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return port_name_;
}