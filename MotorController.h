#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "SerialPort.h"
#include <string>
#include <mutex>

enum class MotorState {
    IDLE,
    MOVING_LEFT,
    MOVING_RIGHT,
    STOPPED,
    CALIBRATING
};

class MotorController {
private:
    SerialPort serial_port_;
    MotorState state_;
    float current_position_;
    bool is_connected_;
    mutable std::mutex mutex_;
    int8_t last_data_value_;
    
public:
    MotorController();
    ~MotorController();
    
    bool connect(const std::string& port_name = "/dev/ttyUSB0");
    void disconnect();
    void sendData(float pixel_error);
    std::string getStateString() const;
    void stop();
    MotorState getState() const;
    float getCurrentPosition() const;
    float getCurrentSpeed() const;
    bool isConnected() const;
    std::string getPortName() const;
};

#endif