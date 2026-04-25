#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <string>
#include <mutex>
#include <atomic>
#include "SerialPort.h"

enum class MotorState {
    IDLE,
    MOVING_LEFT,
    MOVING_RIGHT,
    STOPPED,
    CALIBRATING
};

class MotorController {
public:
    MotorController();
    ~MotorController();

    bool connect(const std::string& port_name);
    void disconnect();
    void sendData(float pixel_error);
    void stop();
    
    MotorState getState() const;
    std::string getStateString() const;
    float getCurrentPosition() const;
    float getCurrentSpeed() const;
    bool isConnected() const;
    std::string getPortName() const;

private:
    SerialPort serial_port_;
    mutable std::mutex mutex_;
    
    std::atomic<MotorState> state_;
    float current_position_;
    std::atomic<bool> is_connected_;
    int8_t last_data_value_;
};

#endif // MOTOR_CONTROLLER_H