#include "MotorController.h"
#include <iostream>
#include <cmath>

MotorController::MotorController() 
    : state_(MotorState::IDLE),
      current_position_(0.0f),
      is_connected_(false),
      last_data_value_(0) {
}

MotorController::~MotorController() {
    disconnect();
}

bool MotorController::connect(const std::string& port_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (is_connected_) return true;
    
    if (serial_port_.connect(port_name, 115200)) {
        is_connected_ = true;
        state_ = MotorState::IDLE;
        last_data_value_ = 0;
        std::cout << "电机控制器已通过串口连接" << std::endl;
        return true;
    } else {
        std::cerr << "电机控制器连接失败" << std::endl;
        return false;
    }
}

void MotorController::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_connected_) {
        sendData(0);
        serial_port_.disconnect();
        is_connected_ = false;
        state_ = MotorState::STOPPED;
    }
}

void MotorController::sendData(float pixel_error) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_connected_) {
        std::cerr << "电机控制器未连接，无法发送数据" << std::endl;
        return;
    }
    
    int8_t data_value = 0;
    float abs_error = fabs(pixel_error);
    
    if (abs_error < 50.0f) {
        data_value = 0;
        state_ = MotorState::STOPPED;
    } else if (abs_error < 100.0f) {
        data_value = (pixel_error > 0) ? 1 : -1;
        state_ = (pixel_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    } else if (abs_error < 150.0f) {
        data_value = (pixel_error > 0) ? 2 : -2;
        state_ = (pixel_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    } else if (abs_error < 200.0f) {
        data_value = (pixel_error > 0) ? 3 : -3;
        state_ = (pixel_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    } else if (abs_error < 250.0f) {
        data_value = (pixel_error > 0) ? 4 : -4;
        state_ = (pixel_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    } else {
        data_value = (pixel_error > 0) ? 5 : -5;
        state_ = (pixel_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    }
    
    if (serial_port_.sendDataFrame(data_value)) {
        last_data_value_ = data_value;
        float degree_per_data = 2.0f;
        current_position_ += data_value * degree_per_data;
        
        std::cout << "电机控制: 像素误差=" << pixel_error 
                  << "px, 控制值=" << static_cast<int>(data_value)
                  << ", 状态=" << getStateString() << std::endl;
    }
}

std::string MotorController::getStateString() const {
    switch (state_) {
        case MotorState::IDLE: return "空闲";
        case MotorState::MOVING_LEFT: return "向左移动";
        case MotorState::MOVING_RIGHT: return "向右移动";
        case MotorState::STOPPED: return "停止";
        case MotorState::CALIBRATING: return "校准中";
        default: return "未知";
    }
}

void MotorController::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_connected_) {
        serial_port_.sendDataFrame(0);
        state_ = MotorState::STOPPED;
        last_data_value_ = 0;
    }
}

MotorState MotorController::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

float MotorController::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_position_;
}

float MotorController::getCurrentSpeed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_data_value_ * 10.0f;
}

bool MotorController::isConnected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_connected_;
}

std::string MotorController::getPortName() const {
    return serial_port_.getPortName();
}