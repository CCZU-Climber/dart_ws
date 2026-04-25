#include "MotorController.h"
#include <iostream>
#include <cmath>
#include <mutex>
#include <cstdint>

// ============================================================================
// 【核心配置参数】
// ============================================================================

// 目标像素偏移量：76mm / 260mm_per_px ≈ 22.03px
// 含义：物体应该停留在距离图像中心右侧22.03像素的位置
constexpr float TARGET_PIXEL_OFFSET = 22.03f;

// 死区阈值：5px (约17mm物理距离)
constexpr float DEAD_ZONE_THRESHOLD = 5.0f;

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
        std::cout << "电机控制器已通过串口连接: " << port_name << std::endl;
        return true;
    } else {
        std::cerr << "电机控制器连接失败: " << port_name << std::endl;
        return false;
    }
}

void MotorController::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_connected_) {
        serial_port_.sendDataFrame(0);
        serial_port_.disconnect();
        is_connected_ = false;
        state_ = MotorState::STOPPED;
        std::cout << "电机控制器已断开连接" << std::endl;
    }
}

void MotorController::sendData(float pixel_error) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_connected_) {
        return;
    }
    
    int8_t data_value = 0;
    
    // ========================================================================
    // 【控制逻辑详解】
    // ========================================================================
    // pixel_error = 物体中心坐标 - 图像中心坐标
    //   > 0 : 物体在图像右侧
    //   < 0 : 物体在图像左侧
    //
    // control_error = pixel_error - TARGET_PIXEL_OFFSET
    //   > 0 : 物体在目标位置右侧，需要相机向右转（让物体左移）
    //   < 0 : 物体在目标位置左侧，需要相机向左转（让物体右移）
    //
    // 相机转动与图像关系：
    //   相机向右转 → 图像中物体向左移动 → 像素值减小
    //   相机向左转 → 图像中物体向右移动 → 像素值增大
    //
    // 串口数据约定（根据实际硬件调整）：
    //   正值 → 电机向右转
    //   负值 → 电机向左转
    // ========================================================================
    
    float control_error = pixel_error - TARGET_PIXEL_OFFSET;
    float abs_control_error = std::fabs(control_error);
    
    if (abs_control_error < DEAD_ZONE_THRESHOLD) {
        // 死区内：停止
        data_value = 0;
        state_ = MotorState::STOPPED;
    } 
    else if (abs_control_error < 6.0f) {
        // 微动 (速度1)
        data_value = (control_error > 0) ? 1 : -1;
        state_ = (control_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    } 
    else if (abs_control_error < 15.0f) {
        // 低速 (速度2)
        data_value = (control_error > 0) ? 2 : -2;
        state_ = (control_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    } 
    else if (abs_control_error < 45.0f) {
        // 中速 (速度3)
        data_value = (control_error > 0) ? 3 : -3;
        state_ = (control_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    } 
    else if (abs_control_error < 90.0f) {
        // 高速 (速度4)
        data_value = (control_error > 0) ? 4 : -4;
        state_ = (control_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    } 
    else {
        // 全速 (速度5)
        data_value = (control_error > 0) ? 5 : -5;
        state_ = (control_error > 0) ? MotorState::MOVING_RIGHT : MotorState::MOVING_LEFT;
    }
    
    if (serial_port_.sendDataFrame(data_value)) {
        last_data_value_ = data_value;
        float degree_per_data = 2.0f;
        current_position_ += data_value * degree_per_data;
        
        std::cout << "电机控制: 原始误差=" << pixel_error 
                  << "px, 控制误差(距22px)=" << control_error 
                  << "px, 绝对误差=" << abs_control_error
                  << ", 命令值=" << static_cast<int>(data_value)
                  << ", 状态=" << getStateString() << std::endl;
    }
}

std::string MotorController::getStateString() const {
    switch (state_) {
        case MotorState::IDLE: return "空闲";
        case MotorState::MOVING_LEFT: return "向左移动";
        case MotorState::MOVING_RIGHT: return "向右移动";
        case MotorState::STOPPED: return "停止 (已对准)";
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
        std::cout << "电机紧急停止" << std::endl;
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