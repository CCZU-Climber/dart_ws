#include "AlignmentController.h"
#include <iostream>
#include <cmath>
#include <unistd.h> // for usleep

AlignmentController::AlignmentController() 
    : auto_align_enabled_(false),
      alignment_threshold_(5.0f),
      alignment_kp_(0.3f),
      current_pixel_error_(0.0f),
      is_aligned_(false),
      alignment_frame_count_(0),
      last_motor_data_(0) {
}

AlignmentController::~AlignmentController() {
    motor_controller_.disconnect();
}

void AlignmentController::performAlignment(const cv::Point2f& circle_center, int image_width) {
    // 计算图像中心
    float image_center_x = image_width / 2.0f;
    
    // 计算像素误差
    current_pixel_error_ = circle_center.x - image_center_x;
    
    // 判断是否已经对准（考虑死区）
    if (fabs(current_pixel_error_) <= alignment_threshold_) {
        alignment_frame_count_++;
        
        // 连续多帧对准才认为真正对准（去抖动）
        if (alignment_frame_count_ >= 5) {
            if (!is_aligned_) {
                is_aligned_ = true;
                // 发送停止命令
                motor_controller_.stop();
                last_motor_data_ = 0;
                std::cout << "✓ 已对准！像素误差: " << current_pixel_error_ 
                          << "px (阈值: " << alignment_threshold_ << "px)" << std::endl;
            }
        }
        return;
    } else {
        alignment_frame_count_ = 0;
        is_aligned_ = false;
    }
    
    // 检查串口连接状态
    if (!motor_controller_.isConnected()) {
        std::cout << "警告: 串口未连接，无法发送控制指令" << std::endl;
        return;
    }
    
    // 计算控制信号（直接使用像素误差，不限制范围）
    float control_signal = current_pixel_error_;
    
    // 发送控制信号给电机控制器
    motor_controller_.sendData(control_signal);
    
    // 记录最后发送的数据（根据像素误差计算）
    float abs_error = fabs(control_signal);
    if (abs_error < 50.0f) {
        last_motor_data_ = 0;
    } else if (abs_error < 100.0f) {
        last_motor_data_ = (control_signal > 0) ? 1 : -1;
    } else if (abs_error < 150.0f) {
        last_motor_data_ = (control_signal > 0) ? 2 : -2;
    } else if (abs_error < 200.0f) {
        last_motor_data_ = (control_signal > 0) ? 3 : -3;
    } else if (abs_error < 250.0f) {
        last_motor_data_ = (control_signal > 0) ? 4 : -4;
    } else {
        last_motor_data_ = (control_signal > 0) ? 5 : -5;
    }
}

void AlignmentController::toggleAutoAlign() {
    auto_align_enabled_ = !auto_align_enabled_;
    
    if (auto_align_enabled_) {
        std::cout << "自动对准已启用" << std::endl;
        std::cout << "对准阈值: " << alignment_threshold_ << "px" << std::endl;
        std::cout << "比例系数: " << alignment_kp_ << std::endl;
        
        // 检查串口连接
        if (!motor_controller_.isConnected()) {
            std::cout << "警告: 串口未连接，自动对准可能无法正常工作" << std::endl;
        }
    } else {
        std::cout << "自动对准已禁用" << std::endl;
        motor_controller_.stop();
        is_aligned_ = false;
        alignment_frame_count_ = 0;
        last_motor_data_ = 0;
    }
}

void AlignmentController::setAlignmentThreshold(float threshold) {
    if (threshold >= 1.0f && threshold <= 20.0f) {
        alignment_threshold_ = threshold;
        std::cout << "对准阈值已设置为: " << alignment_threshold_ << "px" << std::endl;
    } else {
        std::cout << "阈值超出范围，保持原值: " << alignment_threshold_ << "px" << std::endl;
    }
}

bool AlignmentController::setSerialPort(const std::string& port_name) {
    std::cout << "正在连接串口设备: " << port_name << "..." << std::endl;
    
    // 断开当前连接
    motor_controller_.disconnect();
    
    // 尝试新连接
    if (motor_controller_.connect(port_name)) {
        std::cout << "串口设备连接成功: " << port_name << std::endl;
        return true;
    } else {
        std::cout << "串口设备连接失败: " << port_name << std::endl;
        std::cout << "提示: 常见的串口设备有 /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyACM0 等" << std::endl;
        return false;
    }
}

void AlignmentController::printAlignmentStatus() const {
    std::cout << "\n=== 对准状态 ===" << std::endl;
    std::cout << "自动对准: " << (auto_align_enabled_ ? "启用" : "禁用") << std::endl;
    std::cout << "当前对准: " << (is_aligned_ ? "已对准" : "未对准") << std::endl;
    std::cout << "像素误差: " << current_pixel_error_ << "px" << std::endl;
    std::cout << "对准阈值: " << alignment_threshold_ << "px" << std::endl;
    std::cout << "电机状态: " << getMotorStateString() << std::endl;
    std::cout << "电机数据: " << static_cast<int>(last_motor_data_) << " (-5到5)" << std::endl;
    std::cout << "串口连接: " << (motor_controller_.isConnected() ? "已连接" : "未连接") << std::endl;
    std::cout << "串口设备: " << motor_controller_.getPortName() << std::endl;
    std::cout << "================\n" << std::endl;
}

bool AlignmentController::isAligned() const {
    return is_aligned_;
}

bool AlignmentController::isAutoAlignEnabled() const {
    return auto_align_enabled_;
}

float AlignmentController::getPixelError() const {
    return current_pixel_error_;
}

float AlignmentController::getAlignmentThreshold() const {
    return alignment_threshold_;
}

std::string AlignmentController::getMotorStateString() const {
    return motor_controller_.getStateString();
}

int8_t AlignmentController::getLastMotorData() const {
    return last_motor_data_;
}

bool AlignmentController::isMotorConnected() const {
    return motor_controller_.isConnected();
}

std::string AlignmentController::getPortName() const {
    return motor_controller_.getPortName();
}

void AlignmentController::stop() {
    motor_controller_.stop();
}

void AlignmentController::resetAlignment() {
    auto_align_enabled_ = false;
    is_aligned_ = false;
    alignment_frame_count_ = 0;
    last_motor_data_ = 0;
    motor_controller_.stop();
}

bool AlignmentController::connectMotorController() {
    std::cout << "正在尝试连接电机控制器..." << std::endl;
    
    // 尝试多个可能的串口设备
    const char* possible_ports[] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"};
    bool connected = false;
    
    for (const char* port : possible_ports) {
        std::cout << "尝试连接串口: " << port << std::endl;
        if (motor_controller_.connect(port)) {
            connected = true;
            break;
        }
        // 等待一小段时间再尝试下一个
        usleep(100000); // 100ms
    }
    
    if (!connected) {
        std::cout << "警告: 无法连接到任何串口设备，将在模拟模式下运行" << std::endl;
        return false;
    }
    
    return true;
}