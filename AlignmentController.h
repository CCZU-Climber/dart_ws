#ifndef ALIGNMENTCONTROLLER_H
#define ALIGNMENTCONTROLLER_H

#include "MotorController.h"
#include <opencv2/opencv.hpp>
#include <string>

class AlignmentController {
private:
    MotorController motor_controller_;
    bool auto_align_enabled_;
    float alignment_threshold_;
    float alignment_kp_;
    float current_pixel_error_;
    bool is_aligned_;
    int alignment_frame_count_;
    int8_t last_motor_data_;
    
public:
    AlignmentController();
    ~AlignmentController();
    
    // 执行对准操作
    void performAlignment(const cv::Point2f& circle_center, int image_width);
    
    // 切换自动对准
    void toggleAutoAlign();
    
    // 设置对准阈值
    void setAlignmentThreshold(float threshold);
    
    // 设置串口设备
    bool setSerialPort(const std::string& port_name);
    
    // 打印对准状态
    void printAlignmentStatus() const;
    
    // 获取对准状态
    bool isAligned() const;
    bool isAutoAlignEnabled() const;
    float getPixelError() const;
    float getAlignmentThreshold() const;
    std::string getMotorStateString() const;
    int8_t getLastMotorData() const;
    bool isMotorConnected() const;
    std::string getPortName() const;
    
    // 停止对准
    void stop();
    
    // 重置对准状态
    void resetAlignment();
    
    // 连接电机控制器
    bool connectMotorController();
};

#endif // ALIGNMENTCONTROLLER_H