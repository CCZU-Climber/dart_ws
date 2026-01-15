#include "UserInterface.h"
#include <iostream>
#include <iomanip>

const std::string UserInterface::CAMERA_WINDOW = "Camera View";
const std::string UserInterface::RESULT_WINDOW = "Detection Result";

UserInterface::UserInterface() 
    : show_grid_(true),
      show_debug_info_(false),
      frame_counter_(0) {
}

void UserInterface::initWindows() {
    cv::namedWindow(CAMERA_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(RESULT_WINDOW, cv::WINDOW_AUTOSIZE);
}

void UserInterface::displayResults(const cv::Mat& camera_frame, const cv::Mat& detection_result,
                                 bool show_grid, const AlignmentController& align_controller,
                                 double processing_time_ms) {
    // 创建摄像头视图的副本用于显示
    cv::Mat camera_view = camera_frame.clone();
    
    // 如果启用了网格线，添加到摄像头视图
    if (show_grid) {
        GridDrawer::drawGridLines(camera_view);
    }
    
    // 显示摄像头视图
    cv::imshow(CAMERA_WINDOW, camera_view);
    
    // 显示检测结果
    if (!detection_result.empty()) {
        cv::Mat result_display = detection_result.clone();
        
        // 计算并显示处理时间
        double fps = (processing_time_ms > 0) ? 1000.0 / processing_time_ms : 0;
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
        cv::putText(result_display, 
                   fps_text,
                   cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        std::string time_text = "Time: " + std::to_string(processing_time_ms) + "ms";
        cv::putText(result_display, 
                   time_text,
                   cv::Point(10, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1);
        
        // 显示对准状态
        std::string align_status = align_controller.isAutoAlignEnabled() ? 
                                   "AUTO-ALIGN: ON" : "AUTO-ALIGN: OFF";
        cv::Scalar align_color = align_controller.isAutoAlignEnabled() ? 
                                 cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(result_display, 
                   align_status,
                   cv::Point(10, 90), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, align_color, 1);
        
        if (align_controller.isAligned()) {
            cv::putText(result_display, 
                       "ALIGNED!",
                       cv::Point(result_display.cols - 100, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
        
        // 显示像素误差
        std::string error_text = "Error: " + std::to_string(static_cast<int>(align_controller.getPixelError())) + "px";
        cv::putText(result_display, 
                   error_text,
                   cv::Point(10, 120), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1);
        
        // 显示电机状态
        std::string motor_state = "Motor: " + align_controller.getMotorStateString();
        cv::putText(result_display, 
                   motor_state,
                   cv::Point(result_display.cols - 150, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 100), 1);
        
        // 显示串口连接状态
        std::string serial_status = align_controller.isMotorConnected() ? 
                                   "Serial: Connected" : "Serial: Disconnected";
        cv::Scalar serial_color = align_controller.isMotorConnected() ? 
                                 cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(result_display, 
                   serial_status,
                   cv::Point(result_display.cols - 200, 90), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, serial_color, 1);
        
        // 显示当前电机控制数据
        std::string motor_data = "Motor Data: " + std::to_string(static_cast<int>(align_controller.getLastMotorData()));
        cv::putText(result_display, 
                   motor_data,
                   cv::Point(10, 150), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 100, 255), 1);
        
        // 如果启用了网格线，添加到结果视图
        if (show_grid) {
            GridDrawer::drawGridLines(result_display);
        }
        
        // 显示结果视图
        cv::imshow(RESULT_WINDOW, result_display);
    }
}

void UserInterface::handleKeyPress(int key, VisionDetector& vision_detector, AlignmentController& align_controller) {
    // 按ESC键或'q'键退出
    if (key == 27 || key == 'q' || key == 'Q') {
        return;
    }
    
    // 处理其他按键
    handleSaveFrame(key, vision_detector);
    handleCircularityThreshold(key, vision_detector);
    handleDetectionMode(key, vision_detector);
    handleDebugToggle(key, vision_detector);
    handleGridToggle(key);
    handleAlignmentToggle(key, align_controller);
    handleAlignmentThreshold(key, align_controller);
    handleAlignmentStatus(key, align_controller);
    handleSerialPort(key, align_controller);
    handleResetParameters(key, vision_detector, align_controller);
}

bool UserInterface::getShowGrid() const {
    return show_grid_;
}

bool UserInterface::getShowDebugInfo() const {
    return show_debug_info_;
}

void UserInterface::setShowGrid(bool show) {
    show_grid_ = show;
}

void UserInterface::setShowDebugInfo(bool show) {
    show_debug_info_ = show;
}

void UserInterface::closeWindows() {
    cv::destroyAllWindows();
}

void UserInterface::printHelp() {
    std::cout << "==========================================" << std::endl;
    std::cout << "按 'ESC' 或 'q' 键退出程序" << std::endl;
    std::cout << "按 's' 键保存当前帧" << std::endl;
    std::cout << "按 'r' 键重置参数" << std::endl;
    std::cout << "按 '+' 键增加圆形度阈值" << std::endl;
    std::cout << "按 '-' 键降低圆形度阈值" << std::endl;
    std::cout << "按 'm' 键切换检测模式" << std::endl;
    std::cout << "按 'd' 键显示/隐藏调试信息" << std::endl;
    std::cout << "按 'c' 键显示/隐藏中心网格线" << std::endl;
    std::cout << "按 'a' 键开启/关闭自动对准" << std::endl;
    std::cout << "按 't' 键设置对准阈值" << std::endl;
    std::cout << "按 'p' 键显示当前对准状态" << std::endl;
    std::cout << "按 'o' 键设置串口设备" << std::endl;
    std::cout << "==========================================" << std::endl;
}

void UserInterface::handleSaveFrame(int key, VisionDetector& vision_detector) {
    if (key == 's' || key == 'S') {
        vision_detector.saveCurrentFrame(frame_counter_);
    }
}

void UserInterface::handleCircularityThreshold(int key, VisionDetector& vision_detector) {
    if (key == '+') {
        double current = vision_detector.getCircularityThreshold();
        double new_value = std::min(current + 0.05, 1.0);
        vision_detector.setCircularityThreshold(new_value);
        std::cout << "圆形度阈值增加至: " << new_value << std::endl;
    } else if (key == '-') {
        double current = vision_detector.getCircularityThreshold();
        double new_value = std::max(current - 0.05, 0.1);
        vision_detector.setCircularityThreshold(new_value);
        std::cout << "圆形度阈值降低至: " << new_value << std::endl;
    }
}

void UserInterface::handleDetectionMode(int key, VisionDetector& vision_detector) {
    if (key == 'm' || key == 'M') {
        int current_mode = vision_detector.getDetectionMode();
        int new_mode = (current_mode + 1) % 3;
        vision_detector.setDetectionMode(new_mode);
        
        std::cout << "检测模式: ";
        if (new_mode == 0) std::cout << "亮核检测" << std::endl;
        else if (new_mode == 1) std::cout << "梯度约束" << std::endl;
        else std::cout << "混合模式" << std::endl;
    }
}

void UserInterface::handleDebugToggle(int key, VisionDetector& vision_detector) {
    if (key == 'd' || key == 'D') {
        show_debug_info_ = !show_debug_info_;
        vision_detector.setDebugInfo(show_debug_info_);
        std::cout << "调试信息: " << (show_debug_info_ ? "显示" : "隐藏") << std::endl;
    }
}

void UserInterface::handleGridToggle(int key) {
    if (key == 'c' || key == 'C') {
        show_grid_ = !show_grid_;
        std::cout << "中心网格线: " << (show_grid_ ? "显示" : "隐藏") << std::endl;
    }
}

void UserInterface::handleAlignmentToggle(int key, AlignmentController& align_controller) {
    if (key == 'a' || key == 'A') {
        align_controller.toggleAutoAlign();
    }
}

void UserInterface::handleAlignmentThreshold(int key, AlignmentController& align_controller) {
    if (key == 't' || key == 'T') {
        std::cout << "当前对准阈值: " << align_controller.getAlignmentThreshold() << "px" << std::endl;
        std::cout << "请输入新的阈值 (1-20像素): ";
        
        float new_threshold;
        std::cin >> new_threshold;
        align_controller.setAlignmentThreshold(new_threshold);
    }
}

void UserInterface::handleAlignmentStatus(int key, AlignmentController& align_controller) {
    if (key == 'p' || key == 'P') {
        align_controller.printAlignmentStatus();
    }
}

void UserInterface::handleSerialPort(int key, AlignmentController& align_controller) {
    if (key == 'o' || key == 'O') {
        std::cout << "当前串口设备: " << align_controller.getPortName() << std::endl;
        std::cout << "请输入新的串口设备路径 (例如: /dev/ttyUSB0): ";
        
        std::string new_port;
        std::cin >> new_port;
        align_controller.setSerialPort(new_port);
    }
}

void UserInterface::handleResetParameters(int key, VisionDetector& vision_detector, AlignmentController& align_controller) {
    if (key == 'r' || key == 'R') {
        // 重置视觉检测参数
        vision_detector.setCircularityThreshold(0.5);
        vision_detector.setDetectionMode(2);
        vision_detector.setDebugInfo(false);
        
        // 重置对准参数
        align_controller.resetAlignment();
        align_controller.setAlignmentThreshold(5.0f);
        
        // 重置UI参数
        show_grid_ = true;
        show_debug_info_ = false;
        
        std::cout << "参数已重置" << std::endl;
    }
}