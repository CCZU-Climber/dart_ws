#ifndef USERINTERFACE_H
#define USERINTERFACE_H

#include "VisionDetector.h"
#include "AlignmentController.h"
#include "GridDrawer.h"
#include <string>

class UserInterface {
private:
    // 显示相关参数
    bool show_grid_;
    bool show_debug_info_;
    int frame_counter_;
    
    // 窗口名称
    static const std::string CAMERA_WINDOW;
    static const std::string RESULT_WINDOW;
    
public:
    UserInterface();
    
    // 初始化窗口
    void initWindows();
    
    // 显示处理结果
    void displayResults(const cv::Mat& camera_frame, const cv::Mat& detection_result,
                       bool show_grid, const AlignmentController& align_controller,
                       double processing_time_ms);
    
    // 处理按键事件
    void handleKeyPress(int key, VisionDetector& vision_detector, AlignmentController& align_controller);
    
    // 获取显示参数
    bool getShowGrid() const;
    bool getShowDebugInfo() const;
    
    // 设置显示参数
    void setShowGrid(bool show);
    void setShowDebugInfo(bool show);
    
    // 关闭窗口
    void closeWindows();
    
    // 打印帮助信息
    static void printHelp();
    
private:
    // 处理特定按键
    void handleSaveFrame(int key, VisionDetector& vision_detector);
    void handleCircularityThreshold(int key, VisionDetector& vision_detector);
    void handleDetectionMode(int key, VisionDetector& vision_detector);
    void handleDebugToggle(int key, VisionDetector& vision_detector);
    void handleGridToggle(int key);
    void handleAlignmentToggle(int key, AlignmentController& align_controller);
    void handleAlignmentThreshold(int key, AlignmentController& align_controller);
    void handleAlignmentStatus(int key, AlignmentController& align_controller);
    void handleSerialPort(int key, AlignmentController& align_controller);
    void handleResetParameters(int key, VisionDetector& vision_detector, AlignmentController& align_controller);
};

#endif // USERINTERFACE_H