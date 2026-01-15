#include "HikCam.h"
#include "VisionDetector.h"
#include "AlignmentController.h"
#include "UserInterface.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>

int main() {
    try {
        // 初始化摄像头配置
        sensor::camera::CAM_INFO camInfo;
        camInfo.setCamID(0)           // 设置相机ID
               .setWidth(640)         // 图像宽度
               .setHeight(480)        // 图像高度
               .setOffsetX(0)         // 设置X偏移
               .setOffsetY(0)         // 设置Y偏移
               .setExpTime(5000.0f)   // 设置曝光时间
               .setGain(16.0f)        // 设置增益
               .setTrigger(sensor::camera::SOFTWARE)
               .setGamma(sensor::camera::sRGB);

        // 创建海康摄像头实例
        sensor::camera::HikCam camera(camInfo);
        
        // 创建各个模块实例
        VisionDetector vision_detector;
        AlignmentController alignment_controller;
        UserInterface ui;
        
        // 初始化UI窗口
        ui.initWindows();
        
        // 连接电机控制器
        alignment_controller.connectMotorController();
        
        std::cout << "前哨站绿色圆形灯识别系统初始化完成" << std::endl;
        UserInterface::printHelp();
        
        // 性能统计变量
        int frame_count = 0;
        auto start_total_time = std::chrono::high_resolution_clock::now();
        
        while (true) {
            // 捕获图像
            cv::Mat frame = camera.Grab();
            
            if (!frame.empty()) {
                auto start_time = std::chrono::high_resolution_clock::now();
                
                // 检测绿色圆形并获取检测到的圆形中心
                std::vector<cv::Point2f> detected_circles;
                cv::Mat result = vision_detector.detectGreenCircles(frame, detected_circles);
                
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                double processing_time_ms = duration.count() / 1000.0;
                
                // 如果有检测到圆形且自动对准开启，执行对准操作
                if (!detected_circles.empty() && alignment_controller.isAutoAlignEnabled()) {
                    alignment_controller.performAlignment(detected_circles[0], frame.cols);
                }
                
                // 显示结果
                ui.displayResults(frame, result, ui.getShowGrid(), 
                                 alignment_controller, processing_time_ms);
                
                // 统计帧率
                frame_count++;
            } else {
                std::cout << "获取图像失败!" << std::endl;
            }
            
            // 检查按键
            int key = cv::waitKey(1);  // 使用更短的等待时间
            
            // 处理按键输入
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
            ui.handleKeyPress(key, vision_detector, alignment_controller);
        }
        
        // 计算平均FPS
        auto end_total_time = std::chrono::high_resolution_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_total_time - start_total_time).count();
        
        double avg_fps = (total_duration > 0) ? (frame_count * 1000.0) / total_duration : 0;
        
        std::cout << "\n程序结束" << std::endl;
        std::cout << "总帧数: " << frame_count << std::endl;
        std::cout << "总时间: " << total_duration << "ms" << std::endl;
        std::cout << "平均FPS: " << avg_fps << std::endl;
        
        // 关闭窗口
        ui.closeWindows();
        
    } catch (const std::exception& e) {
        std::cerr << "错误发生: " << e.what() << std::endl;
        return 1;
    }
    
    return 0; 
}