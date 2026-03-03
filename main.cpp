#include "HikCam.h"
#include "VisionDetector.h"
#include "AlignmentController.h"
#include "UserInterface.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <cstdlib>

int main() {
    try {
        // 初始化摄像头配置
        sensor::camera::CAM_INFO camInfo;
        camInfo.setCamID(0)           // 设置相机ID
               .setWidth(2448)         // 图像宽度（针对 2448 x 2048 的 5MP 摄像头）
               .setHeight(2048)        // 图像高度
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

        // 如果已知摄像头相对于发射架的物理偏移（用户提供的 76 mm），尝试通过环境变量设置像素缩放
        // 优先使用直接像素偏移：环境变量 CAM_OFFSET_PX
        // 否则可提供像素每毫米（PX_PER_MM），程序将把 76 mm 转换为像素：px = mm * PX_PER_MM
        const float user_camera_offset_mm = 76.0f; // 用户提供的物理偏移（mm）
        if (const char* env_px = std::getenv("CAM_OFFSET_PX")) {
            try {
                float px = std::stof(env_px);
                alignment_controller.setCameraOffsetPixels(px);
            } catch (...) {
                std::cout << "无法解析环境变量 CAM_OFFSET_PX 的值: " << env_px << std::endl;
            }
        } else if (const char* env_px_per_mm = std::getenv("PX_PER_MM")) {
            try {
                float px_per_mm = std::stof(env_px_per_mm);
                alignment_controller.setCameraOffsetMM(user_camera_offset_mm, px_per_mm);
            } catch (...) {
                std::cout << "无法解析环境变量 PX_PER_MM 的值: " << env_px_per_mm << std::endl;
            }
        } else {
            std::cout << "提示: 未设置摄像头偏移，若要应用 76 mm 偏移，请在启动前设置环境变量之一：" << std::endl;
            std::cout << "  CAM_OFFSET_PX=<像素偏移值> 或 PX_PER_MM=<像素每毫米> (例如: PX_PER_MM=10 意味着 10 px/mm)" << std::endl;
            std::cout << "当前默认使用 0 px 偏移。如需帮助，请提供相机的像素/mm 比例或直接的像素偏移值。" << std::endl;
        }
        
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

                // 自动对准已启用时：
                // - 若检测到圆形则执行对准
                // - 若未检测到圆形则立即停止电机（实现选项 C）
                if (alignment_controller.isAutoAlignEnabled()) {
                    if (!detected_circles.empty()) {
                        alignment_controller.performAlignment(detected_circles[0], frame.cols);
                    } else {
                        // 目标丢失：立即停止电机，防止继续维持最后速度
                        alignment_controller.stop();
                        std::cout << "警告: 目标丢失，已发送停止命令到电机。" << std::endl;
                    }
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