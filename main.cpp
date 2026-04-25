#include "HikCam.h"
#include "VisionDetector.h"
#include "AlignmentController.h"
#include "UserInterface.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <cstdlib>
// 🔧 新增：包含 CameraCalibrator 头文件
#include "CameraCalibrator.h"
#include "DetectionResult.h"
#include "DistanceEstimator.h"

using namespace sensor::camera;

int main() {
    std::cout << "\033[32m=== 海康相机飞镖检测系统 ===\033[0m" << std::endl;
    
    // 👇👇👇 新增标定选项（开始）👇👇👇
    std::cout << "\n\033[33m是否需要进行相机标定？\033[0m" << std::endl;
    std::cout << "1. 进行相机标定（推荐首次使用或更换镜头后）" << std::endl;
    std::cout << "2. 跳过标定，使用现有参数" << std::endl;
    std::cout << "请选择 (1-2): ";
    
    int calibrateChoice;
    std::cin >> calibrateChoice;
    std::cin.ignore();  // 清除输入缓冲区
    
    if (calibrateChoice == 1) {
        std::cout << "\n\033[36m=== 开始相机标定 ===\033[0m" << std::endl;
        std::cout << "标定板规格: 27cm × 36cm，每格3cm" << std::endl;
        std::cout << "请将标定板放置在距离相机2-10米范围内" << std::endl;
        std::cout << "按回车键继续..." << std::endl;
        std::cin.get();
        
        try {
            // 🔧 修正：正确创建 CameraCalibrator 实例
            CameraCalibrator calibrator;
            calibrator.setBoardParams(9, 12, 0.03f);  // 9x12格子，每格3cm
            
            // 初始化相机
            CAM_INFO camInfo;
            camInfo.setCamID(0).setWidth(2448).setHeight(2048).setExpTime(5000).setGain(16);
            HikCam camera(camInfo);
            
            // 采集图像
            bool captured = calibrator.captureImages(camera, "/home/yin/Desktop/HikCam/dart_ws/calibration_images", 15, 30);
            
            if (captured) {
                // 执行标定
                bool calibrated = calibrator.calibrate("calibration_images", "camera_params.yml");
                if (calibrated) {
                    std::cout << "\033[32m✅ 相机标定成功完成！\033[0m" << std::endl;
                    std::cout << "标定参数已保存到: camera_params.yml" << std::endl;
                } else {
                    std::cerr << "\033[31m❌ 标定失败，请检查采集的图像质量\033[0m" << std::endl;
                }
            } else {
                std::cerr << "\033[31m❌ 图像采集失败或数量不足\033[0m" << std::endl;
            }
            
            std::cout << "\n按回车键继续启动飞镖检测系统..." << std::endl;
            std::cin.get();
        } catch (const std::exception& e) {
            std::cerr << "\033[31m❌ 标定过程中出错: " << e.what() << "\033[0m" << std::endl;
            std::cout << "按回车键继续..." << std::endl;
            std::cin.get();
        }
    }
    // 👆👆👆 新增标定选项（结束）👆👆👆

    try {
        // 初始化摄像头配置
        CAM_INFO camInfo;
        camInfo.setCamID(0)
               .setWidth(2448)
               .setHeight(2048)
               .setOffsetX(0)
               .setOffsetY(0)
               .setExpTime(5000.0f)
               .setGain(16.0f)
               .setTrigger(sensor::camera::CONTINUOUS)
               .setGamma(sensor::camera::sRGB);

        // 创建海康摄像头实例
        HikCam camera(camInfo);
        
        // 创建各个模块实例
        VisionDetector vision_detector;
        AlignmentController alignment_controller;
        UserInterface ui;
        
        // 创建距离估算器
        DistanceEstimator distance_estimator;
        
        // 关键配置：根据您的55mm（0.055m）实际尺寸设置
        const float REAL_TARGET_DIAMETER = 0.055f;  // 前哨站灯的真实直径（55mm = 0.055米）
        
        // 焦距可以通过相机标定获得，或者先使用估计值
        float estimated_focal_length = 4968.4f;  // 这个值需要根据您的相机进行标定
        
        // 如果存在标定文件，可以从文件加载
        cv::FileStorage fs("camera_params.yml", cv::FileStorage::READ);
        if (fs.isOpened()) {
            cv::Mat camera_matrix;
            fs["camera_matrix"] >> camera_matrix;
            if (!camera_matrix.empty()) {
                estimated_focal_length = (camera_matrix.at<double>(0, 0) + camera_matrix.at<double>(1, 1)) / 2.0f;
            }
            fs.release();
        }
        
        distance_estimator.setCameraParameters(estimated_focal_length, REAL_TARGET_DIAMETER);
        
        std::cout << "距离估算器已初始化：" << std::endl;
        std::cout << "- 真实直径: " << REAL_TARGET_DIAMETER * 1000 << "mm" << std::endl;
        std::cout << "- 焦距: " << estimated_focal_length << "px" << std::endl;

        // 初始化UI窗口
        ui.initWindows();
        
        // 连接电机控制器
        alignment_controller.connectMotorController();

        // 不要调用不存在的方法
        // alignment_controller.loadDistanceSettingFromEnvironment();

        // 设置物理偏移时必须同时给出 mm/mm_per_pixel 比例
        const float user_camera_offset_mm = 76.0f; // 用户提供的物理偏移（mm）
        const float mm_per_pixel = 0.15f;          // 根据实际标定调整

        if (const char* env_px = std::getenv("CAM_OFFSET_PX")) {
            try {
                float px = std::stof(env_px);
                alignment_controller.setCameraOffsetPixels(px);
            } catch (...) {
                std::cout << "无法解析环境变量 CAM_OFFSET_PX 的值: " << env_px << std::endl;
            }
        } else {
            alignment_controller.setCameraOffsetMM(user_camera_offset_mm, mm_per_pixel);
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
                
                // 使用新的检测方法获取完整结果
                std::vector<DetectionResult> detection_results;
                cv::Mat result = vision_detector.detectGreenCirclesWithResults(frame, detection_results);

                // 计算距离
                if (!detection_results.empty()) {
                    distance_estimator.estimateDistances(detection_results);
                    
                    // 打印调试信息
                    for (size_t i = 0; i < detection_results.size(); ++i) {
                        const auto& res = detection_results[i];
                        std::string distance_str = res.has_distance ? 
                            std::to_string(res.distance) + "m" : "N/A";
                        
                        std::cout << "目标 #" << i << ": "
                                  << "像素直径: " << res.pixel_diameter << "px, "
                                  << "圆度: " << res.confidence << ", "
                                  << "距离: " << distance_str
                                  << std::endl;
                    }
                }

                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                double processing_time_ms = duration.count() / 1000.0;

                // 自动对准已启用时：
                // - 若检测到圆形则执行对准
                // - 若未检测到圆形则立即停止电机（实现选项 C）
                if (alignment_controller.isAutoAlignEnabled()) {
                    if (!detection_results.empty()) {
                        // 使用最佳检测结果进行对准
                        auto& best_result = detection_results[0];
                        cv::Point2f center(best_result.circle[0], best_result.circle[1]);
                        alignment_controller.performAlignment(center, frame.cols);
                    } else {
                        // 目标丢失：立即停止电机，防止继续维持最后速度
                        alignment_controller.stop();
                        std::cout << "警告: 目标丢失，已发送停止命令到电机。" << std::endl;
                    }
                }
                
                // 显示结果（需要更新UI以显示距离信息）
                ui.displayResults(frame, result, 
                 alignment_controller, processing_time_ms, 
                 &detection_results, ui.getShowGrid());
                
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