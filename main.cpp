#include "HikCam.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

class GreenCircleDetector {
public:
    GreenCircleDetector() {
        // 初始化参数
        init_parameters();
        
        // 只创建必要的窗口
        cv::namedWindow("Camera View", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
        
        std::cout << "前哨站绿色圆形灯识别系统初始化完成" << std::endl;
        std::cout << "==========================================" << std::endl;
        std::cout << "按 'ESC' 或 'q' 键退出程序" << std::endl;
        std::cout << "按 's' 键保存当前帧" << std::endl;
        std::cout << "按 'r' 键重置参数" << std::endl;
        std::cout << "按 '+' 键增加圆形度阈值" << std::endl;
        std::cout << "按 '-' 键降低圆形度阈值" << std::endl;
        std::cout << "按 'm' 键切换检测模式" << std::endl;
        std::cout << "按 'd' 键显示/隐藏调试信息" << std::endl;
        std::cout << "按 'c' 键显示/隐藏中心网格线" << std::endl;
        std::cout << "==========================================" << std::endl;
    }
    
    ~GreenCircleDetector() {
        cv::destroyAllWindows();
    }
    
    void processFrame(const cv::Mat& frame) {
        if (frame.empty()) {
            return;
        }
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 检测绿色圆形
        cv::Mat result = detectGreenCircles(frame);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        // 计算并显示处理时间
        if (!result.empty()) {
            double fps = (duration.count() > 0) ? 1000000.0 / duration.count() : 0;
            std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
            cv::putText(result, 
                       fps_text,
                       cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            // 显示处理时间
            std::string time_text = "Time: " + std::to_string(duration.count() / 1000.0) + "ms";
            cv::putText(result, 
                       time_text,
                       cv::Point(10, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1);
        }
        
        // 创建摄像头视图的副本用于显示（添加网格线）
        cv::Mat camera_view = frame.clone();
        
        // 如果启用了网格线，添加到摄像头视图
        if (show_grid_) {
            drawGridLines(camera_view);
        }
        
        // 显示结果
        if (!result.empty()) {
            // 在检测结果上也添加网格线
            if (show_grid_) {
                drawGridLines(result);
            }
            cv::imshow("Detection Result", result);
        }
        cv::imshow("Camera View", camera_view);
    }
    
    void handleKeyPress(int key) {
        switch (key) {
            case 's':  // 保存图像
            case 'S':
                saveCurrentFrame();
                break;
            case 'r':  // 重置参数
            case 'R':
                resetParameters();
                break;
            case '+':  // 增加圆形度阈值
                circularity_threshold_ = std::min(circularity_threshold_ + 0.05, 1.0);
                std::cout << "圆形度阈值增加至: " << circularity_threshold_ << std::endl;
                break;
            case '-':  // 降低圆形度阈值
                circularity_threshold_ = std::max(circularity_threshold_ - 0.05, 0.1);
                std::cout << "圆形度阈值降低至: " << circularity_threshold_ << std::endl;
                break;
            case 'm':  // 切换检测模式
            case 'M':
                detection_mode_ = (detection_mode_ + 1) % 3;
                std::cout << "检测模式: ";
                if (detection_mode_ == 0) std::cout << "亮核检测" << std::endl;
                else if (detection_mode_ == 1) std::cout << "梯度约束" << std::endl;
                else std::cout << "混合模式" << std::endl;
                break;
            case 'd':  // 切换调试显示
            case 'D':
                show_debug_info_ = !show_debug_info_;
                std::cout << "调试信息: " << (show_debug_info_ ? "显示" : "隐藏") << std::endl;
                break;
            case 'c':  // 切换网格线显示
            case 'C':
                show_grid_ = !show_grid_;
                std::cout << "中心网格线: " << (show_grid_ ? "显示" : "隐藏") << std::endl;
                break;
        }
    }

private:
    cv::Scalar green_lower_;
    cv::Scalar green_upper_;
    cv::Mat current_frame_;
    cv::Mat green_mask_;
    cv::Mat bright_core_mask_;
    cv::Mat gradient_mask_;
    cv::Mat combined_mask_;
    int frame_counter_ = 0;
    int detection_mode_ = 2;  // 0:亮核检测, 1:梯度约束, 2:混合模式
    bool show_debug_info_ = false;
    bool show_grid_ = true;  // 默认显示网格线
    
    // 检测参数
    double circularity_threshold_ = 0.5;  // 降低阈值以提高检测率
    double min_area_ = 20.0;  // 降低最小面积
    double max_area_ = 5000.0;
    double min_radius_ = 3.0;  // 降低最小半径
    double max_radius_ = 80.0;
    
    // 亮核检测参数
    double brightness_threshold_low_ = 120.0;  // 降低亮度阈值
    double brightness_threshold_high_ = 255.0;
    
    // 梯度检测参数
    double gradient_threshold_low_ = 15.0;  // 降低梯度阈值
    double gradient_threshold_high_ = 255.0;
    
    // 形态学参数
    int morph_kernel_size_ = 3;
    
    void init_parameters() {
        // 初始化绿色范围（针对前哨站灯光） - 扩大范围
        green_lower_ = cv::Scalar(35, 50, 50);   // H: 35-85, S: 50-255, V: 50-255
        green_upper_ = cv::Scalar(85, 255, 255);
        
        print_parameters();
    }
    
    void print_parameters() {
        std::cout << "\n当前参数设置:" << std::endl;
        std::cout << "绿色HSV范围: H(" << green_lower_[0] << "-" << green_upper_[0] 
                  << ") S(" << green_lower_[1] << "-" << green_upper_[1] 
                  << ") V(" << green_lower_[2] << "-" << green_upper_[2] << ")" << std::endl;
        std::cout << "圆形度阈值: " << circularity_threshold_ << std::endl;
        std::cout << "检测模式: ";
        if (detection_mode_ == 0) std::cout << "亮核检测" << std::endl;
        else if (detection_mode_ == 1) std::cout << "梯度约束" << std::endl;
        else std::cout << "混合模式" << std::endl;
        std::cout << "调试信息: " << (show_debug_info_ ? "显示" : "隐藏") << std::endl;
        std::cout << "中心网格线: " << (show_grid_ ? "显示" : "隐藏") << std::endl;
        std::cout << std::endl;
    }
    
    // 绘制网格线：从中心点开始向四个方向每隔50像素画线
    void drawGridLines(cv::Mat& image) {
        int center_x = image.cols / 2;
        int center_y = image.rows / 2;
        
        // 网格线颜色（青色，比较明显）
        cv::Scalar grid_color(255, 255, 0);  // BGR: 青色
        
        // 设置线宽
        int line_thickness = 1;
        int dash_length = 10;  // 虚线段的长度
        int gap_length = 5;    // 虚线段之间的间隔
        
        // 从中心点开始，向右每隔50像素画一条垂直线（虚线）
        for (int x = center_x + 50; x < image.cols; x += 50) {
            // 画虚线
            for (int y = 0; y < image.rows; y += dash_length + gap_length) {
                int y_end = std::min(y + dash_length, image.rows);
                cv::line(image, 
                        cv::Point(x, y), 
                        cv::Point(x, y_end), 
                        grid_color, 
                        line_thickness);
            }
            // 标记坐标
            cv::putText(image, 
                       std::to_string(x - center_x), 
                       cv::Point(x, center_y - 15), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.4, 
                       grid_color, 
                       1);
        }
        
        // 从中心点开始，向左每隔50像素画一条垂直线（虚线）
        for (int x = center_x - 50; x >= 0; x -= 50) {
            // 画虚线
            for (int y = 0; y < image.rows; y += dash_length + gap_length) {
                int y_end = std::min(y + dash_length, image.rows);
                cv::line(image, 
                        cv::Point(x, y), 
                        cv::Point(x, y_end), 
                        grid_color, 
                        line_thickness);
            }
            // 标记坐标
            cv::putText(image, 
                       std::to_string(x - center_x), 
                       cv::Point(x, center_y - 15), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.4, 
                       grid_color, 
                       1);
        }
        
        // 从中心点开始，向下每隔50像素画一条水平线（虚线）
        for (int y = center_y + 50; y < image.rows; y += 50) {
            // 画虚线
            for (int x = 0; x < image.cols; x += dash_length + gap_length) {
                int x_end = std::min(x + dash_length, image.cols);
                cv::line(image, 
                        cv::Point(x, y), 
                        cv::Point(x_end, y), 
                        grid_color, 
                        line_thickness);
            }
            // 标记坐标
            cv::putText(image, 
                       std::to_string(y - center_y), 
                       cv::Point(center_x + 10, y + 5), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.4, 
                       grid_color, 
                       1);
        }
        
        // 从中心点开始，向上每隔50像素画一条水平线（虚线）
        for (int y = center_y - 50; y >= 0; y -= 50) {
            // 画虚线
            for (int x = 0; x < image.cols; x += dash_length + gap_length) {
                int x_end = std::min(x + dash_length, image.cols);
                cv::line(image, 
                        cv::Point(x, y), 
                        cv::Point(x_end, y), 
                        grid_color, 
                        line_thickness);
            }
            // 标记坐标
            cv::putText(image, 
                       std::to_string(y - center_y), 
                       cv::Point(center_x + 10, y + 5), 
                       cv::FONT_HERSHEY_SIMPLEX, 
                       0.4, 
                       grid_color, 
                       1);
        }
        
        // 绘制中心十字线（实线，比网格线更粗更明显）
        cv::Scalar center_color(0, 255, 255);  // 黄色中心线
        int center_line_thickness = 2;
        
        // 绘制水平中心线（从左到右）
        cv::line(image, 
                cv::Point(0, center_y), 
                cv::Point(image.cols, center_y), 
                center_color, 
                center_line_thickness);
        
        // 绘制垂直中心线（从顶部到底部）
        cv::line(image, 
                cv::Point(center_x, 0), 
                cv::Point(center_x, image.rows), 
                center_color, 
                center_line_thickness);
        
        // 绘制中心点（大圆点）
        cv::circle(image, 
                  cv::Point(center_x, center_y), 
                  8, 
                  center_color, 
                  2);  // 外圆
        cv::circle(image, 
                  cv::Point(center_x, center_y), 
                  4, 
                  cv::Scalar(0, 0, 255),  // 红色内圆
                  -1);  // 填充圆
        
        // 在图像角落显示中心坐标和网格信息
        std::string center_coord = "Center: (" + std::to_string(center_x) + ", " + std::to_string(center_y) + ")";
        cv::putText(image, 
                   center_coord, 
                   cv::Point(10, image.rows - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, 
                   cv::Scalar(255, 255, 255), 
                   1);
        
        std::string grid_info = "Grid: 50px spacing";
        cv::putText(image, 
                   grid_info, 
                   cv::Point(10, image.rows - 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, 
                   cv::Scalar(255, 255, 255), 
                   1);
        
        // 绘制坐标轴标签
        cv::putText(image, 
                   "→ X", 
                   cv::Point(image.cols - 40, center_y - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, 
                   grid_color, 
                   1);
        
        cv::putText(image, 
                   "↓ Y", 
                   cv::Point(center_x + 10, 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, 
                   grid_color, 
                   1);
    }
    
    // 优化的预处理函数
    cv::Mat preprocessFrame(const cv::Mat& frame) {
        cv::Mat processed;
        
        // 使用高斯模糊，计算量比双边滤波小
        cv::GaussianBlur(frame, processed, cv::Size(5, 5), 1.5);
        
        return processed;
    }
    
    // 亮核检测：检测最亮的区域
    cv::Mat detectBrightCore(const cv::Mat& frame) {
        cv::Mat gray, bright_core;
        
        // 转换为灰度图
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        // 高斯模糊减少噪声
        cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.5);
        
        // 阈值化，提取亮核区域
        cv::inRange(gray, brightness_threshold_low_, brightness_threshold_high_, bright_core);
        
        // 简化的形态学操作
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                  cv::Size(morph_kernel_size_, morph_kernel_size_));
        cv::morphologyEx(bright_core, bright_core, cv::MORPH_CLOSE, kernel);
        
        return bright_core;
    }
    
    // 优化的梯度检测
    cv::Mat detectGradient(const cv::Mat& frame) {
        cv::Mat gray, gradient, gradient_mask;
        
        // 转换为灰度图
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        // 高斯模糊
        cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.5);
        
        // 使用更高效的Laplacian算子计算梯度
        cv::Laplacian(gray, gradient, CV_16S, 3);
        cv::convertScaleAbs(gradient, gradient);
        
        // 阈值化，得到梯度掩码
        cv::inRange(gradient, gradient_threshold_low_, gradient_threshold_high_, gradient_mask);
        
        return gradient_mask;
    }
    
    // 颜色分割：提取绿色区域
    cv::Mat detectGreenColor(const cv::Mat& frame) {
        cv::Mat hsv, color_mask;
        
        // 转换为HSV颜色空间
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // 提取绿色区域
        cv::inRange(hsv, green_lower_, green_upper_, color_mask);
        
        return color_mask;
    }
    
    // 计算圆形度
    double calculateCircularity(const std::vector<cv::Point>& contour) {
        double area = cv::contourArea(contour);
        double perimeter = cv::arcLength(contour, true);
        
        if (perimeter == 0) return 0;
        
        double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        return circularity;
    }
    
    // 主检测函数
    cv::Mat detectGreenCircles(const cv::Mat& frame) {
        // 保存当前帧
        frame.copyTo(current_frame_);
        
        // 预处理
        cv::Mat processed = preprocessFrame(frame);
        
        // 检测颜色（绿色区域）
        cv::Mat color_mask = detectGreenColor(processed);
        
        // 保存绿色掩码用于调试
        color_mask.copyTo(green_mask_);
        
        // 根据检测模式选择不同的处理策略
        cv::Mat detection_mask;
        
        switch (detection_mode_) {
            case 0:  // 亮核检测模式：亮核 + 颜色
                bright_core_mask_ = detectBrightCore(processed);
                cv::bitwise_and(color_mask, bright_core_mask_, detection_mask);
                break;
                
            case 1:  // 梯度约束模式：颜色 + 梯度
                gradient_mask_ = detectGradient(processed);
                cv::bitwise_and(color_mask, gradient_mask_, detection_mask);
                break;
                
            case 2:  // 混合模式：颜色 + (亮核或梯度)
            default:
                // 并行计算亮核和梯度
                bright_core_mask_ = detectBrightCore(processed);
                gradient_mask_ = detectGradient(processed);
                
                // 合并掩码
                cv::Mat temp;
                cv::bitwise_or(bright_core_mask_, gradient_mask_, temp);
                cv::bitwise_and(color_mask, temp, detection_mask);
                break;
        }
        
        // 形态学操作优化掩码
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                  cv::Size(morph_kernel_size_, morph_kernel_size_));
        
        // 先闭运算填充小孔
        cv::morphologyEx(detection_mask, detection_mask, cv::MORPH_CLOSE, kernel);
        
        // 再开运算去除小噪点
        cv::morphologyEx(detection_mask, detection_mask, cv::MORPH_OPEN, kernel);
        
        // 复制原始图像用于显示结果
        cv::Mat result;
        frame.copyTo(result);
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(detection_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        int detected_count = 0;
        
        // 批量处理轮廓以提高效率
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            
            // 面积过滤
            if (area < min_area_ || area > max_area_) {
                continue;
            }
            
            // 使用最小外接圆
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            
            // 半径过滤
            if (radius < min_radius_ || radius > max_radius_) {
                continue;
            }
            
            // 圆形度检测（放在后面，因为计算成本较高）
            double circularity = calculateCircularity(contour);
            if (circularity < circularity_threshold_) {
                continue;
            }
            
            // 边界矩形比例检查（快速排除明显非圆形）
            cv::Rect rect = cv::boundingRect(contour);
            double aspect_ratio = static_cast<double>(rect.width) / rect.height;
            if (aspect_ratio < 0.6 || aspect_ratio > 1.4) {
                continue;
            }
            
            // 计算轮廓的矩以获取质心
            cv::Moments M = cv::moments(contour);
            if (M.m00 == 0) continue;
            
            cv::Point centroid(
                static_cast<int>(M.m10 / M.m00), 
                static_cast<int>(M.m01 / M.m00)
            );
            
            // 绘制检测结果 - 以圆心为中心绘制矩形
            drawDetectionResult(result, contour, center, radius, circularity, area, centroid);
            
            detected_count++;
            
            // 输出检测信息
            if (show_debug_info_) {
                std::cout << "✓ 检测到绿色圆形灯 #" << detected_count 
                          << " - 半径: " << radius 
                          << ", 圆度: " << circularity 
                          << ", 面积: " << area 
                          << ", 中心: (" << center.x << ", " << center.y << ")" << std::endl;
                
                // 计算相对于图像中心的坐标
                int img_center_x = result.cols / 2;
                int img_center_y = result.rows / 2;
                int dx = center.x - img_center_x;
                int dy = center.y - img_center_y;
                
                std::cout << "   相对中心: (" << dx << ", " << dy << ")" << std::endl;
            }
        }
        
        // 在图像上显示统计信息
        std::string stats = "检测到 " + std::to_string(detected_count) + " 个绿色圆形灯";
        cv::putText(result, stats, 
                   cv::Point(10, result.rows - 50), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // 显示当前参数
        std::string mode_str;
        if (detection_mode_ == 0) mode_str = "亮核";
        else if (detection_mode_ == 1) mode_str = "梯度";
        else mode_str = "混合";
        
        std::string param_info = "模式:" + mode_str + " 圆度:" + std::to_string(static_cast<int>(circularity_threshold_ * 100)) + "%";
        cv::putText(result, param_info, 
                   cv::Point(10, result.rows - 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1);
        
        return result;
    }
    
    // 优化的绘制检测结果函数 - 以圆心为中心绘制矩形
    void drawDetectionResult(cv::Mat& result, const std::vector<cv::Point>& contour,
                            const cv::Point2f& center, float radius, 
                            double circularity, double area, const cv::Point& centroid) {
        // 计算矩形尺寸（以半径的2倍作为矩形边长的一半）
        int rect_size = static_cast<int>(radius * 1.5);  // 矩形边长的一半
        
        // 计算矩形左上角和右下角坐标
        cv::Point rect_top_left(static_cast<int>(center.x - rect_size), 
                               static_cast<int>(center.y - rect_size));
        cv::Point rect_bottom_right(static_cast<int>(center.x + rect_size), 
                                   static_cast<int>(center.y + rect_size));
        
        // 确保矩形在图像范围内
        rect_top_left.x = std::max(0, rect_top_left.x);
        rect_top_left.y = std::max(0, rect_top_left.y);
        rect_bottom_right.x = std::min(result.cols - 1, rect_bottom_right.x);
        rect_bottom_right.y = std::min(result.rows - 1, rect_bottom_right.y);
        
        // 绘制矩形（蓝色边框，厚度2）
        cv::rectangle(result, rect_top_left, rect_bottom_right, 
                     cv::Scalar(255, 0, 0), 2);
        
        // 绘制矩形中心到四个角的对角线（橙色）
        cv::line(result, center, cv::Point(rect_top_left.x, rect_top_left.y), cv::Scalar(0, 100, 255), 1);
        cv::line(result, center, cv::Point(rect_bottom_right.x, rect_top_left.y), cv::Scalar(0, 100, 255), 1);
        cv::line(result, center, cv::Point(rect_top_left.x, rect_bottom_right.y), cv::Scalar(0, 100, 255), 1);
        cv::line(result, center, cv::Point(rect_bottom_right.x, rect_bottom_right.y), cv::Scalar(0, 100, 255), 1);
        
        // 绘制轮廓（绿色，可选）
        cv::drawContours(result, std::vector<std::vector<cv::Point>>{contour}, 0, 
                        cv::Scalar(0, 255, 0), 1);
        
        // 绘制圆心（红色点）
        cv::circle(result, center, 3, cv::Scalar(0, 0, 255), -1);
        
        // 绘制质心（黄色点）
        cv::circle(result, centroid, 2, cv::Scalar(0, 255, 255), -1);
        
        // 绘制连接圆心和质心的线（应该很接近）
        cv::line(result, center, centroid, cv::Scalar(255, 255, 0), 1);
        
        // 显示检测信息（格式：R:半径 C:圆形度%）
        std::string info = "R:" + std::to_string(static_cast<int>(radius)) + 
                          " C:" + std::to_string(static_cast<int>(circularity * 100)) + "%";
        
        // 将文本放在矩形上方
        cv::Point text_pos(
            std::max(0, rect_top_left.x), 
            std::max(0, rect_top_left.y - 10)
        );
        
        cv::putText(result, info, 
                   text_pos,
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        // 在矩形内部中心显示一个点表示准确的中心位置
        cv::circle(result, center, 1, cv::Scalar(0, 255, 255), -1);
    }
    
    void saveCurrentFrame() {
        if (!current_frame_.empty()) {
            std::string filename = "detection_" + std::to_string(frame_counter_++) + ".jpg";
            cv::imwrite(filename, current_frame_);
            std::cout << "已保存图像: " << filename << std::endl;
        }
    }
    
    void resetParameters() {
        init_parameters();
        circularity_threshold_ = 0.5;
        min_area_ = 20.0;
        min_radius_ = 3.0;
        brightness_threshold_low_ = 120.0;
        brightness_threshold_high_ = 255.0;
        gradient_threshold_low_ = 15.0;
        gradient_threshold_high_ = 255.0;
        detection_mode_ = 2;
        show_grid_ = true;
        std::cout << "参数已重置" << std::endl;
    }
};

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
        
        // 创建绿色圆形检测器
        GreenCircleDetector detector;
        
        std::cout << "前哨站绿色圆形灯识别系统启动" << std::endl;
        std::cout << "==========================================" << std::endl;
        
        // 性能统计变量
        int frame_count = 0;
        double total_fps = 0.0;
        auto start_total_time = std::chrono::high_resolution_clock::now();
        
        while (true) {
            // 捕获图像
            cv::Mat frame = camera.Grab();
            
            if (!frame.empty()) {
                // 处理图像
                detector.processFrame(frame);
                
                // 统计帧率
                frame_count++;
            } else {
                std::cout << "获取图像失败!" << std::endl;
            }
            
            // 检查按键
            int key = cv::waitKey(1);  // 使用更短的等待时间
            
            // 处理按键输入
            detector.handleKeyPress(key);
            
            // 按ESC键或'q'键退出
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
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
        
    } catch (const std::exception& e) {
        std::cerr << "错误发生: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}