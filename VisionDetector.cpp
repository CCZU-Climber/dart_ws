#include "VisionDetector.h"
#include <iostream>
#include <algorithm>

VisionDetector::VisionDetector() : show_debug_info_(false) {
    init_parameters();
}

void VisionDetector::init_parameters() {
    // 初始化绿色范围（针对前哨站灯光） - 扩大范围
    green_lower_ = cv::Scalar(35, 50, 50);   // H: 35-85, S: 50-255, V: 50-255
    green_upper_ = cv::Scalar(85, 255, 255);
    
    // 检测参数
    circularity_threshold_ = 0.5;  // 降低阈值以提高检测率
    min_area_ = 20.0;  // 降低最小面积
    max_area_ = 5000.0;
    min_radius_ = 3.0;  // 降低最小半径
    max_radius_ = 80.0;
    
    // 亮核检测参数
    brightness_threshold_low_ = 120.0;  // 降低亮度阈值
    brightness_threshold_high_ = 255.0;
    
    // 梯度检测参数
    gradient_threshold_low_ = 15.0;  // 降低梯度阈值
    gradient_threshold_high_ = 255.0;
    
    // 形态学参数
    morph_kernel_size_ = 3;
    
    // 检测模式
    detection_mode_ = 2;  // 混合模式
}

void VisionDetector::setDetectionMode(int mode) {
    detection_mode_ = mode;
}

void VisionDetector::setCircularityThreshold(double threshold) {
    circularity_threshold_ = threshold;
}

void VisionDetector::setDebugInfo(bool show) {
    show_debug_info_ = show;
}

cv::Mat VisionDetector::detectGreenCircles(const cv::Mat& frame, std::vector<cv::Point2f>& detected_circles) {
    detected_circles.clear();
    frame.copyTo(current_frame_);
    cv::Mat processed = preprocessFrame(frame);
    cv::Mat color_mask = detectGreenColor(processed);
    color_mask.copyTo(green_mask_);
    
    cv::Mat detection_mask;
    switch (detection_mode_) {
        case 0:
            bright_core_mask_ = detectBrightCore(processed);
            cv::bitwise_and(color_mask, bright_core_mask_, detection_mask);
            break;
        case 1:
            gradient_mask_ = detectGradient(processed);
            cv::bitwise_and(color_mask, gradient_mask_, detection_mask);
            break;
        case 2:
        default:
            bright_core_mask_ = detectBrightCore(processed);
            gradient_mask_ = detectGradient(processed);
            cv::Mat temp;
            cv::bitwise_or(bright_core_mask_, gradient_mask_, temp);
            cv::bitwise_and(color_mask, temp, detection_mask);
            break;
    }
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                              cv::Size(morph_kernel_size_, morph_kernel_size_));
    cv::morphologyEx(detection_mask, detection_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(detection_mask, detection_mask, cv::MORPH_OPEN, kernel);
    detection_mask.copyTo(combined_mask_);
    
    cv::Mat result;
    frame.copyTo(result);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(detection_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    int detected_count = 0;
    cv::Point2f best_circle_center(-1, -1);
    float best_circle_radius = 0;
    double best_circularity = 0;
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_area_ || area > max_area_) continue;
        
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        
        if (radius < min_radius_ || radius > max_radius_) continue;
        
        double circularity = calculateCircularity(contour);
        if (circularity < circularity_threshold_) continue;
        
        cv::Rect rect = cv::boundingRect(contour);
        double aspect_ratio = static_cast<double>(rect.width) / rect.height;
        if (aspect_ratio < 0.6 || aspect_ratio > 1.4) continue;
        
        cv::Moments M = cv::moments(contour);
        if (M.m00 == 0) continue;
        
        cv::Point centroid(
            static_cast<int>(M.m10 / M.m00), 
            static_cast<int>(M.m01 / M.m00)
        );
        
        drawDetectionResult(result, contour, center, radius, circularity, area, centroid);
        
        if (detected_circles.empty() || circularity > best_circularity) {
            best_circle_center = center;
            best_circle_radius = radius;
            best_circularity = circularity;
        }
        
        detected_count++;
        
        if (show_debug_info_) {
            std::cout << "✓ 检测到绿色圆形灯 #" << detected_count 
                      << " - 半径: " << radius 
                      << ", 圆度: " << circularity 
                      << ", 面积: " << area 
                      << ", 中心: (" << center.x << ", " << center.y << ")" << std::endl;
        }
    }
    
    if (best_circle_center.x >= 0 && best_circle_center.y >= 0) {
        detected_circles.push_back(best_circle_center);
    }
    
    std::string stats = "检测到 " + std::to_string(detected_count) + " 个绿色圆形灯";
    cv::putText(result, stats, 
               cv::Point(10, result.rows - 50), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    
    return result;
}

cv::Mat VisionDetector::getCurrentFrame() const {
    return current_frame_;
}

double VisionDetector::getCircularityThreshold() const {
    return circularity_threshold_;
}

int VisionDetector::getDetectionMode() const {
    return detection_mode_;
}

cv::Mat VisionDetector::getGreenMask() const {
    return green_mask_;
}

cv::Mat VisionDetector::getBrightCoreMask() const {
    return bright_core_mask_;
}

cv::Mat VisionDetector::getGradientMask() const {
    return gradient_mask_;
}

cv::Mat VisionDetector::getCombinedMask() const {
    return combined_mask_;
}

void VisionDetector::saveCurrentFrame(int& frame_counter) {
    if (!current_frame_.empty()) {
        std::string filename = "detection_" + std::to_string(frame_counter++) + ".jpg";
        cv::imwrite(filename, current_frame_);
        std::cout << "已保存图像: " << filename << std::endl;
    }
}

// 优化的预处理函数
cv::Mat VisionDetector::preprocessFrame(const cv::Mat& frame) {
    cv::Mat processed;
    cv::GaussianBlur(frame, processed, cv::Size(5, 5), 1.5);
    return processed;
}

// 亮核检测：检测最亮的区域
cv::Mat VisionDetector::detectBrightCore(const cv::Mat& frame) {
    cv::Mat gray, bright_core;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.5);
    cv::inRange(gray, brightness_threshold_low_, brightness_threshold_high_, bright_core);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                              cv::Size(morph_kernel_size_, morph_kernel_size_));
    cv::morphologyEx(bright_core, bright_core, cv::MORPH_CLOSE, kernel);
    return bright_core;
}

// 优化的梯度检测
cv::Mat VisionDetector::detectGradient(const cv::Mat& frame) {
    cv::Mat gray, gradient, gradient_mask;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.5);
    cv::Laplacian(gray, gradient, CV_16S, 3);
    cv::convertScaleAbs(gradient, gradient);
    cv::inRange(gradient, gradient_threshold_low_, gradient_threshold_high_, gradient_mask);
    return gradient_mask;
}

// 颜色分割：提取绿色区域
cv::Mat VisionDetector::detectGreenColor(const cv::Mat& frame) {
    cv::Mat hsv, color_mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, green_lower_, green_upper_, color_mask);
    return color_mask;
}

// 计算圆形度
double VisionDetector::calculateCircularity(const std::vector<cv::Point>& contour) {
    double area = cv::contourArea(contour);
    double perimeter = cv::arcLength(contour, true);
    if (perimeter == 0) return 0;
    double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
    return circularity;
}

// 绘制检测结果
void VisionDetector::drawDetectionResult(cv::Mat& result, const std::vector<cv::Point>& contour,
                                        const cv::Point2f& center, float radius, 
                                        double circularity, double area, const cv::Point& centroid) {
    // 使用参数避免编译器警告
    (void)contour;
    (void)circularity;
    (void)area;
    (void)centroid;
    
    int rect_size = static_cast<int>(radius * 1.5);
    cv::Point rect_top_left(static_cast<int>(center.x - rect_size), 
                           static_cast<int>(center.y - rect_size));
    cv::Point rect_bottom_right(static_cast<int>(center.x + rect_size), 
                               static_cast<int>(center.y + rect_size));
    
    rect_top_left.x = std::max(0, rect_top_left.x);
    rect_top_left.y = std::max(0, rect_top_left.y);
    rect_bottom_right.x = std::min(result.cols - 1, rect_bottom_right.x);
    rect_bottom_right.y = std::min(result.rows - 1, rect_bottom_right.y);
    
    cv::rectangle(result, rect_top_left, rect_bottom_right, 
                 cv::Scalar(255, 0, 0), 2);
    
    cv::line(result, center, cv::Point(rect_top_left.x, rect_top_left.y), cv::Scalar(0, 100, 255), 1);
    cv::line(result, center, cv::Point(rect_bottom_right.x, rect_top_left.y), cv::Scalar(0, 100, 255), 1);
    cv::line(result, center, cv::Point(rect_top_left.x, rect_bottom_right.y), cv::Scalar(0, 100, 255), 1);
    cv::line(result, center, cv::Point(rect_bottom_right.x, rect_bottom_right.y), cv::Scalar(0, 100, 255), 1);
    
    cv::circle(result, center, 3, cv::Scalar(0, 0, 255), -1);
    
    std::string info = "R:" + std::to_string(static_cast<int>(radius)) + 
                      " C:" + std::to_string(static_cast<int>(circularity * 100)) + "%";
    
    cv::Point text_pos(
        std::max(0, rect_top_left.x), 
        std::max(0, rect_top_left.y - 10)
    );
    
    cv::putText(result, info, 
               text_pos,
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    cv::circle(result, center, 1, cv::Scalar(0, 255, 255), -1);
}