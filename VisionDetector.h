#ifndef VISIONDETECTOR_H
#define VISIONDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class VisionDetector {
private:
    // 颜色范围参数
    cv::Scalar green_lower_;
    cv::Scalar green_upper_;
    
    // 检测参数
    double circularity_threshold_;
    double min_area_;
    double max_area_;
    double min_radius_;
    double max_radius_;
    
    // 亮核检测参数
    double brightness_threshold_low_;
    double brightness_threshold_high_;
    
    // 梯度检测参数
    double gradient_threshold_low_;
    double gradient_threshold_high_;
    
    // 形态学参数
    int morph_kernel_size_;
    
    // 检测模式
    int detection_mode_;
    
    // 当前帧
    cv::Mat current_frame_;
    
    // 中间结果
    cv::Mat green_mask_;
    cv::Mat bright_core_mask_;
    cv::Mat gradient_mask_;
    cv::Mat combined_mask_;
    
    // 调试信息
    bool show_debug_info_;
    
public:
    VisionDetector();
    
    // 设置检测模式
    void setDetectionMode(int mode);
    
    // 设置圆形度阈值
    void setCircularityThreshold(double threshold);
    
    // 设置调试信息显示
    void setDebugInfo(bool show);
    
    // 检测绿色圆形并返回检测到的圆形中心
    cv::Mat detectGreenCircles(const cv::Mat& frame, std::vector<cv::Point2f>& detected_circles);
    
    // 获取当前帧
    cv::Mat getCurrentFrame() const;
    
    // 获取检测参数
    double getCircularityThreshold() const;
    int getDetectionMode() const;
    
    // 获取中间结果（用于调试显示）
    cv::Mat getGreenMask() const;
    cv::Mat getBrightCoreMask() const;
    cv::Mat getGradientMask() const;
    cv::Mat getCombinedMask() const;
    
    // 保存当前帧
    void saveCurrentFrame(int& frame_counter);
    
private:
    // 初始化参数
    void init_parameters();
    
    // 优化的预处理函数
    cv::Mat preprocessFrame(const cv::Mat& frame);
    
    // 亮核检测：检测最亮的区域
    cv::Mat detectBrightCore(const cv::Mat& frame);
    
    // 优化的梯度检测
    cv::Mat detectGradient(const cv::Mat& frame);
    
    // 颜色分割：提取绿色区域
    cv::Mat detectGreenColor(const cv::Mat& frame);
    
    // 计算圆形度
    double calculateCircularity(const std::vector<cv::Point>& contour);
    
    // 绘制检测结果
    void drawDetectionResult(cv::Mat& result, const std::vector<cv::Point>& contour,
                            const cv::Point2f& center, float radius, 
                            double circularity, double area, const cv::Point& centroid);
};

#endif // VISIONDETECTOR_H