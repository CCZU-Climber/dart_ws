#ifndef DETECTIONRESULT_H
#define DETECTIONRESULT_H

#include <opencv2/opencv.hpp>

struct DetectionResult {
    cv::Vec3f circle;      // x, y, radius (像素半径)
    double confidence;     // 检测置信度
    float distance;        // 距离（米）
    bool has_distance;     // 是否有有效距离
    
    // 新增字段：像素直径（可选，半径×2即可）
    float pixel_diameter;  // 像素直径，方便调试
    
    DetectionResult() : confidence(0.0), distance(-1.0f), has_distance(false), pixel_diameter(0.0f) {}
};

#endif // DETECTIONRESULT_H