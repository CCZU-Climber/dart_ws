#include "DistanceEstimator.h"
#include <iostream>

DistanceEstimator::DistanceEstimator() 
    : focal_length_(1000.0f), real_world_diameter_(0.055f) {} // 默认焦距和55mm直径

void DistanceEstimator::setTargetParameters(float focal_length, float real_world_diameter) {
    focal_length_ = focal_length;
    real_world_diameter_ = real_world_diameter;
    std::cout << "[DistanceEstimator] 设置参数: 焦距=" << focal_length_ 
              << "px, 真实直径=" << real_world_diameter_ * 1000 << "mm" << std::endl;
}

float DistanceEstimator::estimateDistanceFromDiameter(float pixel_diameter) const {
    if (pixel_diameter <= 0 || focal_length_ <= 0 || real_world_diameter_ <= 0) {
        return -1.0f;
    }
    
    // 正确的透视投影公式: 距离 = (焦距 × 真实尺寸) / 像素尺寸
    // 这里的焦距单位是像素，真实尺寸单位是米，像素尺寸单位是像素
    float distance = (focal_length_ * real_world_diameter_) / pixel_diameter;
    return distance;
}

float DistanceEstimator::estimateDistanceFromRadius(float pixel_radius) const {
    if (pixel_radius <= 0 || focal_length_ <= 0 || real_world_diameter_ <= 0) {
        return -1.0f;
    }
    
    // 如果使用半径计算，需要将直径转换为半径
    float pixel_diameter = 2.0f * pixel_radius;
    float distance = (focal_length_ * real_world_diameter_) / pixel_diameter;
    return distance;
}

void DistanceEstimator::estimateDistances(std::vector<DetectionResult>& results) const {
    for (auto& res : results) {
        if (res.pixel_diameter > 0) {
            // 使用像素直径计算距离
            res.distance = estimateDistanceFromDiameter(res.pixel_diameter);
            res.has_distance = isDistanceValid(res.distance);
        } else if (res.circle[2] > 0) {  // 如果只有半径信息
            res.distance = estimateDistanceFromRadius(res.circle[2]);
            res.has_distance = isDistanceValid(res.distance);
        }
    }
}

bool DistanceEstimator::isDistanceValid(float distance) const {
    return (distance > 0.1f && distance < 100.0f);  // 0.1m - 100m 有效范围
}

std::string DistanceEstimator::formatDistance(float distance) const {
    if (!isDistanceValid(distance)) {
        return "N/A";
    }
    return std::to_string(static_cast<int>(distance * 100)) + "cm"; // 返回厘米
}