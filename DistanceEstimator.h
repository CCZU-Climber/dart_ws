#ifndef DISTANCEESTIMATOR_H
#define DISTANCEESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "DetectionResult.h"

class DistanceEstimator {
private:
    float focal_length_;           // 焦距（像素）
    float real_world_diameter_;    // 目标真实直径（米）
    
public:
    DistanceEstimator();
    
    // 设置目标参数
    void setTargetParameters(float focal_length, float real_world_diameter);
    
    // 使用像素直径计算距离
    float estimateDistanceFromDiameter(float pixel_diameter) const;
    
    // 使用像素半径计算距离（更常用）
    float estimateDistanceFromRadius(float pixel_radius) const;
    
    // 批量处理
    void estimateDistances(std::vector<DetectionResult>& results) const;
    
    // 验证距离是否有效
    bool isDistanceValid(float distance) const;
    
    // 格式化距离显示
    std::string formatDistance(float distance) const;
    
    // 获取焦距
    float getFocalLength() const { return focal_length_; }
    
    // 重命名 setCameraParameters 为兼容方法
    void setCameraParameters(float focal_length, float real_world_diameter) { 
        setTargetParameters(focal_length, real_world_diameter); 
    }
};

#endif // DISTANCEESTIMATOR_H