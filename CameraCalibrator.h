#ifndef CAMERA_CALIBRATOR_H
#define CAMERA_CALIBRATOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "HikCam.h"  // 确保包含HikCam头文件

class CameraCalibrator {
public:
    CameraCalibrator();
    
    /**
     * 设置标定板参数
     * @param boardWidth 棋盘格宽度（格子数）
     * @param boardHeight 棋盘格高度（格子数）
     * @param squareSizeM 每个格子的实际尺寸（米）
     */
    void setBoardParams(int boardWidth = 9, int boardHeight = 12, float squareSizeM = 0.03f);
    
    /**
     * 从相机采集标定图像
     * @param camera 海康相机实例
     * @param saveDir 保存图像的目录
     * @param minImages 最小采集图像数量
     * @param maxImages 最大采集图像数量
     * @return 是否成功采集到足够图像
     */
    bool captureImages(sensor::camera::HikCam& camera, const std::string& saveDir = "calibration_images", 
                      int minImages = 15, int maxImages = 30);
    
    /**
     * 执行相机标定
     * @param imagesDir 包含标定图像的目录
     * @param outputFile 输出标定参数文件
     * @return 是否标定成功
     */
    bool calibrate(const std::string& imagesDir = "calibration_images",  // 🔧 修正：去掉多余的 std::
                  const std::string& outputFile = "camera_params.yml");
    
    /**
     * 检查文件是否存在
     */
    static bool fileExists(const std::string& filePath);
    
    /**
     * 加载标定参数
     */
    static bool loadParams(const std::string& fileName, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
    
    /**
     * 畸变校正
     */
    static cv::Mat undistortImage(const cv::Mat& src, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

private:
    int boardWidth_;  // 存储内角点数
    int boardHeight_; // 存储内角点数
    float squareSizeM_;
    std::vector<std::vector<cv::Point2f>> imagePoints_;    std::vector<std::vector<cv::Point3f>> objectPoints_;
    std::vector<cv::Point3f> calcBoardCornerPositions() const;  // 🔧 添加：函数声明
};

#endif // CAMERA_CALIBRATOR_H