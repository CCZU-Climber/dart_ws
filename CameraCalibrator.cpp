#include "CameraCalibrator.h"
#include <iostream>
#include <filesystem>  // 🔧 修正：添加 filesystem 头文件
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <thread>      // 🔧 修正：添加 thread 头文件
#include <chrono>      // 🔧 修正：添加 chrono 头文件

CameraCalibrator::CameraCalibrator() 
    : boardWidth_(8), boardHeight_(11), squareSizeM_(0.03f) {  // 🔧 修改：默认内角点8x11（对应9x12格子）
    // 初始化3D对象点
    objectPoints_.clear();
    std::vector<cv::Point3f> objp;
    for(int i = 0; i < boardHeight_; ++i) {
        for(int j = 0; j < boardWidth_; ++j) {
            objp.push_back(cv::Point3f(j * squareSizeM_, i * squareSizeM_, 0));
        }
    }
    // 第一帧会添加这些点
}

void CameraCalibrator::setBoardParams(int boardWidth, int boardHeight, float squareSizeM) {
    // 🔧 修改：输入格子数，内部转换为内角点数
    boardWidth_ = boardWidth - 1;   // 格子数9 -> 内角点8
    boardHeight_ = boardHeight - 1; // 格子数12 -> 内角点11
    squareSizeM_ = squareSizeM;
    
    // 重新初始化对象点
    std::vector<cv::Point3f> objp;
    for(int i = 0; i < boardHeight_; ++i) {
        for(int j = 0; j < boardWidth_; ++j) {
            objp.push_back(cv::Point3f(j * squareSizeM_, i * squareSizeM_, 0));
        }
    }
    // 清除现有对象点，新的会在captureImages中添加
    objectPoints_.clear();
}

bool CameraCalibrator::captureImages(sensor::camera::HikCam& camera, const std::string& saveDir, 
                                    int minImages, int maxImages) {
    std::cout << "开始采集标定图像..." << std::endl;
    std::cout << "按 's' 保存当前帧，按 'q' 退出采集" << std::endl;
    
    // 创建保存目录
    namespace fs = std::filesystem;  // 🔧 修正：使用命名空间别名
    if (!fs::exists(saveDir)) {
        fs::create_directory(saveDir);
    }
    
    int imageCount = 0;
    cv::namedWindow("Calibration Capture", cv::WINDOW_NORMAL);
    
    while (imageCount < maxImages) {
        cv::Mat frame = camera.Grab();
        
        if (frame.empty()) {
            std::cerr << "获取帧失败，重试中..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // 处理图像
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, cv::Size(boardWidth_, boardHeight_), corners,
                                             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK);
        
        cv::Mat displayFrame = frame.clone();
        
        if (found) {
            // 绘制角点
            cv::drawChessboardCorners(displayFrame, cv::Size(boardWidth_, boardHeight_), corners, found);
            cv::putText(displayFrame, "Chessboard Found! Press 's' to save", cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            std::cout << "找到棋盘格！按 's' 保存，按 'q' 退出" << std::endl;
        } else {
            // 🔧 新增：即使未找到，也显示状态和提示
            cv::putText(displayFrame, "Chessboard Not Found. Press 's' to save anyway", cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            cv::putText(displayFrame, "Check lighting, angle, and distance", cv::Point(10, 70), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            std::cout << "未找到棋盘格。按 's' 强制保存，按 'q' 退出" << std::endl;
            
            // 可选：尝试绘制部分检测到的角点（如果 corners 不为空，但 found 为 false）
            if (!corners.empty()) {
                cv::drawChessboardCorners(displayFrame, cv::Size(boardWidth_, boardHeight_), corners, false);
                cv::putText(displayFrame, "Partial detection", cv::Point(10, 110), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
            }
        }
        
        // 显示图像
        cv::imshow("Calibration Capture", displayFrame);
        
        // 🔧 修复：无论是否找到棋盘格，都处理按键
        int key = cv::waitKey(30);
        if (key == 'q' || key == 'Q' || key == 27) {  // 27 是 ESC
            std::cout << "退出采集" << std::endl;
            break;
        } else if (key == 's' || key == 'S') {
            // 保存图像
            std::string filename = saveDir + "/calibration_" + std::to_string(imageCount + 1) + ".jpg";
            if (cv::imwrite(filename, frame)) {  // 保存原始帧
                imagePoints_.push_back(corners);
                objectPoints_.push_back(calcBoardCornerPositions());
                imageCount++;
                std::cout << "✅ 保存图像: " << filename << " (总计: " << imageCount << ")" << std::endl;
            } else {
                std::cerr << "❌ 保存失败: " << filename << std::endl;
            }
        }
    }
    
    cv::destroyWindow("Calibration Capture");
    
    if (imageCount < minImages) {
        std::cerr << "采集图像数量不足！需要至少 " << minImages << " 张，实际采集 " << imageCount << " 张" << std::endl;
        return false;
    }
    
    std::cout << "成功采集 " << imageCount << " 张标定图像" << std::endl;
    return true;
}

bool CameraCalibrator::calibrate(const std::string& imagesDir, const std::string& outputFile) {
    (void)imagesDir;  // 🔧 添加：消除未使用参数警告
    if (imagePoints_.empty() || objectPoints_.empty()) {
        std::cerr << "没有采集到标定图像数据！" << std::endl;
        return false;
    }
    
    if (imagePoints_.size() != objectPoints_.size()) {
        std::cerr << "图像点和对象点数量不匹配！" << std::endl;
        return false;
    }
    
    std::cout << "开始相机标定，使用 " << imagePoints_.size() << " 张图像..." << std::endl;
    
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    
    // 执行标定
    double rms = cv::calibrateCamera(objectPoints_, imagePoints_, 
                                    cv::Size(2448, 2048),  // 使用你的相机分辨率
                                    cameraMatrix, distCoeffs,
                                    rvecs, tvecs,
                                    cv::CALIB_FIX_ASPECT_RATIO);
    
    std::cout << "重投影误差 RMS: " << rms << std::endl;
    
    if (rms > 2.0) {
        std::cerr << "警告：重投影误差较高 (" << rms << ")，标定质量可能不佳" << std::endl;
    }
    
    // 保存标定参数
    cv::FileStorage fs(outputFile, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "无法打开文件保存标定参数: " << outputFile << std::endl;
        return false;
    }
    
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs << "rms" << rms;
    fs << "imageWidth" << 2448;
    fs << "imageHeight" << 2048;
    fs.release();
    
    std::cout << "标定参数已保存到: " << outputFile << std::endl;
    std::cout << "相机矩阵:\n" << cameraMatrix << std::endl;
    std::cout << "畸变系数:\n" << distCoeffs << std::endl;
    
    return true;
}

bool CameraCalibrator::fileExists(const std::string& filePath) {
    namespace fs = std::filesystem;  // 🔧 修正：使用命名空间别名
    return fs::exists(filePath);
}

bool CameraCalibrator::loadParams(const std::string& fileName, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    namespace fs = std::filesystem;
    if (!fs::exists(fileName)) {
        std::cerr << "标定参数文件不存在: " << fileName << std::endl;
        return false;
    }

    cv::FileStorage fileStorage(fileName, cv::FileStorage::READ);
    if (!fileStorage.isOpened()) {
        std::cerr << "无法打开标定参数文件: " << fileName << std::endl;
        return false;
    }

    fileStorage["cameraMatrix"] >> cameraMatrix;
    fileStorage["distCoeffs"] >> distCoeffs;
    fileStorage.release();

    return true;
}

cv::Mat CameraCalibrator::undistortImage(const cv::Mat& src, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    cv::Mat undistorted;
    cv::undistort(src, undistorted, cameraMatrix, distCoeffs);
    return undistorted;
}

std::vector<cv::Point3f> CameraCalibrator::calcBoardCornerPositions() const {
    std::vector<cv::Point3f> objp;
    for(int i = 0; i < boardHeight_; ++i) {
        for(int j = 0; j < boardWidth_; ++j) {
            objp.push_back(cv::Point3f(j * squareSizeM_, i * squareSizeM_, 0));
        }
    }
    return objp;
}