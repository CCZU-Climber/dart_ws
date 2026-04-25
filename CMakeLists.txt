cmake_minimum_required(VERSION 3.10)
project(HikCamGreenDetector)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "找到 OpenCV 版本: ${OpenCV_VERSION}")

# ============ 海康威视SDK配置 ============
# 设置SDK根路径
set(MVSDK_ROOT "/opt/MVS")

# 设置库文件路径（根据你的实际架构）
set(MVSDK_LIB_DIR "${MVSDK_ROOT}/lib/aarch64")

# 设置头文件路径
set(MVSDK_INCLUDE_DIR "${MVSDK_ROOT}/include")

# 查找海康SDK库文件
find_library(MVSDK_LIB 
    NAMES MvCameraControl
    PATHS 
        ${MVSDK_LIB_DIR}
        /usr/lib
        /usr/local/lib
        /opt/MVS/lib
        /opt/MVS/lib/64
        ${CMAKE_CURRENT_SOURCE_DIR}/lib
    REQUIRED
)

# 查找海康SDK头文件
find_path(HIK_INCLUDE_DIR
    NAMES MvCameraControl.h
    PATHS 
        ${MVSDK_INCLUDE_DIR}
        /usr/include
        /usr/local/include
        /opt/MVS/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    REQUIRED
)

message(STATUS "海康SDK库路径: ${MVSDK_LIB}")
message(STATUS "海康SDK头文件路径: ${HIK_INCLUDE_DIR}")
# =========================================

# 包含目录
include_directories(
    ${HIK_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
)

# 源文件列表
set(SOURCE_FILES
    main.cpp
    HikCam.cpp
    SerialPort.cpp
    MotorController.cpp
    VisionDetector.cpp
    AlignmentController.cpp
    UserInterface.cpp
    GridDrawer.cpp
    CameraCalibrator.cpp
    DetectionResult.cpp  # 添加DetectionResult实现文件
    DistanceEstimator.cpp  # 添加DistanceEstimator实现文件
)

# 创建可执行文件
add_executable(hikcam_green_detector ${SOURCE_FILES})

# 链接库
target_link_libraries(hikcam_green_detector
    ${OpenCV_LIBS}
    ${MVSDK_LIB}
    pthread
    rt
)

# 添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(hikcam_green_detector PRIVATE -Wall -Wextra)
endif()

# 已去掉 bin 目录设置，可执行文件将直接生成在 build/ 目录下