cmake_minimum_required(VERSION 3.10)
project(HikCamGreenDetector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV - 移除ximgproc依赖
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/Includes
)

# 设置海康相机SDK库路径
set(HIKVISION_SDK_PATH "/home/dart/Desktop/HikCam/lib")

# 链接目录
link_directories(
    ${HIKVISION_SDK_PATH}/lib
)

# 源文件
set(SOURCES
    HikCam.cpp
)

set(HEADERS
    HikCam.h
    Includes/MvCameraControl.h
)

# 创建HikCam库
add_library(HikCamLib ${SOURCES} ${HEADERS})

# 链接HikCam库
target_link_libraries(HikCamLib 
    ${OpenCV_LIBS}
    # 根据你的设备架构选择amd64或者arm64
    /home/dart/Desktop/HikCam/lib/amd64/libMvCameraControl.so
)

# 绿色识别可执行文件
add_executable(hikcam_green_detector main.cpp)

# 链接库
target_link_libraries(hikcam_green_detector 
    HikCamLib
    ${OpenCV_LIBS}
)

# 设置可执行文件属性
set_target_properties(hikcam_green_detector PROPERTIES
    OUTPUT_NAME "hikcam_green_detector"
)