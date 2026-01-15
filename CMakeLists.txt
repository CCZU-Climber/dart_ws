cmake_minimum_required(VERSION 3.10)
project(HikCamGreenDetector)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 查找海康SDK库
set(HIK_SDK_PATHS
    /usr/lib
    /usr/local/lib
    /opt/MVS/lib
    /opt/MVS/lib/64
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

# 查找海康SDK库文件
find_library(MVSDK_LIB 
    NAMES MvCameraControl
    PATHS ${HIK_SDK_PATHS}
    REQUIRED
)

# 查找海康SDK头文件
find_path(HIK_INCLUDE_DIR
    NAMES MvCameraControl.h
    PATHS /usr/include /usr/local/include /opt/MVS/include ${CMAKE_CURRENT_SOURCE_DIR}/include
    REQUIRED
)

# 包含目录
include_directories(${HIK_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# 源文件列表（包括所有新建的模块）
set(SOURCE_FILES
    main.cpp
    HikCam.cpp
    SerialPort.cpp
    MotorController.cpp
    VisionDetector.cpp
    AlignmentController.cpp
    UserInterface.cpp
    GridDrawer.cpp
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