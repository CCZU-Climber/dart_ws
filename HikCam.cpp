/*************************************************************************
 * Copyright (c) 2024, Misaka21
 * All rights reserved.
 *
 *
 * File name    :   hik_camera.cpp
 * Brief        :   海康相机的相关函数
 * Revision     :   V3.0
 * Author       :   Misaka21
 * Date         :   2024.04.10
 * Update       :   2023.10.16  V1.0    完成基本代码的编写
 *                  2023.10.18  V1.1    完成单线程的基本代码编写
 *                  2023.10.21  V2.0    完成相机双线程读写缓存
 *                  2023.11.10  V2.1    完成BayerRG转Mat的操作
 *                  2024.01.01  V2.2    完成相机错误代码的优化
 *                  2024.04.20  V3.0    完成产线上工业相机的硬触发和相关参数的设置
 *                  2024.04.28  V3.1    加入Gamma选择和超时时间
 *                  2024.04.30  V4.0    RM发布者版本的海康取流
 *                  TODO：加入垂直翻转，水平翻转，相机参数输出，简化设置相机参数流程
*************************************************************************/
#include "HikCam.h"
#include <opencv2/imgcodecs.hpp>

namespace sensor::camera{
        
    bool HikCam::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("The Pointer of pstMVDevInfo is NULL!\n");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
            printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
            printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
        }
        else
        {
            printf("Not support.\n");
        }

        return true;
    }



    HikCam::HikCam(CAM_INFO Info) : _info(Info) {  // 🔧 修改：使用初始化列表赋值 _info

        // ch:初始化SDK | en:Initialize SDK
        // _nRet = MV_CC_Initialize();
        // if (MV_OK != _nRet)
        // {
        //    //printf("Initialize SDK fail! nRet [0x%x]\n", _nRet);
        //    std::cout<<RED_START<<"[ERROR]: Initialize SDK fail! nRet [0x%x]\n"<<COLOR_END<<_nRet<<std::endl;
        //    //break;
        // }

        // ch:枚举设备 | Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        _nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != _nRet)
        {
            std::cout<<RED_START<<"[ERROR]Enum Devices fail! nRet [0x%x]\n"<<COLOR_END<<_nRet<<std::endl;
            //printf("Enum Devices fail! nRet [0x%x]\n", _nRet);
            //break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    //break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            std::cout<<RED_START<<"[ERROR]: Find No Devices!\n"<<COLOR_END<<std::endl;
            //printf("Find No Devices!\n");
            //break;
        }

        printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum - 1);
        unsigned int nIndex = Info._nCamID;
        printf(" %d\n", Info._nCamID);


        // ch:选择设备并创建句柄 | Select device and create handle
        _nRet = MV_CC_CreateHandle(&_handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != _nRet)
        {
            printf("Create _handle fail! nRet [0x%x]\n", _nRet);
            //break;
        }

        // ch:打开设备 | Open device
        _nRet = MV_CC_OpenDevice(_handle);
        if (MV_OK != _nRet)
        {
            printf("%sOpen Device fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            //printf("Open Device fail! nRet [0x%x]\n", _nRet);
            //break;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(_handle);
            if (nPacketSize > 0)
            {
                _nRet = MV_CC_SetIntValue(_handle, "GevSCPSPacketSize", nPacketSize);
                if (_nRet != MV_OK)
                {
                    printf("%s[Warning]: Set Packet Size fail nRet [0x%x]!%s\n", YELLOW_START, _nRet, COLOR_END);
                    //printf("Warning: Set Packet Size fail nRet [0x%x]!", _nRet);
                }
            }
            else
            {
                printf("%s[Warning]: Get Packet Size fail nRet [0x%x]!%s\n", YELLOW_START, nPacketSize, COLOR_END);
                //printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        // ch:设置触发模式 | en:Set trigger mode (removed here, handled in SetAttribute)
        // _nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        // if (MV_OK != _nRet)
        // {
        //     printf("%s[ERROR]: Set Trigger Mode fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
        //     //printf("Set Trigger Mode fail! nRet [0x%x]\n", _nRet);
        //     //break;
        // }
        SetAttribute();  // 🔧 修改：调用无参数版本

        // ch:开始取流 | en:Start grab image
        _nRet = MV_CC_StartGrabbing(_handle);
        if (MV_OK != _nRet)
        {
            printf("%s[ERROR]: Start Grabbing fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            //printf("Start Grabbing fail! nRet [0x%x]\n", _nRet);
            //break;
        }
    }

    auto HikCam::Grab() -> cv::Mat
    {
        cv::Mat srcImage;
        MV_FRAME_OUT stImageInfo = {};
        const int maxRetries = 5; // 最大重试次数
        int numRetries = 0;

        while (numRetries < maxRetries) {
            if (_info._nTrigger == SOFTWARE) {  // 🔧 修改：使用 _info
                _nRet = MV_CC_SetCommandValue(_handle, "TriggerSoftware");
                if (_nRet != MV_OK) {
                    printf("Trigger Software fail! nRet [0x%x]\n", _nRet);
                    numRetries++;
                    continue;
                }
            }
            _nRet = MV_CC_GetImageBuffer(_handle, &stImageInfo, 1000);

            if (_nRet == MV_OK)
            {
                unsigned char *_pDstData = (unsigned char *)stImageInfo.pBufAddr; // 直接使用提供的缓冲区

                // 处理常见的像素格式
                unsigned int pixelType = stImageInfo.stFrameInfo.enPixelType;
                // 仅在第一次获取到像素格式时打印一次，避免每帧都输出
                if (_lastPixelType == -1) {
                    printf("[Debug] PixelType: 0x%x\n", pixelType);
                    _lastPixelType = (int)pixelType;
                }
                int width = stImageInfo.stFrameInfo.nWidth;
                int height = stImageInfo.stFrameInfo.nHeight;
                unsigned int frameLen = stImageInfo.stFrameInfo.nFrameLen;

                if (PixelType_Gvsp_Mono8 == pixelType)
                { // Mono8类型
                    srcImage = cv::Mat(height, width, CV_8UC1, _pDstData).clone();
                }
                else if (PixelType_Gvsp_BayerRG8 == pixelType || PixelType_Gvsp_BayerGR8 == pixelType
                      || PixelType_Gvsp_BayerGB8 == pixelType || PixelType_Gvsp_BayerBG8 == pixelType)
                { // Bayer系列
                    srcImage = cv::Mat(height, width, CV_8UC1, _pDstData).clone();
                    if (PixelType_Gvsp_BayerRG8 == pixelType) cv::cvtColor(srcImage, srcImage, cv::COLOR_BayerRG2BGR);
                    else if (PixelType_Gvsp_BayerGR8 == pixelType) cv::cvtColor(srcImage, srcImage, cv::COLOR_BayerGR2BGR);
                    else if (PixelType_Gvsp_BayerGB8 == pixelType) cv::cvtColor(srcImage, srcImage, cv::COLOR_BayerGB2BGR);
                    else if (PixelType_Gvsp_BayerBG8 == pixelType) cv::cvtColor(srcImage, srcImage, cv::COLOR_BayerBG2BGR);
                }
                else if (PixelType_Gvsp_RGB8_Packed == pixelType || PixelType_Gvsp_BGR8_Packed == pixelType
                      || PixelType_Gvsp_HB_RGB8_Packed == pixelType || PixelType_Gvsp_HB_BGR8_Packed == pixelType)
                { // RGB/BGR packed
                    srcImage = cv::Mat(height, width, CV_8UC3, _pDstData).clone();
                    if (PixelType_Gvsp_RGB8_Packed == pixelType || PixelType_Gvsp_HB_RGB8_Packed == pixelType)
                    {
                        // 转为OpenCV默认的BGR排列
                        cv::cvtColor(srcImage, srcImage, cv::COLOR_RGB2BGR);
                    }
                }
                else if (PixelType_Gvsp_YUV420SP_NV12 == pixelType)
                {
                    // NV12: height * 3/2 rows, single channel
                    cv::Mat yuv(height * 3 / 2, width, CV_8UC1, _pDstData);
                    cv::cvtColor(yuv, srcImage, cv::COLOR_YUV2BGR_NV12);
                }
                else if (PixelType_Gvsp_YUV420SP_NV21 == pixelType)
                {
                    cv::Mat yuv(height * 3 / 2, width, CV_8UC1, _pDstData);
                    cv::cvtColor(yuv, srcImage, cv::COLOR_YUV2BGR_NV21);
                }
                else if (PixelType_Gvsp_YUV422_Packed == pixelType || PixelType_Gvsp_YUV422_YUYV_Packed == pixelType
                         || PixelType_Gvsp_HB_YUV422_Packed == pixelType || PixelType_Gvsp_HB_YUV422_YUYV_Packed == pixelType)
                {
                    cv::Mat yuv(height, width, CV_8UC2, _pDstData);
                    // 假设为YUYV打包格式（YUY2）
                    cv::cvtColor(yuv, srcImage, cv::COLOR_YUV2BGR_YUY2);
                }
                else if (PixelType_Gvsp_Jpeg == pixelType)
                {
                    // 压缩的JPEG数据，使用imdecode解码
                    std::vector<uchar> buf(_pDstData, _pDstData + frameLen);
                    srcImage = cv::imdecode(buf, cv::IMREAD_COLOR);
                }
                else
                {
                    // 未知/不支持的像素格式，打印信息但不立即exit，以便调试
                    printf("%s[ERROR]: Unsupported pixel format 0x%x%s\n", RED_START, pixelType, COLOR_END);
                    // 你可以在此处添加更多格式的处理，或者将此像素格式映射到合适的OpenCV转换
                }
                _nRet = MV_CC_FreeImageBuffer(_handle, &stImageInfo);
                if (_nRet != MV_OK)
                {
                    printf("Free Image Buffer fail! nRet [0x%x]\n", _nRet);
                }
                // 成功获取图像后退出循环
                break;
            }
            else
            {
                printf("%s[ERROR]: Get Image fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
                numRetries++; // 增加重试计数
                
            }
        }

        return srcImage;
    }

    void HikCam::SetAttribute() {
        if (_info._nTrigger == SOFTWARE) {
            _nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", MV_TRIGGER_MODE_ON);
            if (MV_OK != _nRet) {
                printf("%s[ERROR]: Set Trigger Mode ON fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            }
            _nRet = MV_CC_SetEnumValueByString(_handle, "TriggerSource", "Software");
            if (MV_OK != _nRet) {
                printf("%s[WARNING]: Set Trigger Software fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            }
        } else if (_info._nTrigger == LINE0) {
            _nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", MV_TRIGGER_MODE_ON);
            if (MV_OK != _nRet) {
                printf("%s[ERROR]: Set Trigger Mode ON fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            }
            _nRet = MV_CC_SetEnumValueByString(_handle, "TriggerSource", "Line0");
            if (MV_OK != _nRet) {
                printf("%s[WARNING]: Set Trigger Line0 fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            }
        } else if (_info._nTrigger == LINE2) {
            _nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", MV_TRIGGER_MODE_ON);
            if (MV_OK != _nRet) {
                printf("%s[ERROR]: Set Trigger Mode ON fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            }
            _nRet = MV_CC_SetEnumValueByString(_handle, "TriggerSource", "Line2");
            if (MV_OK != _nRet) {
                printf("%s[WARNING]: Set Trigger Line2 fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            }
        } else {
            _nRet = MV_CC_SetEnumValue(_handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
            if (MV_OK != _nRet) {
                printf("%s[ERROR]: Set Trigger Mode OFF fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            }
        }

        // 设置曝光时间
        _nRet = MV_CC_SetFloatValue(_handle, "ExposureTime", _info._nExpTime);  // 🔧 修改：使用 _info
        if (MV_OK != _nRet)
        {
            printf("%s[WARNING]: Set ExposureTime fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            //printf("Set ExposureTime fail! nRet [0x%x]\n", _nRet);
            //break;
        }

        // 针对 50Hz 工频环境的默认防闪烁设置：关闭自动曝光并固定曝光为 20ms（20000 us），同时启用固定帧率 25 fps
        // 这些设置会覆盖 Info 中的曝光值，便于减少由环境光源导致的闪烁
        _nRet = MV_CC_SetEnumValueByString(_handle, "ExposureAuto", "Off");
        if (MV_OK != _nRet)
        {
            printf("%s[WARNING]: Disable Auto Exposure fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
        }
        // 强制曝光 20ms（20000 微秒）
        _nRet = MV_CC_SetFloatValue(_handle, "ExposureTime", 20000.0f);
        if (MV_OK != _nRet)
        {
            printf("%s[WARNING]: Set ExposureTime(20ms) fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
        }
        // 启用并设置固定采集帧率为 25 fps（适配 50Hz 照明）
        _nRet = MV_CC_SetBoolValue(_handle, "AcquisitionFrameRateEnable", true);
        if (MV_OK == _nRet)
        {
            _nRet = MV_CC_SetFloatValue(_handle, "AcquisitionFrameRate", 25.0f);
            if (MV_OK != _nRet)
            {
                printf("%s[WARNING]: Set AcquisitionFrameRate fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            }
        }
        // 设置增益
        _nRet = MV_CC_SetFloatValue(_handle, "Gain", _info._nGain);  // 🔧 修改：使用 _info
        if (MV_OK != _nRet)
        {
            printf("%s[WARNING]: Set Gain fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            //printf("Set Gain fail! nRet [0x%x]\n", _nRet);
            //break;
        }
        // 设置宽度
        _nRet = MV_CC_SetIntValue(_handle, "Width", _info._nWidth);  // 🔧 修改：使用 _info
        if (MV_OK != _nRet)
        {
            printf("%s[WARNING]: Set Width fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            //printf("Set Width fail! nRet [0x%x]\n", _nRet);
            //break;
        }
        // 设置高度
        _nRet = MV_CC_SetIntValue(_handle, "Height", _info._nHeight);  // 🔧 修改：使用 _info
        if (MV_OK != _nRet)
        {
            printf("%s[WARNING]: Set Height fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            // printf("Set Height fail! nRet [0x%x]\n", _nRet);
            //break;
        }
        // 设置偏移X
        _nRet = MV_CC_SetIntValue(_handle, "OffsetX", _info._nOffsetX);  // 🔧 修改：使用 _info
        if (MV_OK != _nRet)
        {
            printf("%s[WARNING]: Set OffsetX fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            //printf("Set OffsetX fail! nRet [0x%x]\n", _nRet);
            //break;
        }
        // 设置偏移Y
        _nRet = MV_CC_SetIntValue(_handle, "OffsetY", _info._nOffsetY);  // 🔧 修改：使用 _info
        if (MV_OK != _nRet)
        {
            printf("%s[ERROR]: Set OffsetY fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
            //printf("Set OffsetY fail! nRet [0x%x]\n", _nRet);
            //break;
        }
    //    //设置心跳时间
    //    _nRet = MV_CC_SetIntValue(_handle, "GevHeartbeatTimeout", Info._nHeartTimeOut);
    //    if (MV_OK != _nRet)
    //    {
    //        printf("%s[ERROR]: Set HeartbeatTimeout fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
    //        //printf("Set OffsetY fail! nRet [0x%x]\n", _nRet);
    //        //break;
    //    }
        //设置Gamma使能
        _nRet = MV_CC_SetBoolValue(_handle, "GammaEnable", _info._nGamma);  // 🔧 修改：使用 _info
        if(_info._nGamma)
            _nRet = MV_CC_SetEnumValue(_handle, "GammaSelector", _info._nGamma);
        if (MV_OK != _nRet)
        {
            printf("%s[ERROR]: Set GammaMode fail! nRet [0x%x]%s\n", YELLOW_START, _nRet, COLOR_END);
        }

        //输出当前设置
        printf("%s", GREEN_START);
        printf("Current Setting:\n");
        printf("ExposureTime: %f\n", _info._nExpTime);  // 🔧 修改：使用 _info
        printf("Gain: %f\n", _info._nGain);  // 🔧 修改：使用 _info
        printf("Width: %d\n", _info._nWidth);  // 🔧 修改：使用 _info
        printf("Height: %d\n", _info._nHeight);  // 🔧 修改：使用 _info
        printf("HeartbeatTimeout: %d\n", _info._nHeartTimeOut);  // 🔧 修改：使用 _info
        printf("OffsetX: %d\n", _info._nOffsetX);  // 🔧 修改：使用 _info
        printf("OffsetY: %d\n", _info._nOffsetY);  // 🔧 修改：使用 _info
        auto getTriggerSource = [](TRIGGERSOURCE trigger) -> const char* {
            switch (trigger) {
            case CONTINUOUS: return "CONTINUOUS";
            case SOFTWARE: return "SOFTWARE";
            case LINE0:    return "LINE0";
            case LINE2:    return "LINE2";
            default:       return "ERROR";
            }
        };
        printf("TriggerSource: %s\n", getTriggerSource(_info._nTrigger));  // 🔧 修改：使用 _info
        auto getGammaMode = [](GAMMAMODE nGamma) -> const char* {
            switch (nGamma) {
                case OFF:       return "OFF";
                case USER:      return "User";
                case sRGB:      return "sRGB";
                default:        return "ERROR";
            }
        };
        printf("GammaMode: %s\n", getGammaMode(_info._nGamma));  // 🔧 修改：使用 _info
        printf("**************************%s", COLOR_END);
    }
    HikCam::~HikCam() {
        // ch:停止取流 | en:Stop grab image
        _nRet = MV_CC_StopGrabbing(_handle);
        if (MV_OK != _nRet)
        {
            printf("%s[ERROR]: Stop Grabbing fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            //printf("Stop Grabbing fail! nRet [0x%x]\n", _nRet);
            //break;
        }

        // ch:注销抓图回调 | en:Unregister image callback
        _nRet = MV_CC_RegisterImageCallBackEx(_handle, NULL, NULL);
        if (MV_OK != _nRet)
        {
            printf("%s[ERROR]: Unregister Image CallBack fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            //printf("Unregister Image CallBack fail! nRet [0x%x]\n", _nRet);
            //break;
        }

        // ch:关闭设备 | en:Close device
        _nRet = MV_CC_CloseDevice(_handle);
        if (MV_OK != _nRet)
        {
            printf("%s[ERROR]: Close Device fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            //printf("Close Device fail! nRet [0x%x]\n", _nRet);
            //break;
        }

        // ch:销毁句柄 | en:Destroy handle
        _nRet = MV_CC_DestroyHandle(_handle);
        if (MV_OK != _nRet)
        {
            printf("%s[ERROR]: Destroy _handle fail! nRet [0x%x]%s\n", RED_START, _nRet, COLOR_END);
            //printf("Destroy _handle fail! nRet [0x%x]\n", _nRet);
            //break;
        }
        _handle = NULL;


        if (_handle != NULL)
        {
            MV_CC_DestroyHandle(_handle);
            _handle = NULL;
        }


        // ch:反初始化SDK | en:Finalize SDK
        //MV_CC_Finalize();
    }

}