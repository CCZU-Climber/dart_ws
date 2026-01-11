#ifndef HIK_CAMERA_H
#define HIK_CAMERA_H

#include <cstdio>
#include <iostream>
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include "Includes/MvCameraControl.h"

#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define GREEN_START "\033[32m"
#define COLOR_END "\033[0m"

namespace sensor::camera
{

    enum TRIGGERSOURCE
    {
        SOFTWARE,
        LINE0,
        LINE2,
    };
    enum GAMMAMODE
    {
        OFF,
        USER,
        sRGB,
    };

    class CAM_INFO
    {
    public:
        CAM_INFO &setCamID(int id)
        {
            _nCamID = id;
            return *this;
        }
        CAM_INFO &setWidth(int width)
        {
            _nWidth = width;
            return *this;
        }
        CAM_INFO &setHeight(int height)
        {
            _nHeight = height;
            return *this;
        }
        CAM_INFO &setOffsetX(int offsetX)
        {
            _nOffsetX = offsetX;
            return *this;
        }
        CAM_INFO &setOffsetY(int offsetY)
        {
            _nOffsetY = offsetY;
            return *this;
        }
        CAM_INFO &setExpTime(float expTime)
        {
            _nExpTime = expTime;
            return *this;
        }
        CAM_INFO &setGain(float gain)
        {
            _nGain = gain;
            return *this;
        }
        CAM_INFO &setTrigger(TRIGGERSOURCE trg)
        {
            _nTrigger = trg;
            return *this;
        }
        CAM_INFO &setHeartTimeOut(int Time)
        {
            _nHeartTimeOut = Time;
            return *this;
        }
        CAM_INFO &setGamma(GAMMAMODE Gamma)
        {
            _nGamma = Gamma;
            return *this;
        }
        friend class HikCam;

    private:
        int _nCamID = 0;
        int _nWidth = 1440;
        int _nHeight = 1080;
        int _nOffsetX = 0;
        int _nOffsetY = 0;
        int _nHeartTimeOut = 1000;
        float _nExpTime = 5000;
        float _nGain = 16;
        TRIGGERSOURCE _nTrigger = SOFTWARE;
        GAMMAMODE _nGamma = sRGB;
    };
    void __stdcall ImageCallBackEx(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser);
    class HikCam
    {
    public:
        HikCam(CAM_INFO Info);
        ~HikCam();
        auto Grab() -> cv::Mat;

    private:
        int _nRet = MV_OK;
        void *_handle = NULL;
        unsigned char *_pDstData = NULL;

        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        void SetAttribute(CAM_INFO Info);
    };

} // namespace sensor
#endif // HIK_CAMERA_H
