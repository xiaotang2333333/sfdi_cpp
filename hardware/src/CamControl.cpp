#include "CamControl.hpp"
#include <iostream>
#include <utility>
namespace Hardware
{
    CamControl::CamControl()
    {
        // 初始化成员变量
    }
    CamControl::~CamControl()
    {
        // 关闭相机并释放资源
        if (m_cameraHandle)
        {
            IMV_Close(m_cameraHandle);
            IMV_DestroyHandle(m_cameraHandle);
            m_cameraHandle = nullptr;
        }
    }
    const IMV_DeviceList &CamControl::scanCameras()
    {
        IMV_EnumDevices(&m_deviceList, interfaceTypeUsb3);
        return m_deviceList;
    }
    bool CamControl::connectCamera(int index)
    {
        if (index < 0 || index >= static_cast<int>(m_deviceList.nDevNum))
        {
            std::cerr << "Invalid camera index: " << index << std::endl;
            return false;
        }
        // 创建相机句柄
        int ret = IMV_CreateHandle(&m_cameraHandle, modeByIndex, reinterpret_cast<void *>(&index));
        if (ret != IMV_OK)
        {
            std::cerr << "Failed to create camera handle. Error code: " << ret << std::endl;
            return false;
        }
        // 打开相机
        if (IMV_IsOpen(m_cameraHandle))
        {
            std::cout << "Camera is already open." << std::endl;
        }
        else
        {
            ret = IMV_Open(m_cameraHandle);
            if (ret != IMV_OK)
            {
                CamControl::throwError("Failed to open camera.", ret);
            }
        }
        uint64_t pixelFormat = 0;
        if(IMV_GetEnumFeatureValue(m_cameraHandle, "PixelFormat", &pixelFormat) != IMV_OK)
        {
            CamControl::throwError("Failed to get PixelFormat.", ret);
            return false;
        }
        else
        {
            if(pixelFormat != gvspPixelMono12)
            {
                std::cout<< "Change to Mono12 format." <<std::endl;
                IMV_SetEnumFeatureValue(m_cameraHandle, "PixelFormat", gvspPixelMono12);
            }
        }
        ret = IMV_AttachGrabbing(m_cameraHandle, callbackFrameReceived, this);
        if (ret != IMV_OK)
        {
            CamControl::throwError("Failed to attach grabbing.", ret);
            return false;
        }
        ret = IMV_StartGrabbing(m_cameraHandle);
        if (ret != IMV_OK)
        {
            CamControl::throwError("Failed to start grabbing.", ret);
            return false;
        }
        std::cout << "Camera connected successfully." << std::endl;
        return true;
    }
    bool CamControl::disconnectCamera(int index)
    {
        safeReleaseCamera();
        std::cout << "Camera disconnected successfully." << std::endl;
        return true;
    }
    void CamControl::callbackFrameReceived(IMV_Frame *pFrame, void *pUser)
    {
        // 处理接收到的帧数据
        CamControl *camControl = reinterpret_cast<CamControl *>(pUser);
        if (camControl)
        {
            camControl->processFrame(pFrame);
        }
    }
    void CamControl::processFrame(IMV_Frame *pFrame)
    {
        IMV_FrameInfo &frameInfo = pFrame->frameInfo;
        IMV_CloneFrame(m_cameraHandle, pFrame, &m_currentFrame);
        frameReceived(m_currentFrame);
    }
    void CamControl::setupCameraParameters(const IMV_Frame &firstFrame)
    {
        // Implementation for setting up camera parameters based on the first frame
        unsigned int width = firstFrame.frameInfo.width;
        unsigned int height = firstFrame.frameInfo.height;
    }
    void CamControl::throwError(const char* msg, const int errCode)
    {
        safeReleaseCamera();
        throw std::runtime_error(std::string(msg) + " Error code: " + std::to_string(errCode));
    }
    void CamControl::safeReleaseCamera()
    {
        if (m_cameraHandle)
        {
            if(IMV_IsGrabbing(m_cameraHandle))
            {
                IMV_StopGrabbing(m_cameraHandle);
            }
            if (IMV_IsOpen(m_cameraHandle))
            {
                IMV_Close(m_cameraHandle);
            }
            IMV_DestroyHandle(m_cameraHandle);
            m_cameraHandle = nullptr;
        }
        else
        {
            std::cout << "Camera handle is already null." << std::endl;
        }
    }
};