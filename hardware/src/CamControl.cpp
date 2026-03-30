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
                return false;
            }
        }
        uint64_t pixelFormat = 0;
        ret = IMV_GetEnumFeatureValue(m_cameraHandle, "PixelFormat", &pixelFormat);
        if (ret != IMV_OK)
        {
            CamControl::throwError("Failed to get PixelFormat.", ret);
            return false;
        }
        else
        {
            if (pixelFormat != gvspPixelMono12)
            {
                std::cout << "Change to Mono12 format." << std::endl;
                IMV_SetEnumFeatureValue(m_cameraHandle, "PixelFormat", gvspPixelMono12);
            }
        }
        // 设置触发模式为Line2
        ret = IMV_SetEnumFeatureSymbol(m_cameraHandle, "TriggerSource", "Line2");
        if (ret != IMV_OK)
        {
            CamControl::throwError("Failed to set trigger source.", ret);
            return false;
        }
        ret = IMV_SetEnumFeatureSymbol(m_cameraHandle, "TriggerSelector", "FrameStart");
        if (IMV_OK != ret)
        {
            std::cout << "Set triggerSelector value failed! ErrorCode[" << ret << "]" << std::endl;
            return false;
        }
        ret = IMV_SetEnumFeatureSymbol(m_cameraHandle, "TriggerMode", "Off");
        if (IMV_OK != ret)
        {
            std::cout << "Set triggerMode value failed! ErrorCode[" << ret << "]" << std::endl;
            return false;
        }
        ret = IMV_SetEnumFeatureSymbol(m_cameraHandle, "TimerTriggerActivation", "RisingEdge");
        if (IMV_OK != ret)
        {
            std::cout << "Set triggerActivation value failed! ErrorCode[" << ret << "]" << std::endl;
            return ret;
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
        m_currentFrameArray.resize(frameInfo.height, frameInfo.width);
        std::memcpy(m_currentFrameArray.data(), pFrame->pData, frameInfo.size);
        emit frameReceived(m_currentFrameArray); // pass by const reference   
    }
    void CamControl::setupCameraParameters(const IMV_Frame &firstFrame)
    {
        // Implementation for setting up camera parameters based on the first frame
        unsigned int width = firstFrame.frameInfo.width;
        unsigned int height = firstFrame.frameInfo.height;
    }
    void CamControl::throwError(const char *msg, const int errCode)
    {
        safeReleaseCamera();
        throw std::runtime_error(std::string(msg) + " Error code: " + std::to_string(errCode));
    }
    void CamControl::safeReleaseCamera()
    {
        if (m_cameraHandle)
        {
            if (IMV_IsGrabbing(m_cameraHandle))
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
    void CamControl::setCameraDoubleParameters(const char *pFeatureName, double value)
    {
        // Implementation for setting double parameters
        if (!m_cameraHandle)
        {
            throw std::runtime_error("Camera is not connected.");
        }
        double maxVal, minVal;
        try
        {
            std::tie(maxVal, minVal) = getCameraDoubleParametersMaxAndMin(pFeatureName);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
        if (value < minVal || value > maxVal)
        {
            throw std::runtime_error("Value out of range for feature: " + std::string(pFeatureName));
        }
        int ret = IMV_SetDoubleFeatureValue(m_cameraHandle, pFeatureName, value);
        if (ret != IMV_OK)
        {
            throw std::runtime_error("Failed to set feature value for: " + std::string(pFeatureName));
        }
    }
    void CamControl::setCameraIntParameters(const char *pFeatureName, int64_t value)
    {
        // Implementation for setting int parameters
        if (!m_cameraHandle)
        {
            throw std::runtime_error("Camera is not connected.");
        }
        int64_t maxVal, minVal;
        try
        {
            std::tie(maxVal, minVal) = getCameraIntParametersMaxAndMin(pFeatureName);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
        if (value < minVal || value > maxVal)
        {
            throw std::runtime_error("Value out of range for feature: " + std::string(pFeatureName));
        }
        int ret = IMV_SetIntFeatureValue(m_cameraHandle, pFeatureName, value);
        if (ret != IMV_OK)
        {
            throw std::runtime_error("Failed to set feature value for: " + std::string(pFeatureName));
        }
    }
    void CamControl::setCameraEnumParameters(const char *pFeatureName, uint64_t value)
    {
        // Implementation for setting enum parameters
        if (!m_cameraHandle)
        {
            throw std::runtime_error("Camera is not connected.");
        }
        unsigned int enumCount = 0;
        int ret = IMV_GetEnumFeatureEntryNum(m_cameraHandle, pFeatureName, &enumCount);
        if (ret != IMV_OK)
        {
            throw std::runtime_error("Failed to get enum entry number for: " + std::string(pFeatureName));
        }
        else if (value >= enumCount)
        {
            throw std::runtime_error("Enum value out of range for feature: " + std::string(pFeatureName));
        }
        ret = IMV_SetEnumFeatureValue(m_cameraHandle, pFeatureName, value);
        if (ret != IMV_OK)
        {
            throw std::runtime_error("Failed to set enum feature value for: " + std::string(pFeatureName));
        }
    }
    std::pair<double, double> CamControl::getCameraDoubleParametersMaxAndMin(const char *pFeatureName)
    {
        if (!m_cameraHandle)
        {
            throw std::runtime_error("Camera is not connected.");
        }
        double maxVal, minVal;
        int ret;
        ret = IMV_GetDoubleFeatureMax(m_cameraHandle, pFeatureName, &maxVal);
        if (ret != IMV_OK)
        {
            throw std::runtime_error("Failed to get max value for feature: " + std::string(pFeatureName));
        }
        ret = IMV_GetDoubleFeatureMin(m_cameraHandle, pFeatureName, &minVal);
        if (ret != IMV_OK)
        {
            throw std::runtime_error("Failed to get min value for feature: " + std::string(pFeatureName));
        }
        return std::make_pair(maxVal, minVal);
    }
    std::pair<int64_t, int64_t> CamControl::getCameraIntParametersMaxAndMin(const char *pFeatureName)
    {
        if (!m_cameraHandle)
        {
            throw std::runtime_error("Camera is not connected.");
        }
        int64_t maxVal, minVal;
        int ret;
        ret = IMV_GetIntFeatureMax(m_cameraHandle, pFeatureName, &maxVal);
        if (ret != IMV_OK)
        {
            throw std::runtime_error("Failed to get max value for feature: " + std::string(pFeatureName));
        }
        ret = IMV_GetIntFeatureMin(m_cameraHandle, pFeatureName, &minVal);
        if (ret != IMV_OK)
        {
            throw std::runtime_error("Failed to get min value for feature: " + std::string(pFeatureName));
        }
        return std::make_pair(maxVal, minVal);
    }
    void CamControl::setCameraTriggerMode(bool enableExternalTrigger)
    {
        if (!m_cameraHandle)
        {
            throw std::runtime_error("Camera is not connected.");
        }
        int ret = IMV_SetEnumFeatureSymbol(m_cameraHandle, "TriggerMode", enableExternalTrigger ? "On" : "Off");
        if (IMV_OK != ret)
        {
            std::cout << "Set triggerMode value failed! ErrorCode[" << ret << "]" << std::endl;
            return;
        }
    }
};