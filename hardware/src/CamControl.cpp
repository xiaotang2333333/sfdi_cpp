#include "CamControl.hpp"
#include <iostream>
#include <utility>
#include <thread>
#include <cstring>

namespace Hardware
{
    void FrameQueue::push(IMV_HANDLE cameraHandle, IMV_Frame &&frame)
    {
        QMutexLocker locker(&m_mutex);
        if (m_queue.size() >= MAX_QUEUE_SIZE)
        {
            if (cameraHandle && m_queue.front().pData)
            {
                IMV_ReleaseFrame(cameraHandle, &m_queue.front());
            }
            m_queue.pop_front();
        }
        m_queue.push_back(std::move(frame));
        m_cond.wakeOne();
    }

    bool FrameQueue::pop(IMV_Frame &frame, int timeoutMs)
    {
        QMutexLocker locker(&m_mutex);
        while (m_queue.empty())
        {
            if (!m_cond.wait(&m_mutex, timeoutMs))
            {
                return false;
            }
        }
        frame = std::move(m_queue.front());
        m_queue.pop_front();
        return true;
    }

    void FrameQueue::clear(IMV_HANDLE cameraHandle)
    {
        QMutexLocker locker(&m_mutex);
        if (cameraHandle)
        {
            for (auto &frame : m_queue)
            {
                if (frame.pData)
                {
                    IMV_ReleaseFrame(cameraHandle, &frame);
                }
            }
        }
        m_queue.clear();
    }

    int FrameQueue::size() const
    {
        QMutexLocker locker(&m_mutex);
        return static_cast<int>(m_queue.size());
    }

    CamControl::CamControl()
    {
    }

    CamControl::~CamControl()
    {
        m_running.storeRelease(0);
        if (m_dispatchThread && m_dispatchThread->joinable())
        {
            m_dispatchThread->join();
        }
        m_frameQueue.clear(m_cameraHandle);
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

    void CamControl::scanCamerasRequest()
    {
        QStringList cameraLabels;
        QString status;
        const IMV_DeviceList &deviceList = scanCameras();
        const int cameraCount = static_cast<int>(deviceList.nDevNum);

        if (cameraCount == 0)
        {
            status = "No cameras found.";
        }
        else
        {
            for (unsigned int i = 0; i < deviceList.nDevNum; ++i)
            {
                const IMV_DeviceInfo &devInfo = deviceList.pDevInfo[i];
                cameraLabels.push_back(QString("Camera %1: %2")
                                           .arg(i)
                                           .arg(QString::fromStdString(devInfo.modelName)));
            }
            status = QString("Found %1 camera(s).").arg(cameraCount);
        }

        emit scanCompleted(cameraLabels, status);
    }

    void CamControl::connectCameraRequest(int index)
    {
        bool success = false;
        QString status;

        try
        {
            success = connectCamera(index);
            status = success ? "Camera connected." : "Failed to connect camera.";
        }
        catch (const std::exception &e)
        {
            success = false;
            status = QString("Failed to connect camera: %1").arg(e.what());
        }

        emit connectionCompleted(success, status);

        if (success)
        {
            try
            {
                const auto exposureRange = getCameraDoubleParametersMaxAndMin("ExposureTime");
                double safeMin = exposureRange.minValue;
                double safeMax = exposureRange.maxValue;
                if (safeMin > safeMax)
                {
                    std::swap(safeMin, safeMax);
                }
                emit exposureRangeReady(true, safeMin, safeMax, "Exposure range loaded.");
            }
            catch (const std::exception &e)
            {
                emit exposureRangeReady(false, 0.0, 0.0, QString("Failed to query exposure range: %1").arg(e.what()));
            }
        }
    }

    void CamControl::disconnectCameraRequest(int index)
    {
        const bool success = disconnectCamera(index);
        emit disconnectionCompleted(success, success ? "Camera disconnected." : "Failed to disconnect camera.");
    }

    void CamControl::setCameraExposureTimeRequest(double exposureTime)
    {
        try
        {
            setCameraDoubleParameters("ExposureTime", exposureTime);
            emit parameterSetCompleted(true, QString("Exposure time set to %1 us.").arg(exposureTime));
        }
        catch (const std::exception &e)
        {
            emit parameterSetCompleted(false, QString("Error setting exposure time: %1").arg(e.what()));
        }
    }

    void CamControl::setCameraTriggerModeRequest(bool enableExternalTrigger)
    {
        try
        {
            setCameraTriggerMode(enableExternalTrigger);
            emit parameterSetCompleted(true, QString("Camera trigger mode set to %1.").arg(enableExternalTrigger ? "External Trigger" : "Free Run"));
        }
        catch (const std::exception &e)
        {
            emit parameterSetCompleted(false, QString("Error setting camera trigger mode: %1").arg(e.what()));
        }
    }

    bool CamControl::connectCamera(int index)
    {
        if (index < 0 || index >= static_cast<int>(m_deviceList.nDevNum))
        {
            std::cerr << "Invalid camera index: " << index << std::endl;
            return false;
        }
        int ret = IMV_CreateHandle(&m_cameraHandle, modeByIndex, reinterpret_cast<void *>(&index));
        if (ret != IMV_OK)
        {
            std::cerr << "Failed to create camera handle. Error code: " << ret << std::endl;
            return false;
        }
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
            return false;
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

        m_running.storeRelease(1);
        m_dispatchThread = std::make_unique<std::thread>(&CamControl::dispatchThread, this);

        std::cout << "Camera connected successfully." << std::endl;
        return true;
    }

    bool CamControl::disconnectCamera(int index)
    {
        m_running.storeRelease(0);
        if (m_dispatchThread && m_dispatchThread->joinable())
        {
            m_dispatchThread->join();
        }
        m_frameQueue.clear(m_cameraHandle);
        safeReleaseCamera();
        std::cout << "Camera disconnected successfully." << std::endl;
        return true;
    }

    void CamControl::callbackFrameReceived(IMV_Frame *pFrame, void *pUser)
    {
        CamControl *camControl = reinterpret_cast<CamControl *>(pUser);
        if (camControl)
        {
            camControl->processFrame(pFrame);
        }
    }

    void CamControl::processFrame(IMV_Frame *pFrame)
    {
        IMV_Frame clonedFrame{};

        const int ret = IMV_CloneFrame(m_cameraHandle, pFrame, &clonedFrame);
        if (ret != IMV_OK)
        {
            std::cerr << "Failed to clone frame. Error code: " << ret << std::endl;
            return;
        }

        m_frameQueue.push(m_cameraHandle, std::move(clonedFrame));
    }

    void CamControl::dispatchThread()
    {
        while (m_running.loadAcquire())
        {
            dequeueAndDispatchFrame(50);
        }
    }

    bool CamControl::dequeueAndDispatchFrame(int timeoutMs)
    {
        IMV_Frame frame{};
        if (!m_frameQueue.pop(frame, timeoutMs))
        {
            return false;
        }

        const IMV_FrameInfo &frameInfo = frame.frameInfo;
        if (!frame.pData || frameInfo.size == 0)
        {
            if (m_cameraHandle && frame.pData)
            {
                IMV_ReleaseFrame(m_cameraHandle, &frame);
            }
            return true;
        }

        auto ownedData = std::make_unique<uint8_t[]>(frameInfo.size);
        std::shared_ptr<uint8_t> copiedData(ownedData.release(), std::default_delete<uint8_t[]>());
        std::memcpy(copiedData.get(), frame.pData, frameInfo.size);

        emit frameReceivedForUI(copiedData,
                                static_cast<int>(frameInfo.width),
                                static_cast<int>(frameInfo.height),
                              static_cast<int>(frameInfo.size));
        emit frameReceivedForCompute(copiedData,
                                     static_cast<int>(frameInfo.width),
                                     static_cast<int>(frameInfo.height),
                                  static_cast<int>(frameInfo.size));

        if (m_cameraHandle)
        {
            IMV_ReleaseFrame(m_cameraHandle, &frame);
        }
        return true;
    }

    void CamControl::setupCameraParameters(const IMV_Frame &firstFrame)
    {
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
        if (!m_cameraHandle)
        {
            throw std::runtime_error("Camera is not connected.");
        }
        double maxVal = 0.0;
        double minVal = 0.0;
        try
        {
            const auto range = getCameraDoubleParametersMaxAndMin(pFeatureName);
            maxVal = range.maxValue;
            minVal = range.minValue;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
        if (minVal > maxVal)
        {
            std::swap(minVal, maxVal);
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
        if (!m_cameraHandle)
        {
            throw std::runtime_error("Camera is not connected.");
        }
        int64_t maxVal = 0;
        int64_t minVal = 0;
        try
        {
            const auto range = getCameraIntParametersMaxAndMin(pFeatureName);
            maxVal = range.maxValue;
            minVal = range.minValue;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
        if (minVal > maxVal)
        {
            std::swap(minVal, maxVal);
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

    FeatureRange<double> CamControl::getCameraDoubleParametersMaxAndMin(const char *pFeatureName)
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
        return FeatureRange<double>{maxVal, minVal};
    }

    FeatureRange<int64_t> CamControl::getCameraIntParametersMaxAndMin(const char *pFeatureName)
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
        return FeatureRange<int64_t>{maxVal, minVal};
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
            throw std::runtime_error("Failed to set trigger mode.");
        }
    }
};