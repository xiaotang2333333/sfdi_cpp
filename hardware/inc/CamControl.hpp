#pragma once
#include <QtCore/QObject>
#include "IMVApi.h"
#include <Eigen/Dense>
namespace Hardware
{
    // Frame stored as a Array: [height, width]
    class CamControl : public QObject
    {
        Q_OBJECT
    signals:
        // Pass FrameArray by const reference to avoid copy on emit; receiver should copy immediately.
        void frameReceived(const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &frame);
    
    private:
        void safeReleaseCamera();
        void throwError(const char *msg, const int errCode);
        IMV_DeviceList m_deviceList;
        IMV_HANDLE m_cameraHandle = nullptr;
        static void callbackFrameReceived(IMV_Frame *pFrame, void *pUser);
        void processFrame(IMV_Frame *pFrame);
        void setupCameraParameters(const IMV_Frame &firstFrame);
        Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> m_currentFrameArray; // 当前帧数据的Eigen表示
    public:
        CamControl();
        ~CamControl();
        const IMV_DeviceList &scanCameras();
        IMV_HANDLE getCameraHandle() const { return m_cameraHandle; }
        bool connectCamera(int index);
        bool disconnectCamera(int index);
        void setCameraTriggerMode(bool enableExternalTrigger);
        void setCameraDoubleParameters(const char *pFeatureName, double value);
        void setCameraIntParameters(const char *pFeatureName, int64_t value);
        void setCameraEnumParameters(const char *pFeatureName, uint64_t value);
        std::pair<double, double> getCameraDoubleParametersMaxAndMin(const char *pFeatureName);
        std::pair<int64_t, int64_t> getCameraIntParametersMaxAndMin(const char *pFeatureName);
    };
};
