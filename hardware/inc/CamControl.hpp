#pragma once
#include <QtCore/QObject>
#include "IMVApi.h"
namespace Hardware
{
    // Frame stored as a Array: [height, width]
    class CamControl : public QObject
    {
        Q_OBJECT
    signals:
        // Pass FrameArray by value so the signal/slot system can own a copy safely.
        void frameReceived(IMV_Frame frame);

    private:
        void safeReleaseCamera();
        void throwError(const char *msg,const int errCode);
        IMV_DeviceList m_deviceList;
        IMV_HANDLE m_cameraHandle = nullptr;
        static void callbackFrameReceived(IMV_Frame *pFrame, void *pUser);
        void processFrame(IMV_Frame *pFrame);
        void setupCameraParameters(const IMV_Frame &firstFrame);
        IMV_Frame m_currentFrame;

    public:
        CamControl();
        ~CamControl();
        const IMV_DeviceList &scanCameras();
        bool connectCamera(int index);
        bool disconnectCamera(int index);
        void setCameraDoubleParameters(const char *pFeatureName, double value);
        void setCameraIntParameters(const char *pFeatureName, int64_t value);
        void setCameraEnumParameters(const char *pFeatureName, uint64_t value);
        std::pair<double,double> getCameraDoubleParametersMaxAndMin(const char *pFeatureName);
        std::pair<int64_t,int64_t> getCameraIntParametersMaxAndMin(const char *pFeatureName);
    };
};
