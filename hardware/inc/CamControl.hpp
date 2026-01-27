#pragma once
#include <QtCore/QObject>
#include <unsupported/Eigen/CXX11/Tensor>
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
    };
};
