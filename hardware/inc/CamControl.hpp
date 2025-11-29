#pragma once
#ifndef CAMCONTROL_HPP
#define CAMCONTROL_HPP
#include <QtCore/QObject>
#include <unsupported/Eigen/CXX11/Tensor>
#include "CameraApi.h"

namespace Hardware
{
    // Frame stored as a 3D tensor: [height, width, channels]
    using CamFrameArray = Eigen::Tensor<BYTE, 3>;
    class CamControl : public QObject
    {
        Q_OBJECT
    signals:
        // Pass FrameArray by value so the signal/slot system can own a copy safely.
        void imageGrabbed(CamFrameArray frame);

    private:
        CameraHandle hCamera = -1;
        tSdkCameraDevInfo cameraInstance;
        CamFrameArray m_pFrameMat;
        static void GrabImageCallback(CameraHandle hCamera, BYTE *pFrameBuffer,
                                      tSdkFrameHead *pFrameHead, PVOID pContext);
        void GrabImageCallbackInstance(CameraHandle hCamera, BYTE *pFrameBuffer,
                                       tSdkFrameHead *pFrameHead);

    public:
        CamControl();
        ~CamControl();
        void CamStartGrab();
        void CamStopGrab();
        // Set camera instance info before starting grab (call from GUI thread before thread starts)
        void setSdkCameraDevInfo(const tSdkCameraDevInfo &info);
    };
};
#endif // CAMCONTROL_HPP