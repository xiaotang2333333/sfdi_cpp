#include "CamControl.hpp"
#include <iostream>
namespace Hardware
{
    void CamControl::GrabImageCallback(CameraHandle hCamera, BYTE *pBuffer, tSdkFrameHead *pFrameHead, PVOID pUser)
    {
        // 从 user 参数取回对象并转发
        CamControl *self = static_cast<CamControl *>(pUser);
        if (self)
        {
            self->GrabImageCallbackInstance(hCamera, pBuffer, pFrameHead);
        }
    }
    CamControl::CamControl(CameraHandle cameraHandle) : hCamera(cameraHandle)
    {
        if (hCamera == -1)
        {
            throw std::runtime_error("Invalid camera handle provided to CamControl.");
        }
        if (SDK_UNSUCCESS(CameraPlay(hCamera)))
        {
            throw std::runtime_error("Failed to initialize camera in CamControl.");
        }
        m_pFrameMat.resize(2688, 1520);
        m_pFrameMat.setZero(); // Example resolution, adjust as needed
        m_pFrameBuffer = reinterpret_cast<BYTE *>(m_pFrameMat.data());
        CameraSetCallbackFunction(hCamera, GrabImageCallback, (void *)this, nullptr);
    }

    CamControl::~CamControl()
    {
        if (hCamera != -1)
        {
            CameraUnInit(hCamera);
        }
    }
    void CamControl::GrabImageCallbackInstance(CameraHandle hCamera, BYTE *pFrameBuffer,
                                               tSdkFrameHead *pFrameHead)
    {

        if (SDK_UNSUCCESS(CameraImageProcess(hCamera, pFrameBuffer,
                                             m_pFrameBuffer, pFrameHead)))
        {
            std::cerr << "Image processing failed in GrabImageCallback." << std::endl;
            return;
        }
        FrameArray frameCopy = m_pFrameMat;
        emit imageGrabbed(frameCopy);
        m_pFrameMat.setZero();
    }
}