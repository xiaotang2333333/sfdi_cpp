#include "CamControl.hpp"
#include <iostream>
#include <utility>
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
    CamControl::CamControl()
    {
        // 初始化成员变量
        m_pFrameMat.resize(1520,2688,3);
        m_pFrameMat.setZero(); // Example resolution, adjust as needed
    }

    void CamControl::setSdkCameraDevInfo(const tSdkCameraDevInfo &info)
    {
        cameraInstance = info;
    }
    void CamControl::CamStartGrab()
    {
        if (SDK_UNSUCCESS(CameraInit(&cameraInstance, -1, -1, &hCamera)))
        {
            throw std::runtime_error("Invalid camera instance provided to CamControl.");
        }
        if (hCamera == -1)
        {
            throw std::runtime_error("Invalid camera handle provided to CamControl.");
        }
        if (SDK_UNSUCCESS(CameraPlay(hCamera)))
        {
            throw std::runtime_error("Failed to initialize camera in CamControl.");
        }
        if (SDK_UNSUCCESS(CameraSetCallbackFunction(hCamera, GrabImageCallback, (void *)this, nullptr)))
        {
            throw std::runtime_error("Failed to set Fallback");
        }
    }
    void CamControl::CamStopGrab()
    {
        if (hCamera != -1)
        {
            CameraUnInit(hCamera);
            hCamera = -1;
        }
    }
    CamControl::~CamControl()
    {
        CamStopGrab();
    }
    void CamControl::GrabImageCallbackInstance(CameraHandle hCamera, BYTE *pFrameBuffer,
                                               tSdkFrameHead *pFrameHead)
    {
        if (SDK_UNSUCCESS(CameraImageProcess(hCamera, pFrameBuffer,
                                             m_pFrameMat.data(), pFrameHead)))
        {
            std::cerr << "Image processing failed in GrabImageCallback." << std::endl;
            return;
        }
        emit imageGrabbed(m_pFrameMat.eval());
        m_pFrameMat.setZero();
    }
}