#include <QtCore/QObject>
#include <Eigen/Dense>
#include "CameraApi.h"

namespace Hardware
{
    using FrameArray = Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    class CamControl : public QObject
    {
        Q_OBJECT
    signals:
        void imageGrabbed(const FrameArray &frame);

    private:
        CameraHandle hCamera = -1;
        FrameArray m_pFrameMat;
        BYTE *m_pFrameBuffer = nullptr;
        static void GrabImageCallback(CameraHandle hCamera, BYTE *pFrameBuffer,
                                      tSdkFrameHead *pFrameHead, PVOID pContext);
        void GrabImageCallbackInstance(CameraHandle hCamera, BYTE *pFrameBuffer,
                                       tSdkFrameHead *pFrameHead);
    public:
        CamControl(CameraHandle cameraHandle);
        ~CamControl();
    };
}