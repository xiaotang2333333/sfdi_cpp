#pragma once
#include <QtCore/QObject>
#include <QtCore/QMutex>
#include <QtCore/QWaitCondition>
#include <QtCore/QAtomicInt>
#include <QString>
#include <QStringList>
#include "IMVApi.h"
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <thread>

namespace Hardware
{
    class FrameQueue
    {
    public:
        static const int MAX_QUEUE_SIZE = 18; 
        
        void push(IMV_HANDLE cameraHandle, IMV_Frame&& frame);
        bool pop(IMV_Frame& frame, int timeoutMs = 100);
        void clear(IMV_HANDLE cameraHandle);
        int size() const;
        
    private:
        mutable QMutex m_mutex;
        QWaitCondition m_cond;
        std::deque<IMV_Frame> m_queue;
    };

    class CamControl : public QObject
    {
        Q_OBJECT
    signals:
        void frameReceivedForUI(std::shared_ptr<uint8_t> data, int width, int height, int size);
        void frameReceivedForCompute(std::shared_ptr<uint8_t> data, int width, int height, int size);
        void scanCompleted(QStringList cameraLabels, QString status);
        void connectionCompleted(bool success, QString status);
        void disconnectionCompleted(bool success, QString status);
        void exposureRangeReady(bool success, double minValue, double maxValue, QString status);
        void parameterSetCompleted(bool success, QString status);

    public slots:
        void scanCamerasRequest();
        void connectCameraRequest(int index);
        void disconnectCameraRequest(int index);
        void setCameraExposureTimeRequest(double exposureTime);
        void setCameraTriggerModeRequest(bool enableExternalTrigger);
    
    private:
        void safeReleaseCamera();
        void throwError(const char *msg, const int errCode);
        IMV_DeviceList m_deviceList;
        IMV_HANDLE m_cameraHandle = nullptr;
        static void callbackFrameReceived(IMV_Frame *pFrame, void *pUser);
        void processFrame(IMV_Frame *pFrame);
        bool dequeueAndDispatchFrame(int timeoutMs);
        void dispatchThread();
        void setupCameraParameters(const IMV_Frame &firstFrame);
        
        FrameQueue m_frameQueue;
        QAtomicInt m_running{0};
        std::unique_ptr<std::thread> m_dispatchThread;
        
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

Q_DECLARE_METATYPE(std::shared_ptr<uint8_t>)
