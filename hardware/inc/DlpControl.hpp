#pragma once
#include <QTimer>
#include <QtCore/QObject>
#include <memory>
namespace Hardware
{
    class Dlpc3500 : public QObject
    {
        Q_OBJECT
    public:
        explicit Dlpc3500(QObject *parent = nullptr);
        ~Dlpc3500();
        void stopProject();
        void updateFrequency(unsigned char index);
        void setRepeatMode(bool repeat){this->RepeatMode = repeat;};
        void setDmdExposeTime(unsigned int exposeTime){
            this->exposeTime = exposeTime < 8333 ? 8333 : exposeTime; // max frequency is 120Hz, so min expose time is 8333us
        };
    private:
        /* data */
        bool RepeatMode = true;
        unsigned int exposeTime = 8333; // default to 120Hz
        unsigned int m_numImgInFlash;
        std::unique_ptr<QTimer> m_usbPollTimer;
        void pollTimerTimeout();
        void SetDLPC350InPatternMode();
    signals:
        void dlpcStatus(bool isConnected);
        void flashIndexUpdated(unsigned int imgCount);
    };
};
