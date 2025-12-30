#pragma once
#ifndef DLPCONTROL_HPP
#define DLPCONTROL_HPP
#include <QtCore/QObject>
#include <QTimer>
#include <memory>
namespace Hardware
{
class Dlpc3500 : public QObject
{
    Q_OBJECT
public:
    explicit Dlpc3500(QObject* parent = nullptr);
    ~Dlpc3500();
    void stopProject();
    void updateFrequency(unsigned int index);
private:
    /* data */
    unsigned int m_numImgInFlash;
    std::unique_ptr<QTimer> m_usbPollTimer;
    void pollTimerTimeout();
    void SetDLPC350InPatternMode();
signals:
    void dlpcStatus(bool isConnected);
};
};
#endif // DLPCONTROL_HPP