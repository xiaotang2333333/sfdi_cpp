#include "DlpControl.hpp"
#include "dlpc350_usb.h"
#include "dlpc350_api.h"
#include <thread>
#include <chrono>
using namespace Hardware;
Dlpc3500::Dlpc3500(QObject *parent) : QObject(parent)
{
    DLPC350_USB_Init();
    m_usbPollTimer = std::make_unique<QTimer>(this);
    m_usbPollTimer->setInterval(2000); // Poll every 2 seconds
    connect(m_usbPollTimer.get(), &QTimer::timeout, this, &Dlpc3500::pollTimerTimeout);
    m_usbPollTimer->start();
}
Dlpc3500::~Dlpc3500()
{
    DLPC350_USB_Exit();
}
void Dlpc3500::pollTimerTimeout()
{
    if (!DLPC350_USB_IsConnected())
    {
        if (DLPC350_USB_Open() == 0)
        {
            unsigned int imgCount = 0;
            if (DLPC350_GetNumImagesInFlash(&imgCount) == 0)
            {
                m_numImgInFlash = imgCount;
            }
            SetDLPC350InPatternMode();
        }
    }
    emit dlpcStatus(DLPC350_USB_IsConnected());
}
void Dlpc3500::SetDLPC350InPatternMode()
{
    // Implementation to set DLPC3500 in pattern mode
    int i = 0;
    bool mode;
    unsigned int patMode;
    constexpr int MAX_NUM_RETRIES = 10;
    // Check if it is in Pattern Mode
    DLPC350_GetMode(&mode);
    if (mode == false)
    {
        // Switch to Pattern Mode
        DLPC350_SetMode(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        while (true)
        {
            DLPC350_GetMode(&mode);
            if (mode)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (i++ > MAX_NUM_RETRIES)
                break;
        }
    }
    else
    {
        // First stop pattern sequence
        DLPC350_GetPatternDisplay(&patMode);
        // if it is in PAUSE or RUN mode
        if (patMode != 0)
        {
            stopProject();
        }
    }
    return;
}
void Dlpc3500::stopProject()
{
    if(!DLPC350_USB_IsConnected())
    {
        return;
    }
    int i = 0;
    unsigned int patMode;
    constexpr int MAX_NUM_RETRIES = 10;
    DLPC350_PatternDisplay(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (1)
    {
        DLPC350_GetPatternDisplay(&patMode);
        if (patMode == 0)
            break;
        else
            DLPC350_PatternDisplay(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (i++ > MAX_NUM_RETRIES)
            break;
    }
}
void Dlpc3500::updateFrequency(unsigned int index)
{
    // Implementation to update the pattern display frequency
    if (!DLPC350_USB_IsConnected())
    {
        return;
    }
    DLPC350_ClearPatLut();
    DLPC350_AddToPatLut(0, 0, 8, 1, false, false, true, false);
    DLPC350_AddToPatLut(0, 1, 8, 1, false, false, false, false);
    DLPC350_AddToPatLut(0, 2, 8, 1, false, false, false, false);
    DLPC350_SetPatternDisplayMode(false); // Set to internal trigger mode
    if (DLPC350_SetPatternConfig(3, true, 1, 1) < 0)
    {
        return;
    }
    if (DLPC350_SetExposure_FramePeriod(1000000, 1000000) < 0)
    {
        return;
    }
    if (DLPC350_SetPatternTriggerMode(1) < 0)
    {
        return;
    }
    if (DLPC350_SendPatLut() < 0)
    {
        return;
    }
    std::vector<unsigned char> splashLut = {4};
    if (DLPC350_SendImageLut(splashLut.data(), splashLut.size()) < 0)
    {
        return;
    }
    stopProject();
    if (DLPC350_StartPatLutValidate())
    {
        return;
    }
    QEventLoop loop;
    bool ready = false;
    unsigned int status = 0;
    constexpr int MAX_VALIDATE_RETRIES = 50;
    constexpr int VALIDATE_WAIT_MS = 1000;

    for (int attempt = 0; attempt < MAX_VALIDATE_RETRIES; ++attempt)
    {
        if (DLPC350_CheckPatLutValidate(&ready, &status) < 0)
        {
            return;
        }

        if (ready)
        {
            break;
        }

        QTimer::singleShot(VALIDATE_WAIT_MS, &loop, &QEventLoop::quit);
        loop.exec();
    }
    DLPC350_PatternDisplay(2);
}
