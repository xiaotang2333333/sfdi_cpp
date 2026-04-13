#include "compute_ctrl.hpp"
#include "model_SFDI.hpp"

BaseComputeCtrl::BaseComputeCtrl(double int_time)
    : m_int_time(int_time)
{
}

BaseComputeCtrl::~BaseComputeCtrl()
{
}
void BaseComputeCtrl::convertFrameToEigen(const uint16_t *pData, int width, int height, Eigen::ArrayXXd &output)
{
    if (output.rows() != height || output.cols() != width)
    {
        output.resize(height, width);
    }

    for (int y = 0; y < height; ++y)
    {
        const int rowOffset = y * width;
        for (int x = 0; x < width; ++x)
        {
            output(y, x) = static_cast<double>(pData[rowOffset + x]) / 4095.0;
        }
    }
}

CalibrationComputeCtrl::CalibrationComputeCtrl(double int_time)
    : BaseComputeCtrl(int_time)
{
}

CalibrationComputeCtrl::~CalibrationComputeCtrl()
{
}

void CalibrationComputeCtrl::frameGrabbed(std::shared_ptr<uint8_t> data, int width, int height, int size)
{
    if (m_currentFrameState >= FRAMENUM)
    {
        return;
    }
    const uint16_t *pData = reinterpret_cast<const uint16_t *>(data.get());
    convertFrameToEigen(pData, width, height, m_inputs[m_currentFrameState]);

    emit saveFrame(m_saveFolder + "/frame_" + std::to_string(m_currentFrameState) + ".tiff",
                   std::make_shared<SaveImage>(m_inputs[m_currentFrameState]));
    if (m_currentFrameState == FREQ_0P0)
    {
        if (m_inputs[m_currentFrameState].size() != m_Mdc.size())
        {
            m_Mdc.resize(m_inputs[FREQ_0P0].rows(), m_inputs[FREQ_0P0].cols());
            m_Mac.resize(m_inputs[FREQ_0P0].rows(), m_inputs[FREQ_0P0].cols());
        }
    }
    if (m_currentFrameState == FREQ_1P240)
    {
        SFDI::Compute_Amplitude_Envelope(m_inputs[FREQ_0P0], m_inputs[FREQ_0P120], m_inputs[FREQ_0P240], m_int_time, m_Mdc);
        SFDI::Compute_Amplitude_Envelope(m_inputs[FREQ_1P0], m_inputs[FREQ_1P120], m_inputs[FREQ_1P240], m_int_time, m_Mac);
        emit calibrateComplete();
    }
    m_currentFrameState = static_cast<FrameState>(m_currentFrameState + 1);
}

MeasureComputeCtrl::MeasureComputeCtrl(double int_time)
    : BaseComputeCtrl(int_time), m_lookup(1001, "test.bin")
{
}

MeasureComputeCtrl::~MeasureComputeCtrl()
{
}

void MeasureComputeCtrl::frameGrabbed(std::shared_ptr<uint8_t> data, int width, int height, int size)
{
    constexpr double eps = 1e-8;
    if (m_currentFrameState >= FRAMENUM)
    {
        return;
    }
    const uint16_t *pData = reinterpret_cast<const uint16_t *>(data.get());
    convertFrameToEigen(pData, width, height, m_inputs[m_currentFrameState]);

    // emit saveFrame(m_saveFolder + "/frame_" + std::to_string(m_currentFrameState) + ".tiff",
    //                std::make_shared<SaveImage>(m_inputs[m_currentFrameState]));
    if (m_currentFrameState == FREQ_0P0)
    {
        if (m_inputs[m_currentFrameState].size() != m_Rdc.size())
        {
            m_Rdc.resize(m_inputs[FREQ_0P0].rows(), m_inputs[FREQ_0P0].cols());
            m_Rac.resize(m_inputs[FREQ_0P0].rows(), m_inputs[FREQ_0P0].cols());
            m_mua.resize(m_inputs[FREQ_0P0].rows(), m_inputs[FREQ_0P0].cols());
            m_musp.resize(m_inputs[FREQ_0P0].rows(), m_inputs[FREQ_0P0].cols());
            m_mua_8bit.resize(m_inputs[FREQ_0P0].rows(), m_inputs[FREQ_0P0].cols());
            m_musp_8bit.resize(m_inputs[FREQ_0P0].rows(), m_inputs[FREQ_0P0].cols());
        }
    }
    if (m_currentFrameState == FREQ_0P240)
    {
        SFDI::Compute_Amplitude_Envelope(m_inputs[FREQ_0P0], m_inputs[FREQ_0P120], m_inputs[FREQ_0P240], m_int_time, m_Mdc);
        if(m_Mdc.size() == m_calMdc.size())
        {
            m_Rdc = m_Mdc * m_calibrationReflect(0) / (m_calMdc + eps);
        }
    }
    if (m_currentFrameState == FREQ_1P240)
    {
        SFDI::Compute_Amplitude_Envelope(m_inputs[FREQ_1P0], m_inputs[FREQ_1P120], m_inputs[FREQ_1P240], m_int_time, m_Mac);
        if (m_Mdc.size() == m_calMdc.size() && m_Mac.size() == m_calMac.size())
        {
            m_Rac = m_Mac * m_calibrationReflect(1) / (m_calMac + eps);
#pragma omp parallel for
            for (int i = 0; i < m_Rdc.rows(); ++i)
            {
                for (int j = 0; j < m_Rdc.cols(); ++j)
                {

                    m_lookup.query(SFDI::Reflect(m_Rdc(i, j), m_Rac(i, j)), m_mua(i, j), m_musp(i, j));
                }
            }
            // emit saveFrame(m_saveFolder + "/mua.tiff", std::make_shared<SaveImage>(m_mua));
            // emit saveFrame(m_saveFolder + "/musp.tiff", std::make_shared<SaveImage>(m_musp));
            double mua_max = m_mua.maxCoeff();
            double musp_max = m_musp.maxCoeff();
            m_mua_8bit = (m_mua / mua_max * 255.0).cast<uint8_t>();
            m_musp_8bit = (m_musp / musp_max * 255.0).cast<uint8_t>();
            emit measureComplete(std::make_shared<MeasureImage>(m_mua_8bit),
                                 std::make_shared<MeasureImage>(m_musp_8bit));
        }
        m_currentFrameState = static_cast<FrameState>(FREQ_0P0);
    }
    else
    {
        m_currentFrameState = static_cast<FrameState>(m_currentFrameState + 1);
    }
}
void Saver::saveFrame(std::string filename, SaveImagePtr data)
{
    if (!data)
    {
        return;
    }

    QFileInfo fileInfo(QString::fromStdString(filename));
    QDir dir = fileInfo.absoluteDir();
    if (!dir.exists())
    {
        dir.mkpath(".");
    }
    SFDI::save_tiff(filename, *data);
}
