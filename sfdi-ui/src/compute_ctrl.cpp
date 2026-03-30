#include "compute_ctrl.hpp"
#include "model_SFDI.hpp"

// BaseComputeCtrl implementation
BaseComputeCtrl::BaseComputeCtrl(double int_time)
    : m_int_time(int_time)
{
}

BaseComputeCtrl::~BaseComputeCtrl()
{
}

// CalibrationComputeCtrl implementation
CalibrationComputeCtrl::CalibrationComputeCtrl(double int_time)
    : BaseComputeCtrl(int_time)
{
}

CalibrationComputeCtrl::~CalibrationComputeCtrl()
{
}

void CalibrationComputeCtrl::frameGrabbed(const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &frame)
{
    if (m_currentFrameState >= FRAMENUM)
    {
        return;
    }
    m_inputs[m_currentFrameState] = (frame.cast<double>()) / 4095.0; // 12-bit相机，最大值4095，归一化到0-1范围
    emit saveFrame(m_saveFolder + "/frame_" + std::to_string(m_currentFrameState) + ".tiff", m_inputs[m_currentFrameState]);
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

// MeasureComputeCtrl implementation
MeasureComputeCtrl::MeasureComputeCtrl(double int_time)
    : BaseComputeCtrl(int_time), m_lookup(101, "test.bin")
{
}

MeasureComputeCtrl::~MeasureComputeCtrl()
{
}
void MeasureComputeCtrl::frameGrabbed(const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &frame)
{
    if (m_currentFrameState >= FRAMENUM)
    {
        return;
    }
    m_inputs[m_currentFrameState] = (frame.cast<double>()) / 4095.0; // 12-bit相机，最大值4095，归一化到0-1范围
    emit saveFrame(m_saveFolder + "/frame_" + std::to_string(m_currentFrameState) + ".tiff", m_inputs[m_currentFrameState]);
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
    if (m_currentFrameState == FREQ_1P240)
    {
        SFDI::Compute_Amplitude_Envelope(m_inputs[FREQ_0P0], m_inputs[FREQ_0P120], m_inputs[FREQ_0P240], m_int_time, m_Mdc);
        SFDI::Compute_Amplitude_Envelope(m_inputs[FREQ_1P0], m_inputs[FREQ_1P120], m_inputs[FREQ_1P240], m_int_time, m_Mac);
        if (m_Mdc.size() == m_calMdc.size() && m_Mac.size() == m_calMac.size())
        {
            m_Rdc = m_Mdc / m_calMdc * m_calibrationReflect(0);
            m_Rac = m_Mac / m_calMac * m_calibrationReflect(1);
            for (int i = 0; i < m_Rdc.rows(); ++i)
            {
                for (int j = 0; j < m_Rdc.cols(); ++j)
                {
                    m_lookup.query(SFDI::Reflect(m_Rdc(i, j), m_Rac(i, j)), m_mua(i, j), m_musp(i, j));
                }
            }
            double mua_max = m_mua.maxCoeff();
            double musp_max = m_musp.maxCoeff();
            m_mua_8bit = (m_mua / mua_max * 255.0).cast<uint8_t>();
            m_musp_8bit = (m_musp / musp_max * 255.0).cast<uint8_t>();
            emit measureComplete(m_mua_8bit, m_musp_8bit);
        }
    }
    m_currentFrameState = static_cast<FrameState>(m_currentFrameState + 1);
}