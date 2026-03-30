#pragma once

#include <QObject>
#include <QString>
#include <vector>
#include "CamControl.hpp"
#include "model_SFDI.hpp"
#include "lookup.hpp"

enum FrameState{
    FREQ_0P0 = 0,
    FREQ_0P120 = 1,
    FREQ_0P240 = 2,
    FREQ_1P0 = 3,
    FREQ_1P120 = 4,
    FREQ_1P240 = 5,
    FRAMENUM,
};

// 计算控制器基类，提供公共接口和成员，校准和计算类似，计算多出反演的功能
class BaseComputeCtrl : public QObject 
{
    Q_OBJECT
public:
    explicit BaseComputeCtrl(double int_time);
    virtual ~BaseComputeCtrl() = 0;
    void setIntTime(double int_time) { m_int_time = int_time; }
    void setSavePath(const std::string& path) { m_saveFolder = path; }
    const Eigen::ArrayXXd & getMdc() const { return m_Mdc; }
    const Eigen::ArrayXXd & getMac() const { return m_Mac; }
protected:
    std::string m_saveFolder; // 用于存储帧文件路径
    std::array<Eigen::ArrayXXd, FRAMENUM> m_inputs;
    double m_int_time;
    Eigen::ArrayXXd m_Mdc, m_Mac;
    FrameState m_currentFrameState = FRAMENUM;
public slots:
    virtual void frameGrabbed(const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &frame) = 0;
    void startRun() { m_currentFrameState = FREQ_0P0; }
signals:
    void saveFrame(const std::string& filename, const Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &data);
};

class CalibrationComputeCtrl : public BaseComputeCtrl
{
    Q_OBJECT
public:
    explicit CalibrationComputeCtrl(double int_time);
    ~CalibrationComputeCtrl() override;
public slots:
    void frameGrabbed(const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &frame);
signals:
    void calibrateComplete();
};

class MeasureComputeCtrl : public BaseComputeCtrl
{
    Q_OBJECT
public:
    explicit MeasureComputeCtrl(double int_time);
    ~MeasureComputeCtrl() override;
    void setCalibrationReflect(const SFDI::Reflect &calib) { m_calibrationReflect = calib; }
    void setCalibrationM(const Eigen::ArrayXXd &MDC, const Eigen::ArrayXXd &MAC) { m_calMdc = MDC; m_calMac = MAC; }
private:
    SFDI::SFDI_Lookup m_lookup;
    SFDI::Reflect m_calibrationReflect;
    Eigen::ArrayXXd m_calMdc,m_calMac; //校准得到的MDC和MAC图
    Eigen::ArrayXXd m_Rdc, m_Rac; //当前测量的反射率图
    Eigen::ArrayXXd m_mua, m_musp; //反演结果
    Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> m_mua_8bit, m_musp_8bit; //反演结果的8位图
public slots:
    void frameGrabbed(const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &frame);
signals:
    void measureComplete(const Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &mua_8bit,
                         const Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &musp_8bit);
};
class Saver: public QObject
{
    Q_OBJECT
public:
    Saver() = default;
public slots:
    void saveFrame(const std::string& filename, const Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &data)
    {
        SFDI::save_tiff(filename, data);
    }
};