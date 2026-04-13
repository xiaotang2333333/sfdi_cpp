#pragma once

#include <QObject>
#include <QString>
#include <QDir>
#include <QFileInfo>
#include <QElapsedTimer>
#include <QDebug>
#include <vector>
#include <memory>
#include "CamControl.hpp"
#include "model_SFDI.hpp"
#include "lookup.hpp"

using MeasureImage = Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MeasureImagePtr = std::shared_ptr<const MeasureImage>;
using SaveImage = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using SaveImagePtr = std::shared_ptr<const SaveImage>;

Q_DECLARE_METATYPE(MeasureImagePtr)
Q_DECLARE_METATYPE(SaveImagePtr)

enum FrameState
{
    FREQ_0P0 = 0,
    FREQ_0P120 = 1,
    FREQ_0P240 = 2,
    FREQ_1P0 = 3,
    FREQ_1P120 = 4,
    FREQ_1P240 = 5,
    FRAMENUM,
};

class BaseComputeCtrl : public QObject
{
    Q_OBJECT
public:
    explicit BaseComputeCtrl(double int_time);
    virtual ~BaseComputeCtrl() = 0;
    void setIntTime(double int_time) { m_int_time = int_time; }
    void setSavePath(const std::string &path) { m_saveFolder = path; }
    const Eigen::ArrayXXd &getMdc() const { return m_Mdc; }
    const Eigen::ArrayXXd &getMac() const { return m_Mac; }

protected:
    std::string m_saveFolder;
    std::array<Eigen::ArrayXXd, FRAMENUM> m_inputs;
    double m_int_time;
    Eigen::ArrayXXd m_Mdc, m_Mac;
    FrameState m_currentFrameState = FRAMENUM;
    void convertFrameToEigen(const uint16_t *pData, int width, int height, Eigen::ArrayXXd &output);
public slots:
    virtual void frameGrabbed(std::shared_ptr<uint8_t> data, int width, int height, int size) = 0;
    inline void startRun() { m_currentFrameState = FREQ_0P0; };
    inline void stopRun() { m_currentFrameState = FRAMENUM; }
signals:
    void saveFrame(const std::string &filename, SaveImagePtr data);
};

class CalibrationComputeCtrl : public BaseComputeCtrl
{
    Q_OBJECT
public:
    explicit CalibrationComputeCtrl(double int_time);
    ~CalibrationComputeCtrl() override;
public slots:
    void frameGrabbed(std::shared_ptr<uint8_t> data, int width, int height, int size) override;
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
    void setCalibrationM(const Eigen::ArrayXXd &MDC, const Eigen::ArrayXXd &MAC)
    {
        m_calMdc = MDC;
        m_calMac = MAC;
    }

private:
    SFDI::SFDI_Lookup m_lookup;
    SFDI::Reflect m_calibrationReflect;
    Eigen::ArrayXXd m_calMdc, m_calMac;
    Eigen::ArrayXXd m_Rdc, m_Rac;
    Eigen::ArrayXXd m_mua, m_musp;
    MeasureImage m_mua_8bit, m_musp_8bit;
public slots:
    void frameGrabbed(std::shared_ptr<uint8_t> data, int width, int height, int size) override;
signals:
    void measureComplete(MeasureImagePtr mua_8bit,
                         MeasureImagePtr musp_8bit);
};
class Saver : public QObject
{
    Q_OBJECT
public:
    Saver() = default;
public slots:
    void saveFrame(std::string filename, SaveImagePtr data);
};