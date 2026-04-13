#include "MainWindow.hpp"
#include "ui_MainWindow.h"
#include "colormap.hpp"
#include <iostream>
#include <QtWidgets/QPushButton>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QPixmap>
#include <QSignalBlocker>
#include <QMetaType>
#include <Eigen/Dense>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(std::make_unique<Ui::MainWindow>()), m_camControl(Hardware::CamControl()), m_calibComputeCtrl(1.0), m_measureComputeCtrl(1.0)
{
    qRegisterMetaType<std::shared_ptr<uint8_t>>("std::shared_ptr<uint8_t>");
    qRegisterMetaType<MeasureImagePtr>("MeasureImagePtr");
    qRegisterMetaType<SaveImagePtr>("SaveImagePtr");
    ui->setupUi(this);
    ui->pushButton_ConnectCam->setCheckable(true);
    ui->pushButton_ConnectCam->setText("connect");
    m_camThread = std::make_unique<QThread>();
    m_camControl.moveToThread(m_camThread.get());
    m_camThread->start();
    // Start frame-interval timer
    m_prevFrameMs = -1;
    m_frameTimer.start();

    m_sceneData[CamScene].scene = std::make_unique<QGraphicsScene>();
    m_sceneData[CamScene].pixmapItem = m_sceneData[CamScene].scene->addPixmap(QPixmap());
    ui->graphicsView_CamView->setScene(m_sceneData[CamScene].scene.get());
    connect(this, &MainWindow::requestScan, &m_camControl, &Hardware::CamControl::scanCamerasRequest, Qt::QueuedConnection);
    connect(this, &MainWindow::requestConnectCamera, &m_camControl, &Hardware::CamControl::connectCameraRequest, Qt::QueuedConnection);
    connect(this, &MainWindow::requestDisconnectCamera, &m_camControl, &Hardware::CamControl::disconnectCameraRequest, Qt::QueuedConnection);
    connect(this, &MainWindow::requestSetCameraExposure, &m_camControl, &Hardware::CamControl::setCameraExposureTimeRequest, Qt::QueuedConnection);
    connect(this, &MainWindow::requestSetCameraTriggerMode, &m_camControl, &Hardware::CamControl::setCameraTriggerModeRequest, Qt::QueuedConnection);
    connect(&m_camControl, &Hardware::CamControl::frameReceivedForUI, this, &MainWindow::onImageGrabbed);
    connect(&m_camControl, &Hardware::CamControl::frameReceivedForCompute, &m_calibComputeCtrl, &CalibrationComputeCtrl::frameGrabbed);
    connect(&m_camControl, &Hardware::CamControl::frameReceivedForCompute, &m_measureComputeCtrl, &MeasureComputeCtrl::frameGrabbed);
    connect(&m_camControl, &Hardware::CamControl::scanCompleted, this, &MainWindow::onCameraScanCompleted);
    connect(&m_camControl, &Hardware::CamControl::connectionCompleted, this, &MainWindow::onCameraConnectionCompleted);
    connect(&m_camControl, &Hardware::CamControl::disconnectionCompleted, this, &MainWindow::onCameraDisconnectionCompleted);
    connect(&m_camControl, &Hardware::CamControl::exposureRangeReady, this, &MainWindow::onCameraExposureRangeReady);
    connect(&m_camControl, &Hardware::CamControl::parameterSetCompleted, this, &MainWindow::onCameraParameterSetCompleted);
    // 创建标定结果显示场景
    m_sceneData[MuaScene].scene = std::make_unique<QGraphicsScene>();
    m_sceneData[MuaScene].pixmapItem = m_sceneData[MuaScene].scene->addPixmap(QPixmap());
    ui->graphicsView_mua->setScene(m_sceneData[MuaScene].scene.get());
    m_sceneData[MuspScene].scene = std::make_unique<QGraphicsScene>();
    m_sceneData[MuspScene].pixmapItem = m_sceneData[MuspScene].scene->addPixmap(QPixmap());
    ui->graphicsView_musp->setScene(m_sceneData[MuspScene].scene.get());
    connect(&m_measureComputeCtrl, &MeasureComputeCtrl::measureComplete, this, &MainWindow::updateMeasureMap);
    // 创建标定计算线程
    m_calibComputeThread = std::make_unique<QThread>();
    m_calibComputeCtrl.moveToThread(m_calibComputeThread.get());
    m_calibComputeThread->start(); // 先启动计算线程，等待标定控制器接收帧数据后再开始标定计算
    connect(&m_calibComputeCtrl, &CalibrationComputeCtrl::calibrateComplete, this, &MainWindow::onCalibrateComplete);
    // 创建计算线程
    m_measureComputeThread = std::make_unique<QThread>();
    m_measureComputeCtrl.moveToThread(m_measureComputeThread.get());
    m_measureComputeThread->start(); // 先启动计算线程，等待测量控制器接收帧数据后再开始测量计算
    connect(&m_measureComputeCtrl, &MeasureComputeCtrl::measureComplete, this, &MainWindow::onSingleMeasureComplete);
    // 保存线程
    m_saveThread = std::make_unique<QThread>();
    m_saver.moveToThread(m_saveThread.get());
    connect(&m_calibComputeCtrl, &CalibrationComputeCtrl::saveFrame, &m_saver, &Saver::saveFrame);
    connect(&m_measureComputeCtrl, &MeasureComputeCtrl::saveFrame, &m_saver, &Saver::saveFrame);
    m_saveThread->start();
    // dlp连接轮询
    connect(&m_dlpc3500, &Hardware::Dlpc3500::dlpcStatus, this, [this](bool isConnected)
            { ui->checkBox_ProjectStatus->setChecked(isConnected); });
    // ui
    connect(this, &MainWindow::isCamConnected, ui->doubleSpinBox_CamExposeTime, [this](bool connected)
            { ui->doubleSpinBox_CamExposeTime->setEnabled(connected); });
    connect(this, &MainWindow::isCamConnected, ui->doubleSpinBox_CamGain, [this](bool connected)
            { ui->doubleSpinBox_CamGain->setEnabled(connected); });
    connect(this, &MainWindow::isCamConnected, ui->checkBox_AutoExpose, [this](bool connected)
            { ui->checkBox_AutoExpose->setEnabled(connected); });
    connect(this, &MainWindow::isCamConnected, ui->checkBox_AutoGain, [this](bool connected)
            { ui->checkBox_AutoGain->setEnabled(connected); });
    connect(this, &MainWindow::isCamConnected, ui->checkBox_CamSyncDmd, [this](bool connected)
            { ui->checkBox_CamSyncDmd->setEnabled(connected); });
    connect(&this->m_dlpc3500, &Hardware::Dlpc3500::flashIndexUpdated, this, &MainWindow::updateDmdIndex);
}

MainWindow::~MainWindow()
{
    if (m_camThread)
    {
        if (m_camConnected)
        {
            const int cameraIndex = ui->comboxBox_CamSelector->currentIndex();
            QMetaObject::invokeMethod(&m_camControl, "disconnectCameraRequest", Qt::BlockingQueuedConnection, Q_ARG(int, cameraIndex));
        }
        m_camThread->quit();
        m_camThread->wait();
        m_camThread.reset();
    }
    // Clean up CalibrationComputeCtrl thread
    if (m_calibComputeThread)
    {
        m_calibComputeThread->quit();
        m_calibComputeThread->wait();
        m_calibComputeThread.reset();
    }
    // Clean up MeasureComputeCtrl thread
    if (m_measureComputeThread)
    {
        m_measureComputeThread->quit();
        m_measureComputeThread->wait();
        m_measureComputeThread.reset();
    }
    if (m_saveThread)
    {
        m_saveThread->quit();
        m_saveThread->wait();
        m_saveThread.reset();
    }
}
void MainWindow::on_pushButton_ScanCam_clicked()
{
    ui->pushButton_ScanCam->setEnabled(false);
    ui->statusbar->showMessage("Scanning cameras...");
    emit requestScan();
}

void MainWindow::onCameraScanCompleted(const QStringList &cameraLabels, const QString &status)
{
    ui->comboxBox_CamSelector->clear();
    ui->comboxBox_CamSelector->addItems(cameraLabels);
    ui->pushButton_ConnectCam->setEnabled(!cameraLabels.isEmpty() || m_camConnected);
    ui->pushButton_ScanCam->setEnabled(true);
    ui->statusbar->showMessage(status);
}
void MainWindow::on_pushButton_ConnectCam_toggled(bool checked)
{
    ui->pushButton_ConnectCam->setEnabled(false);
    if (ui->comboxBox_CamSelector->currentIndex() < 0)
    {
        ui->statusbar->showMessage("No camera selected.");
        QSignalBlocker blocker(ui->pushButton_ConnectCam);
        ui->pushButton_ConnectCam->setChecked(false);
        ui->pushButton_ConnectCam->setEnabled(true);
    }
    else if (checked)
    {
        const int cameraIndex = ui->comboxBox_CamSelector->currentIndex();
        ui->statusbar->showMessage("Connecting camera...");
        emit requestConnectCamera(cameraIndex);
    }
    else
    {
        ui->statusbar->showMessage("Disconnecting camera...");
        const int cameraIndex = ui->comboxBox_CamSelector->currentIndex();
        emit requestDisconnectCamera(cameraIndex);
    }
}

void MainWindow::onCameraConnectionCompleted(bool success, const QString &status)
{
    if (success)
    {
        m_camConnected = true;
        ui->pushButton_ConnectCam->setText("disconnect");
        QSignalBlocker blocker(ui->pushButton_ConnectCam);
        ui->pushButton_ConnectCam->setChecked(true);
        setIsCamConnected(true);
    }
    else
    {
        m_camConnected = false;
        ui->pushButton_ConnectCam->setText("connect");
        QSignalBlocker blocker(ui->pushButton_ConnectCam);
        ui->pushButton_ConnectCam->setChecked(false);
        setIsCamConnected(false);
    }
    ui->pushButton_ConnectCam->setEnabled(true);
    ui->statusbar->showMessage(status);
}

void MainWindow::onCameraDisconnectionCompleted(bool success, const QString &status)
{
    if (success)
    {
        m_camConnected = false;
        ui->pushButton_ConnectCam->setText("connect");
        QSignalBlocker blocker(ui->pushButton_ConnectCam);
        ui->pushButton_ConnectCam->setChecked(false);
        setIsCamConnected(false);
    }
    else
    {
        ui->pushButton_ConnectCam->setChecked(true);
    }
    ui->pushButton_ConnectCam->setEnabled(true);
    ui->statusbar->showMessage(status);
}

void MainWindow::onCameraExposureRangeReady(bool success, double minValue, double maxValue, const QString &status)
{
    if (success)
    {
        if (minValue > maxValue)
        {
            std::swap(minValue, maxValue);
        }
        ui->doubleSpinBox_CamExposeTime->setMinimum(minValue);
        ui->doubleSpinBox_CamExposeTime->setMaximum(maxValue);
    }
    ui->statusbar->showMessage(status);
}

void MainWindow::onCameraParameterSetCompleted(bool success, const QString &status)
{
    Q_UNUSED(success)
    ui->statusbar->showMessage(status);
}

void MainWindow::onImageGrabbed(std::shared_ptr<uint8_t> data, int width, int height, int size)
{
    if (width <= 0 || height <= 0 || !data)
    {
        ui->statusbar->showMessage("Invalid frame received.");
        return;
    }
    QImage &img = m_sceneData[CamScene].img;
    bool sizeChanged = (img.width() != width) || (img.height() != height);
    if (sizeChanged || img.format() != QImage::Format_Grayscale16)
    {
        img = QImage(width, height, QImage::Format_Grayscale16);
    }
    const uint16_t *src = reinterpret_cast<const uint16_t *>(data.get());
    uint16_t *dst = reinterpret_cast<uint16_t *>(img.bits());
    const int dstStride = static_cast<int>(img.bytesPerLine() / sizeof(uint16_t));
    for (int y = 0; y < height; ++y)
    {
        const int srcRowOffset = y * width;
        const int dstRowOffset = y * dstStride;
        for (int x = 0; x < width; ++x)
        {
            dst[dstRowOffset + x] = static_cast<uint16_t>(src[srcRowOffset + x] << 4);
        }
    }

    if (sizeChanged)
    {
        m_sceneData[CamScene].pixmap = QPixmap::fromImage(img);
    }
    else
    {
        m_sceneData[CamScene].pixmap.convertFromImage(img);
    }
    m_sceneData[CamScene].pixmapItem->setPixmap(m_sceneData[CamScene].pixmap);
    if (sizeChanged)
    {
        ui->graphicsView_CamView->fitInView(m_sceneData[CamScene].pixmapItem->boundingRect(), Qt::KeepAspectRatio);
    }

    const qint64 nowMs = m_frameTimer.elapsed();
    if (m_prevFrameMs >= 0)
    {
        const qint64 deltaMs = nowMs - m_prevFrameMs;
        if (deltaMs > 0)
        {
            m_lastFps = 1000.0 / static_cast<double>(deltaMs);
        }
    }
    m_prevFrameMs = nowMs;

    ui->statusbar->showMessage(
        QString("Frame: %1x%2 | FPS: %3")
            .arg(width)
            .arg(height)
            .arg(m_lastFps, 0, 'f', 1));
}
void MainWindow::on_pushButton_StopProject_clicked()
{
    // Placeholder for stopping projection functionality
    ui->statusbar->showMessage("Projection stopped.");
    m_dlpc3500.stopProject();
    QMetaObject::invokeMethod(&m_calibComputeCtrl, "stopRun", Qt::QueuedConnection);
    QMetaObject::invokeMethod(&m_measureComputeCtrl, "stopRun", Qt::QueuedConnection);
}
void MainWindow::on_pushButton_UpdateFreq_clicked()
{
    // Example frequency index; in a real application, this could come from user input
    unsigned char freqIndex = static_cast<unsigned char>(ui->spinBox_DmdIndex->value()); // Placeholder index
    m_dlpc3500.updateFrequency(freqIndex);
    ui->statusbar->showMessage("Projection frequency updated.");
}
void MainWindow::setIsCamConnected(bool connected)
{
    emit isCamConnected(connected);
}
void MainWindow::on_doubleSpinBox_CamExposeTime_editingFinished()
{
    if (m_camConnected)
    {
        double exposeTime = ui->doubleSpinBox_CamExposeTime->value();
        ui->statusbar->showMessage(QString("Updating exposure time to %1 us...").arg(exposeTime));
        emit requestSetCameraExposure(exposeTime);
    }
}
void MainWindow::on_checkBox_CamSyncDmd_toggled(bool checked)
{
    // switch cam to trigger mode if checked, free-run mode if unchecked
    if (m_camConnected)
    {
        ui->statusbar->showMessage(QString("Updating camera trigger mode to %1...").arg(checked ? "External Trigger" : "Free Run"));
        emit requestSetCameraTriggerMode(checked);
    }
}
void MainWindow::on_pushButton_Calibrate_clicked()
{
    // Placeholder for calibration functionality
    ui->statusbar->showMessage("Calibration started.");
    // 校准时，dmd进行投影，相机进行拍摄，获取图像并进行处理以完成标定
    if (!m_camConnected || !m_calibComputeThread)
    {
        ui->statusbar->showMessage("Camera or Calibration thread not running.");
        return;
    }
    if (ui->checkBox_CamSyncDmd->isChecked())
    {
        ui->statusbar->showMessage("Camera-DMD sync is enabled. Starting calibration...");
    }
    else
    {
        ui->statusbar->showMessage("Camera-DMD sync is disabled. Calibration may not be accurate.");
        // return;
    }
    ui->checkBox_CamSyncDmd->setEnabled(false);   // 关闭同步开关，避免标定过程中关闭相机触发
    ui->pushButton_Calibrate->setEnabled(false);  // 禁用标定按钮，避免重复点击
    ui->pushButton_UpdateFreq->setEnabled(false); // 禁用更新频率按钮，避免标定过程中修改频率
    // 线程开始
    m_calibComputeCtrl.setSavePath("./calibration_frames"); // 设置标定帧的保存路径
    m_calibComputeCtrl.setIntTime(ui->doubleSpinBox_CamExposeTime->value());
    QMetaObject::invokeMethod(&m_calibComputeCtrl, "startRun", Qt::QueuedConnection);
    m_dlpc3500.stopProject();
    m_dlpc3500.setRepeatMode(false);
    unsigned char freqIndex = static_cast<unsigned char>(ui->spinBox_DmdIndex->value()); // Placeholder index
    m_dlpc3500.updateFrequency(freqIndex);
}
void MainWindow::updateDmdIndex(unsigned int imgCount)
{
    ui->spinBox_DmdIndex->setMaximum(static_cast<int>(imgCount) - 1);
    ui->statusbar->showMessage(QString("DMD flash index updated. Total patterns: %1").arg(imgCount));
}

void MainWindow::on_spinBox_DmdExposeTime_editingFinished()
{
    unsigned int exposeTime = static_cast<unsigned int>(ui->spinBox_DmdExposeTime->value());
    if (exposeTime < 8333)
    {
        exposeTime = 8333;
        ui->spinBox_DmdExposeTime->setValue(exposeTime);
    }
    m_dlpc3500.setDmdExposeTime(exposeTime);
    ui->statusbar->showMessage(QString("DMD exposure time set to %1 us.").arg(exposeTime));
}
void MainWindow::on_pushButton_Measure_clicked()
{
    // Placeholder for measurement functionality
    if (ui->pushButton_Measure->text() == "Measure")
    {
        ui->statusbar->showMessage("Measurement started.");
        // 测量时，dmd进行投影，相机进行拍摄，获取图像并进行处理以完成测量
        if (!m_camConnected || !m_measureComputeThread)
        {
            ui->statusbar->showMessage("Camera or Measurement thread not running.");
            return;
        }
        if (ui->checkBox_CamSyncDmd->isChecked())
        {
            ui->statusbar->showMessage("Camera-DMD sync is enabled. Starting measurement...");
        }
        else
        {
            ui->statusbar->showMessage("Camera-DMD sync is disabled. Measurement may not be accurate.");
            return;
        }
        ui->checkBox_CamSyncDmd->setEnabled(false);      // 关闭同步开关，避免测量过程中关闭相机触发
        ui->checkBox_continueMeasure->setEnabled(false); // 关闭连续测量选项，避免测量过程中修改测量模式
        ui->pushButton_Calibrate->setEnabled(false);     // 禁用标定按钮，避免重复点击
        ui->pushButton_Measure->setEnabled(false);       // 禁用测量按钮，避免重复点击
        ui->pushButton_UpdateFreq->setEnabled(false);
        // 线程开始
        m_measureComputeCtrl.setSavePath("./measurement_frames");                                       // 设置测量帧的保存路径
        m_measureComputeCtrl.setCalibrationM(m_calibComputeCtrl.getMdc(), m_calibComputeCtrl.getMac()); // 将标定结果传递给测量控制器
        m_measureComputeCtrl.setIntTime(ui->doubleSpinBox_CamExposeTime->value());
        m_measureComputeCtrl.setCalibrationReflect(SFDI::Reflect(0.5, 0.5)); // 将标定得到的反射率图传递给测量控制器
        m_dlpc3500.stopProject();
        m_dlpc3500.setRepeatMode(ui->checkBox_continueMeasure->isChecked()); // 连续测量
        QMetaObject::invokeMethod(&m_measureComputeCtrl, "startRun", Qt::QueuedConnection);

        unsigned char freqIndex = static_cast<unsigned char>(ui->spinBox_DmdIndex->value()); // Placeholder index
        m_dlpc3500.updateFrequency(freqIndex);
        ui->pushButton_Measure->setText("Stop Measure");
    }
    else
    {
        ui->statusbar->showMessage("Measurement stopping...");
        m_dlpc3500.stopProject();
        QMetaObject::invokeMethod(&m_measureComputeCtrl, "stopRun", Qt::QueuedConnection);
        ui->checkBox_CamSyncDmd->setEnabled(true);
        ui->pushButton_UpdateFreq->setEnabled(true); // 启用更新频率按钮
        ui->pushButton_Calibrate->setEnabled(true);  // 重新启用标定按钮
        ui->pushButton_Measure->setEnabled(true);    // 重新启用测量按钮
        ui->checkBox_continueMeasure->setEnabled(true);
        m_dlpc3500.setRepeatMode(true);
        ui->pushButton_Measure->setText("Measure");
    }
    ui->pushButton_Measure->setEnabled(true);
}
void MainWindow::updateMeasureMap(MeasureImagePtr mua_8bit,
                                  MeasureImagePtr musp_8bit)
{
    if (!mua_8bit || !musp_8bit)
    {
        ui->statusbar->showMessage("Empty computation result received.");
        return;
    }

    static const auto infernoLUT = Colormap::createInfernoLUT();

    const int muaRows = static_cast<int>(mua_8bit->rows());
    const int muaCols = static_cast<int>(mua_8bit->cols());
    const int muspRows = static_cast<int>(musp_8bit->rows());
    const int muspCols = static_cast<int>(musp_8bit->cols());
    bool sizeChanged = (m_sceneData[MuaScene].img.width() != muaCols) || (m_sceneData[MuaScene].img.height() != muaRows);

    if (muaRows > 0 && muaCols > 0)
    {
        m_sceneData[MuaScene].img = Colormap::applyColormap(*mua_8bit, infernoLUT);
        m_sceneData[MuaScene].pixmap = QPixmap::fromImage(m_sceneData[MuaScene].img);
        m_sceneData[MuaScene].pixmapItem->setPixmap(m_sceneData[MuaScene].pixmap);
        if (sizeChanged)
        {
            ui->graphicsView_mua->fitInView(m_sceneData[MuaScene].pixmapItem->boundingRect(), Qt::KeepAspectRatio);
        }
    }

    if (muspRows > 0 && muspCols > 0)
    {
        m_sceneData[MuspScene].img = Colormap::applyColormap(*musp_8bit, infernoLUT);
        m_sceneData[MuspScene].pixmap = QPixmap::fromImage(m_sceneData[MuspScene].img);
        m_sceneData[MuspScene].pixmapItem->setPixmap(m_sceneData[MuspScene].pixmap);
        if (sizeChanged)
        {
            ui->graphicsView_musp->fitInView(m_sceneData[MuspScene].pixmapItem->boundingRect(), Qt::KeepAspectRatio);
        }
    }

    ui->statusbar->showMessage(QString("Computation complete. Mua: %1x%2, Musp: %3x%4")
                                   .arg(muaCols)
                                   .arg(muaRows)
                                   .arg(muspCols)
                                   .arg(muspRows));
}

void MainWindow::onCalibrateComplete()
{
    ui->statusbar->showMessage("Calibration complete.");
    ui->checkBox_CamSyncDmd->setEnabled(true);   // 启用相机-DMD同步选项
    ui->pushButton_UpdateFreq->setEnabled(true); // 启用更新频率按钮
    ui->pushButton_Calibrate->setEnabled(true);  // 重新启用标定按钮
    ui->pushButton_Measure->setEnabled(true);    // 启用测量按钮
    m_dlpc3500.setRepeatMode(true);
}

void MainWindow::onSingleMeasureComplete()
{
    // 只有单一测量模式下才会调用这个函数，连续测量模式下每次测量完成后都会调用updateMeasureMap来更新图像显示，而不是调用这个函数
    if (ui->checkBox_continueMeasure->isChecked())
    {
        return;
    }
    ui->statusbar->showMessage("Measurement complete.");
    ui->checkBox_CamSyncDmd->setEnabled(true);
    ui->pushButton_UpdateFreq->setEnabled(true); // 启用更新频率按钮
    ui->pushButton_Calibrate->setEnabled(true);  // 重新启用标定按钮
    ui->pushButton_Measure->setEnabled(true);    // 重新启用测量按钮
    ui->checkBox_continueMeasure->setEnabled(true);
    QMetaObject::invokeMethod(&m_measureComputeCtrl, "stopRun", Qt::QueuedConnection);
    m_dlpc3500.setRepeatMode(true);
}
