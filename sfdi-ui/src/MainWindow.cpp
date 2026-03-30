#include "MainWindow.hpp"
#include "ui_MainWindow.h"
#include <iostream>
#include <QtWidgets/QPushButton>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QPixmap>
#include <Eigen/Dense>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), m_camControl(Hardware::CamControl()), m_calibComputeCtrl(1.0), m_measureComputeCtrl(1.0) // 初始化标定和测量计算控制器，默认积分时间为1.0s
{
    ui->setupUi(this);
    ui->pushButton_ConnectCam->setCheckable(true);
    ui->pushButton_ConnectCam->setText("connect");
    m_camThread = new QThread(this);
    m_camControl.moveToThread(m_camThread);
    // Start frame-interval timer
    m_prevFrameMs = -1;
    m_frameTimer.start();

    m_sceneData[CamScene].scene = new QGraphicsScene(this);
    m_sceneData[CamScene].pixmapItem = m_sceneData[CamScene].scene->addPixmap(QPixmap());
    ui->graphicsView_CamView->setScene(m_sceneData[CamScene].scene);
    // 相机信号连接
    connect(&m_camControl, &Hardware::CamControl::frameReceived, this, &MainWindow::onImageGrabbed);
    connect(&m_camControl, &Hardware::CamControl::frameReceived, &m_calibComputeCtrl, &CalibrationComputeCtrl::frameGrabbed);
    connect(&m_camControl, &Hardware::CamControl::frameReceived, &m_measureComputeCtrl, &MeasureComputeCtrl::frameGrabbed);
    connect(m_camThread, &QThread::started, this, &MainWindow::onCamThreadStarted);
    connect(m_camThread, &QThread::finished, this, &MainWindow::onCamThreadFinished);
    // 创建标定结果显示场景
    m_sceneData[MuaScene].scene = new QGraphicsScene(this);
    m_sceneData[MuaScene].pixmapItem = m_sceneData[MuaScene].scene->addPixmap(QPixmap());
    ui->graphicsView_mua->setScene(m_sceneData[MuaScene].scene);
    m_sceneData[MuspScene].scene = new QGraphicsScene(this);
    m_sceneData[MuspScene].pixmapItem = m_sceneData[MuspScene].scene->addPixmap(QPixmap());
    ui->graphicsView_musp->setScene(m_sceneData[MuspScene].scene);
    connect(&m_measureComputeCtrl, &MeasureComputeCtrl::measureComplete, this, &MainWindow::updateMeasureMap);
    // 创建标定计算线程
    m_calibComputeThread = new QThread(this);
    m_calibComputeCtrl.moveToThread(m_calibComputeThread);
    connect(m_calibComputeThread, &QThread::started, &m_calibComputeCtrl, &CalibrationComputeCtrl::startRun);
    connect(&m_calibComputeCtrl, &CalibrationComputeCtrl::calibrateComplete, this, &MainWindow::onCalibrateComplete);
    // 创建计算线程
    m_measureComputeThread = new QThread(this);
    m_measureComputeCtrl.moveToThread(m_measureComputeThread);
    connect(m_measureComputeThread, &QThread::started, &m_measureComputeCtrl, &MeasureComputeCtrl::startRun);
    connect(&m_measureComputeCtrl, &MeasureComputeCtrl::measureComplete, this, &MainWindow::onMeasureComplete);
    // 保存线程
    m_saveThread = new QThread(this);
    m_saver.moveToThread(m_saveThread);
    connect(&m_calibComputeCtrl, &CalibrationComputeCtrl::saveFrame, &m_saver, &Saver::saveFrame);
    connect(&m_measureComputeCtrl, &MeasureComputeCtrl::saveFrame, &m_saver, &Saver::saveFrame);
    m_saveThread->start();
    // dlp连接轮询
    connect(&m_dlpc3500, &Hardware::Dlpc3500::dlpcStatus, this, [this](bool isConnected)
            { ui->checkBox_ProjectStatus->setChecked(isConnected); });
    // ui
    connect(this, &MainWindow::isCamConnected, ui->doubleSpinBox_CamExposeTime, [this](bool connected)
            { 
                ui->doubleSpinBox_CamExposeTime->setEnabled(connected); 
                if(connected)
                {
                    double minVal, maxVal;
                    std::tie(maxVal, minVal) = m_camControl.getCameraDoubleParametersMaxAndMin("ExposureTime");
                    ui->doubleSpinBox_CamExposeTime->setMinimum(minVal);
                    ui->doubleSpinBox_CamExposeTime->setMaximum(maxVal);
                } });
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
    // Clean up CamControl thread if running
    if (m_camThread)
    {
        m_camThread->quit();
        m_camThread->wait();
        m_camThread->deleteLater();
        m_camThread = nullptr;
    }
    // Clean up CalibrationComputeCtrl thread
    if (m_calibComputeThread)
    {
        m_calibComputeThread->quit();
        m_calibComputeThread->wait();
        m_calibComputeThread->deleteLater();
        m_calibComputeThread = nullptr;
    }
    // Clean up MeasureComputeCtrl thread
    if (m_measureComputeThread)
    {
        m_measureComputeThread->quit();
        m_measureComputeThread->wait();
        m_measureComputeThread->deleteLater();
        m_measureComputeThread = nullptr;
    }
    if (m_saveThread)
    {
        m_saveThread->quit();
        m_saveThread->wait();
        m_saveThread->deleteLater();
        m_saveThread = nullptr;
    }
    delete ui;
}
void MainWindow::on_pushButton_ScanCam_clicked()
{
    ui->pushButton_ScanCam->setEnabled(false);

    QString status;
    const IMV_DeviceList &deviceList = m_camControl.scanCameras();
    ui->comboxBox_CamSelector->clear();
    int cameraCount = static_cast<int>(deviceList.nDevNum);
    if (cameraCount == 0)
    {
        status = "No cameras found.";
        ui->pushButton_ConnectCam->setEnabled(false);
    }
    else
    {
        for (unsigned int i = 0; i < deviceList.nDevNum; ++i)
        {
            const IMV_DeviceInfo &devInfo = deviceList.pDevInfo[i];
            QString camLabel = QString("Camera %1: %2")
                                   .arg(i)
                                   .arg(QString::fromStdString(devInfo.modelName));
            ui->comboxBox_CamSelector->addItem(camLabel);
        }
        status = QString("Found %1 camera(s).").arg(cameraCount);
        ui->pushButton_ConnectCam->setEnabled(true);
    }
    ui->statusbar->showMessage(status);
    ui->pushButton_ScanCam->setEnabled(true);
}
void MainWindow::on_pushButton_ConnectCam_toggled(bool checked)
{
    ui->pushButton_ConnectCam->setEnabled(false);
    if (ui->comboxBox_CamSelector->currentIndex() < 0)
    {
        ui->statusbar->showMessage("No camera selected.");
        ui->pushButton_ConnectCam->setChecked(false);
    }
    else if (checked)
    {
        // Start CamControl in a worker thread
        if (m_camThread && m_camThread->isRunning())
        {
            ui->statusbar->showMessage("Camera already running.");
        }
        else
        {
            m_camThread->start();
            ui->pushButton_ConnectCam->setText("disconnect");
            ui->statusbar->showMessage("Camera connected (CamControl thread started).");
        }
    }
    else
    {
        if (m_camThread && m_camThread->isRunning())
        {
            m_camThread->quit();
            m_camThread->wait();
        }
        ui->pushButton_ConnectCam->setText("connect");
        ui->statusbar->showMessage("Camera disconnected (CamControl thread stopped).");
    }
    ui->pushButton_ConnectCam->setEnabled(true);
}

void MainWindow::onImageGrabbed(const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &frame)
{
    const int height = frame.rows();
    const int width = frame.cols();
    if (width <= 0 || height <= 0)
    {
        ui->statusbar->showMessage("Invalid frame received.");
        return;
    }
    // 检查图像尺寸是否变化，必要时重新分配QImage
    QImage &img = m_sceneData[CamScene].img;
    bool sizeChanged = (img.width() != width) || (img.height() != height);
    if (sizeChanged || img.format() != QImage::Format_Grayscale16)
    {
        img = QImage(width, height, QImage::Format_Grayscale16);
    }
    uint16_t *dst = reinterpret_cast<uint16_t *>(img.bits());
    const uint16_t *src = frame.data();

    for (int i = 0; i < width * height; ++i)
    {
        dst[i] = src[i] << 4; // 等价于 *16
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
    if (m_camThread)
    {
        double exposeTime = ui->doubleSpinBox_CamExposeTime->value();
        try
        {
            m_camControl.setCameraDoubleParameters("ExposureTime", exposeTime);
            ui->statusbar->showMessage(QString("Exposure time set to %1 us.").arg(exposeTime));
        }
        catch (const std::runtime_error &e)
        {
            ui->statusbar->showMessage(QString("Error setting exposure time: %1").arg(e.what()));
        }
    }
}
void MainWindow::on_checkBox_CamSyncDmd_toggled(bool checked)
{
    // switch cam to trigger mode if checked, free-run mode if unchecked
    if (m_camThread)
    {
        try
        {
            m_camControl.setCameraTriggerMode(checked);
            ui->statusbar->showMessage(QString("Camera trigger mode set to %1.").arg(checked ? "External Trigger" : "Free Run"));
            ui->pushButton_Calibrate->setEnabled(checked); // Enable calibration button when exposure time is set
        }
        catch (const std::runtime_error &e)
        {
            ui->statusbar->showMessage(QString("Error setting camera trigger mode: %1").arg(e.what()));
        }
    }
}
void MainWindow::on_pushButton_Calibrate_clicked()
{
    // Placeholder for calibration functionality
    ui->statusbar->showMessage("Calibration started.");
    // 校准时，dmd进行投影，相机进行拍摄，获取图像并进行处理以完成标定
    if (!m_camThread || !m_calibComputeThread)
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
        return;
    }
    ui->checkBox_CamSyncDmd->setEnabled(false);   // 关闭同步开关，避免标定过程中关闭相机触发
    ui->pushButton_Calibrate->setEnabled(false);  // 禁用标定按钮，避免重复点击
    ui->pushButton_UpdateFreq->setEnabled(false); // 禁用更新频率按钮，避免标定过程中修改频率
    // 线程开始
    m_calibComputeCtrl.setSavePath("calibration_frames"); // 设置标定帧的保存路径
    m_calibComputeCtrl.setIntTime(ui->doubleSpinBox_CamExposeTime->value());
    m_dlpc3500.stopProject();
    m_calibComputeThread->start();
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
    ui->statusbar->showMessage("Measurement started.");
    // 测量时，dmd进行投影，相机进行拍摄，获取图像并进行处理以完成测量
    if (!m_camThread || !m_measureComputeThread)
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
    ui->checkBox_CamSyncDmd->setEnabled(false);  // 关闭同步开关，避免测量过程中关闭相机触发
    ui->pushButton_Calibrate->setEnabled(false); // 禁用标定按钮，避免重复点击
    ui->pushButton_Measure->setEnabled(false);   // 禁用测量按钮，避免重复点击
    ui->pushButton_UpdateFreq->setEnabled(false);
    // 线程开始
    m_measureComputeCtrl.setSavePath("measurement_frames");                                         // 设置测量帧的保存路径
    m_measureComputeCtrl.setCalibrationM(m_calibComputeCtrl.getMdc(), m_calibComputeCtrl.getMac()); // 将标定结果传递给测量控制器
    m_measureComputeCtrl.setIntTime(ui->doubleSpinBox_CamExposeTime->value());
    m_measureComputeCtrl.setCalibrationReflect(SFDI::Reflect(0.5, 0.5)); // 将标定得到的反射率图传递给测量控制器
    m_dlpc3500.stopProject();
    m_measureComputeThread->start();
    m_dlpc3500.setRepeatMode(false);                                                     // 只测量一次
    unsigned char freqIndex = static_cast<unsigned char>(ui->spinBox_DmdIndex->value()); // Placeholder index
    m_dlpc3500.updateFrequency(freqIndex);
}
void MainWindow::updateMeasureMap(const Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &mua_8bit,
                                    const Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &musp_8bit)
{
    // 将计算结果转换为QImage并显示
    // 检查数组是否为空
    if (mua_8bit.size() == 0 || musp_8bit.size() == 0)
    {
        ui->statusbar->showMessage("Empty computation result received.");
        return;
    }

    // 获取数组尺寸
    const int muaRows = static_cast<int>(mua_8bit.rows());
    const int muaCols = static_cast<int>(mua_8bit.cols());
    const int muspRows = static_cast<int>(musp_8bit.rows());
    const int muspCols = static_cast<int>(musp_8bit.cols());
    bool sizeChanged = (m_sceneData[MuaScene].img.width() != muaCols) || (m_sceneData[MuaScene].img.height() != muaRows);
    // 处理并显示 mua 图像
    if (muaRows > 0 && muaCols > 0)
    {
        // 创建或重用 QImage
        if (sizeChanged || m_sceneData[MuaScene].img.format() != QImage::Format_Grayscale8)
        {
            m_sceneData[MuaScene].img = QImage(muaCols, muaRows, QImage::Format_Grayscale8);
        }
        std::memcpy(m_sceneData[MuaScene].img.bits(), mua_8bit.data(), muaRows * muaCols * sizeof(uint8_t));
        // 更新 pixmap
        if (sizeChanged)
        {
            m_sceneData[MuaScene].pixmap = QPixmap::fromImage(m_sceneData[MuaScene].img);
        }
        else
        {
            m_sceneData[MuaScene].pixmap.convertFromImage(m_sceneData[MuaScene].img);
        }
        // 更新 pixmapItem
        m_sceneData[MuaScene].pixmapItem->setPixmap(m_sceneData[MuaScene].pixmap);
        if (sizeChanged)
        {
            ui->graphicsView_mua->fitInView(m_sceneData[MuaScene].pixmapItem->boundingRect(), Qt::KeepAspectRatio);
        }
    }

    // 处理并显示 musp 图像
    if (muspRows > 0 && muspCols > 0)
    {
        // 创建或重用 QImage
        if (sizeChanged || m_sceneData[MuspScene].img.format() != QImage::Format_Grayscale8)
        {
            m_sceneData[MuspScene].img = QImage(muspCols, muspRows, QImage::Format_Grayscale8);
        }
        std::memcpy(m_sceneData[MuspScene].img.bits(), musp_8bit.data(), muspRows * muspCols * sizeof(uint8_t));
        // 更新 pixmap
        if (sizeChanged)
        {
            m_sceneData[MuspScene].pixmap = QPixmap::fromImage(m_sceneData[MuspScene].img);
        }
        else
        {
            m_sceneData[MuspScene].pixmap.convertFromImage(m_sceneData[MuspScene].img);
        }
        // 更新 pixmapItem
        m_sceneData[MuspScene].pixmapItem->setPixmap(m_sceneData[MuspScene].pixmap);
        // 更新Scene视图
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

void MainWindow::onCamThreadStarted()
{
    m_camControl.connectCamera(ui->comboxBox_CamSelector->currentIndex());
    setIsCamConnected(true);
}

void MainWindow::onCamThreadFinished()
{
    m_camControl.disconnectCamera(ui->comboxBox_CamSelector->currentIndex());
    setIsCamConnected(false);
}

void MainWindow::onCalibrateComplete()
{
    m_dlpc3500.stopProject();
    ui->statusbar->showMessage("Calibration complete.");
    m_calibComputeThread->quit();
    ui->checkBox_CamSyncDmd->setEnabled(true);   // 启用相机-DMD同步选项
    ui->pushButton_UpdateFreq->setEnabled(true); // 启用更新频率按钮
    ui->pushButton_Calibrate->setEnabled(true);  // 重新启用标定按钮
    ui->pushButton_Measure->setEnabled(true);    // 启用测量按钮
    m_dlpc3500.setRepeatMode(true);
}

void MainWindow::onMeasureComplete()
{
    m_dlpc3500.stopProject();
    ui->statusbar->showMessage("Measurement complete.");
    m_measureComputeThread->quit();
    ui->checkBox_CamSyncDmd->setEnabled(true);
    ui->pushButton_UpdateFreq->setEnabled(true); // 启用更新频率按钮
    ui->pushButton_Calibrate->setEnabled(true);  // 重新启用标定按钮
    ui->pushButton_Measure->setEnabled(true);    // 重新启用测量按钮
    m_dlpc3500.setRepeatMode(true);
}
