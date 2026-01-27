#include "MainWindow.hpp"
#include "ui_MainWindow.h"
#include <iostream>
#include <QtWidgets/QPushButton>
#include <QGraphicsScene>
#include <QImage>
#include <QPixmap>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // Make the connect button checkable and ensure initial label
    ui->pushButton_ConnectCam->setCheckable(true);
    ui->pushButton_ConnectCam->setText("connect");
    // Start frame-interval timer
    m_prevFrameMs = -1;
    m_frameTimer.start();
    connect(&m_dlpc3500, &Hardware::Dlpc3500::dlpcStatus, this, [this](bool isConnected)
            { ui->checkBox_ProjectStatus->setChecked(isConnected); });
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
    }
    ui->statusbar->showMessage(status);
    ui->pushButton_ScanCam->setEnabled(true);
}

void MainWindow::on_pushButton_ConnectCam_toggled(bool checked)
{
    ui->pushButton_ConnectCam->setEnabled(false);
    int index = ui->comboxBox_CamSelector->currentIndex();
    if (index < 0 || index >= ui->comboxBox_CamSelector->count())
    {
        ui->statusbar->showMessage("Please select a valid camera.");
    }
    else
    {
        if (checked)
        {
            // Start CamControl in a worker thread
            if (m_camThread)
            {
                ui->statusbar->showMessage("Camera already running.");
            }
            else
            {
                m_camThread = new QThread(this);
                m_camControl.moveToThread(m_camThread);
                connect(&m_camControl, &Hardware::CamControl::frameReceived, this, &MainWindow::onImageGrabbed);
                connect(m_camThread, &QThread::started, this, [this, index]()
                        { m_camControl.connectCamera(index); });
                connect(m_camThread, &QThread::finished, &m_camControl, [this, index]()
                        { m_camControl.disconnectCamera(index); });

                m_camThread->start();

                ui->pushButton_ConnectCam->setText("disconnect");
                ui->statusbar->showMessage("Camera connected (CamControl thread started).");
            }
        }
        else
        {
            // Stop CamControl's work and stop the thread. Do NOT delete the member object.
            if (m_camThread)
            {
                m_camThread->quit();
                m_camThread->wait();
                m_camThread->deleteLater();
                m_camThread = nullptr;
            }
            ui->pushButton_ConnectCam->setText("connect");
            ui->statusbar->showMessage("Camera disconnected (CamControl thread stopped).");
        }
    }
    ui->pushButton_ConnectCam->setEnabled(true);
}

void MainWindow::onImageGrabbed(IMV_Frame frame)
{
    // Convert Eigen::Tensor<BYTE,3> ([height,width,channels]) to QImage and display
    const int height = frame.frameInfo.height;
    const int width = frame.frameInfo.width;
    if (width <= 0 || height <= 0)
    {
        ui->statusbar->showMessage("Empty frame received.");
        return;
    }
    
    // 12 to 16 bits per pixel grayscale image
    for(int i = 0; i < width * height; ++i)
    {
        // Shift 12-bit data to 16-bit by left-shifting 4 bits
        reinterpret_cast<uint16_t*>(frame.pData)[i] = reinterpret_cast<uint16_t*>(frame.pData)[i] << 4;
    }
    QImage img(width, height, QImage::Format_Grayscale16);
    std::memcpy(img.bits(), frame.pData, frame.frameInfo.size);
    // Ensure scene exists and update with new pixmap
    if (!m_camScene)
    {
        m_camScene = new QGraphicsScene(this);
        ui->graphicsView_CamView->setScene(m_camScene);
    }
    m_camScene->clear();
    QPixmap pix = QPixmap::fromImage(img);
    m_camScene->addPixmap(pix);
    // Fit the image in view while keeping aspect ratio
    ui->graphicsView_CamView->fitInView(m_camScene->itemsBoundingRect(), Qt::KeepAspectRatio);

    // Compute FPS as inverse of inter-frame interval (1 / delta_seconds)
    qint64 nowMs = m_frameTimer.elapsed();
    if (m_prevFrameMs >= 0)
    {
        qint64 deltaMs = nowMs - m_prevFrameMs;
        if (deltaMs > 0)
        {
            m_lastFps = 1000.0 / static_cast<double>(deltaMs);
        }
    }
    m_prevFrameMs = nowMs;

    ui->statusbar->showMessage(QString("Frame displayed: %1 x %2 | FPS: %3")
                                   .arg(width)
                                   .arg(height)
                                   .arg(m_lastFps, 0, 'f', 2));
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
    unsigned int freqIndex = 0; // Placeholder index
    m_dlpc3500.updateFrequency(freqIndex);
    ui->statusbar->showMessage("Projection frequency updated.");
}