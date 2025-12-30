#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <iostream>
#include <QtWidgets/QPushButton>
#include <QtGui/QStandardItemModel>
#include "CamControl.hpp"
#include <QGraphicsScene>
#include <QImage>
#include <QPixmap>
namespace
{
    std::vector<tSdkCameraDevInfo> cameraList(10);
    INT cameraCount = cameraList.size();
}
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
    if (SDK_UNSUCCESS(CameraSdkInit(1)))
    {
        std::cerr << "Failed to initialize Camera SDK!" << std::endl;
    }
    else
    {
        ui->statusbar->showMessage("Camera SDK initialized successfully.");
    }
    connect(&m_dlpc3500, &Hardware::Dlpc3500::dlpcStatus, this, [this](bool isConnected) {
        ui->checkBox_ProjectStatus->setChecked(isConnected);
    });
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
    int ret;
    ret = CameraEnumerateDevice(cameraList.data(), &cameraCount);
    if (SDK_UNSUCCESS(ret))
    {
        status = "Failed to enumerate cameras." + QString::number(ret);
    }
    else if (cameraCount <= 0)
    {
        status = "No cameras found.";
    }
    else
    {
        status = QString("Found %1 camera(s).").arg(cameraCount);

        // Only show camera friendly names in the selector (single-column)
        ui->comboxBox_CamSelector->clear();
        for (INT i = 0; i < cameraCount; ++i)
        {
            ui->comboxBox_CamSelector->addItem(QString::fromLatin1(cameraList[i].acFriendlyName));
        }
    }
    cameraCount = cameraList.size(); // sdk需要寻找完需要重置
    ui->statusbar->showMessage(status);
    ui->pushButton_ScanCam->setEnabled(true);
}

void MainWindow::on_pushButton_ConnectCam_toggled(bool checked)
{
    ui->pushButton_ConnectCam->setEnabled(false);
    int index = ui->comboxBox_CamSelector->currentIndex();
    if (index < 0 || index >= cameraCount)
    {
        ui->statusbar->showMessage("Please select a valid camera.");
    }
    else
    {
        if (cameraList[index].uInstance == -1)
        {
            ui->statusbar->showMessage("Invalid camera selection.");
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
                    m_camControl.setSdkCameraDevInfo(cameraList[index]);
                    m_camThread = new QThread(this);
                    m_camControl.moveToThread(m_camThread);
                    connect(&m_camControl, &Hardware::CamControl::imageGrabbed, this, &MainWindow::onImageGrabbed);
                    connect(m_camThread, &QThread::started, &m_camControl, &Hardware::CamControl::CamStartGrab);
                    connect(m_camThread, &QThread::finished, &m_camControl, &Hardware::CamControl::CamStopGrab);

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
    }
    ui->pushButton_ConnectCam->setEnabled(true);
}

void MainWindow::onImageGrabbed(const Hardware::CamFrameArray &frame)
{
    // Convert Eigen::Tensor<BYTE,3> ([height,width,channels]) to QImage and display
    if (frame.dimensions().size() != 3)
    {
        ui->statusbar->showMessage("Received frame with unexpected dimensions.");
        return;
    }
    const int height = static_cast<int>(frame.dimension(0));
    const int width = static_cast<int>(frame.dimension(1));
    const int channels = static_cast<int>(frame.dimension(2));
    if (width <= 0 || height <= 0)
    {
        ui->statusbar->showMessage("Empty frame received.");
        return;
    }

    const uchar *dataPtr = reinterpret_cast<const uchar *>(frame.data());

    QImage img;
    if (channels == 3)
    {
        // CameraImageProcess writes RGB888 (R,G,B order) into the buffer
        img = QImage(dataPtr, width, height, width * 3, QImage::Format_RGB888).copy();
    }
    else if (channels == 1)
    {
        img = QImage(dataPtr, width, height, width, QImage::Format_Grayscale8).copy();
    }
    else
    {
        ui->statusbar->showMessage("Unsupported channel count: " + QString::number(channels));
        return;
    }

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
    if (m_prevFrameMs > 0)
    {
        qint64 deltaMs = nowMs - m_prevFrameMs;
        if (deltaMs > 0)
        {
            m_lastFps = 1000.0 / static_cast<double>(deltaMs);
        }
    }
    m_prevFrameMs = nowMs;

    ui->statusbar->showMessage(QString("Frame displayed: %1x%2 | FPS: %3")
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