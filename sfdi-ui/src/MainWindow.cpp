#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <iostream>
#include <QtWidgets/QPushButton>
#include <QtGui/QStandardItemModel>
namespace
{
    std::vector<tSdkCameraDevInfo> cameraList(10);
    INT cameraCount = 0;
}
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    if (SDK_UNSUCCESS(CameraSdkInit(1)))
    {
        std::cerr << "Failed to initialize Camera SDK!" << std::endl;
    }
    else
    {
        ui->statusbar->showMessage("Camera SDK initialized successfully.");
    }
}

MainWindow::~MainWindow()
{
    if(hCamera!= -1)
        CameraUnInit(hCamera);
    delete ui;
}
void MainWindow::on_scanCamBtn_clicked()
{
    if (SDK_UNSUCCESS(CameraEnumerateDevice(cameraList.data(), &cameraCount)))
    {
        ui->statusbar->showMessage("Failed to enumerate cameras.");
        return;
    }
    if (cameraCount <= 0)
    {
        ui->statusbar->showMessage("No cameras found.");
        return;
    }
    ui->statusbar->showMessage(QString("Found %1 camera(s).").arg(cameraCount));
    QStandardItemModel *model = new QStandardItemModel(cameraCount, 2, this);
    model->setHeaderData(0, Qt::Horizontal, "Camera Index");
    model->setHeaderData(1, Qt::Horizontal, "Friendly Name");
    for (INT i = 0; i < cameraCount; ++i)
    {
        model->setItem(i, 0, new QStandardItem(QString::number(i)));
        model->setItem(i, 1, new QStandardItem(QString::fromLatin1(cameraList[i].acFriendlyName)));
    }
    ui->camSelector->setModel(model);
}
void MainWindow::on_controlCamBtn_clicked()
{
    int index = ui->camSelector->currentIndex();
    if (cameraList[index].uInstance == -1)
    {
        ui->statusbar->showMessage("Invalid camera selection.");
        return;
    }
    if (ui->controlCamBtn->text() == "connect")
    {
        if (SDK_UNSUCCESS(CameraInit(&cameraList[index], -1, -1, &hCamera)))
        {
            ui->controlCamBtn->setText("disconnect");
            ui->statusbar->showMessage("Failed to initialize the selected camera.");
            return;
        }
        ui->statusbar->showMessage("Camera connected successfully.");
    }
    else
    {
        if(SDK_UNSUCCESS(CameraUnInit(hCamera)))
        {
            ui->controlCamBtn->setText("connect");
            ui->statusbar->showMessage("Failed to disconnect the camera.");
            return;
        }
        ui->statusbar->showMessage("Camera disconnected successfully.");
    }
}