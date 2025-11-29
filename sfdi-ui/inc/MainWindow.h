#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QElapsedTimer>
#include "CameraApi.h"
#include <QThread>
#include "CamControl.hpp"
class QGraphicsScene;
class QGraphicsPixmapItem;
namespace Ui { class MainWindow; }
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    CameraHandle hCamera = -1;
    QThread *m_camThread = nullptr;
    Hardware::CamControl m_camControl;
    QGraphicsScene *m_camScene = nullptr;
    QGraphicsPixmapItem *m_camPixmapItem = nullptr;
    QElapsedTimer m_frameTimer; // monotonic timer for inter-frame timing
    qint64 m_prevFrameMs = -1;  // last frame timestamp in ms (elapsed from m_frameTimer.start())
    double m_lastFps = 0.0;
private slots:
    void on_scanCamBtn_clicked();
    void on_controlCamBtn_clicked();
    void onImageGrabbed(const Hardware::CamFrameArray &frame);
};

#endif // MAINWINDOW_H