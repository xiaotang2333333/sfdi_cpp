#pragma once
#include <QtWidgets/QMainWindow>
#include <QElapsedTimer>
#include <QThread>
#include "CamControl.hpp"
#include "DlpControl.hpp"
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
    Hardware::Dlpc3500 m_dlpc3500;
    Ui::MainWindow *ui;
    QThread *m_camThread = nullptr;
    Hardware::CamControl m_camControl;
    QGraphicsScene *m_camScene = nullptr;
    QGraphicsPixmapItem *m_camPixmapItem = nullptr;
    QElapsedTimer m_frameTimer; // monotonic timer for inter-frame timing
    qint64 m_prevFrameMs = -1;  // last frame timestamp in ms (elapsed from m_frameTimer.start())
    double m_lastFps = 0.0;
private slots:
    void on_pushButton_ScanCam_clicked();
    void on_pushButton_ConnectCam_toggled(bool checked);
    void on_pushButton_StopProject_clicked();
    void on_pushButton_UpdateFreq_clicked();
    void onImageGrabbed(IMV_Frame frame);
};
