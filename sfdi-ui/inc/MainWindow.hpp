#pragma once
#include <QtWidgets/QMainWindow>
#include <QElapsedTimer>
#include <QThread>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <memory>
#include "CamControl.hpp"
#include "DlpControl.hpp"
#include "compute_ctrl.hpp"

struct SceneData
{
    std::unique_ptr<QGraphicsScene> scene;
    QGraphicsPixmapItem *pixmapItem = nullptr;
    QPixmap pixmap;
    QImage img; 
    SceneData() = default;
};
enum SceneIndex
{
    CamScene = 0,
    FreqScene = 1,
    MuaScene = 2,
    MuspScene = 3,
    SCENENUM
};
namespace Ui { class MainWindow; }
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
signals:
    void isCamConnected(bool connected);
    void requestScan();
    void requestConnectCamera(int cameraIndex);
    void requestDisconnectCamera(int cameraIndex);
    void requestSetCameraExposure(double exposureTime);
    void requestSetCameraTriggerMode(bool enableExternalTrigger);

private:
    Hardware::Dlpc3500 m_dlpc3500;
    std::unique_ptr<Ui::MainWindow> ui;
    std::unique_ptr<QThread> m_camThread;
    Hardware::CamControl m_camControl;
    std::unique_ptr<QThread> m_calibComputeThread;
    CalibrationComputeCtrl m_calibComputeCtrl;
    std::unique_ptr<QThread> m_measureComputeThread;
    MeasureComputeCtrl m_measureComputeCtrl;
    std::unique_ptr<QThread> m_saveThread;
    Saver m_saver;
    QElapsedTimer m_frameTimer;
    qint64 m_prevFrameMs = -1;
    double m_lastFps = 0.0;
    bool m_camConnected = false;
    void setIsCamConnected(bool connected);
    SceneData m_sceneData[SCENENUM];
    
private slots:
    void on_pushButton_ScanCam_clicked();
    void on_pushButton_ConnectCam_toggled(bool checked);
    void on_pushButton_StopProject_clicked();
    void on_pushButton_UpdateFreq_clicked();
    void on_pushButton_Calibrate_clicked();
    void on_pushButton_Measure_clicked();
    void on_doubleSpinBox_CamExposeTime_editingFinished();
    void on_checkBox_CamSyncDmd_toggled(bool checked);
    void on_spinBox_DmdExposeTime_editingFinished();
    void onImageGrabbed(std::shared_ptr<uint8_t> data, int width, int height, int size);
    void onCameraScanCompleted(const QStringList &cameraLabels, const QString &status);
    void onCameraConnectionCompleted(bool success, const QString &status);
    void onCameraDisconnectionCompleted(bool success, const QString &status);
    void onCameraExposureRangeReady(bool success, double minValue, double maxValue, const QString &status);
    void onCameraParameterSetCompleted(bool success, const QString &status);
    void updateMeasureMap(MeasureImagePtr mua_8bit,
                          MeasureImagePtr musp_8bit);
    void updateDmdIndex(unsigned int imgCount);
    void onCalibrateComplete();
    void onSingleMeasureComplete();
};
