#pragma once
#include <QtWidgets/QMainWindow>
#include <QElapsedTimer>
#include <QThread>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include "CamControl.hpp"
#include "DlpControl.hpp"
#include "compute_ctrl.hpp"

struct SceneData
{
    QGraphicsScene *scene = nullptr;
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

private:
    Hardware::Dlpc3500 m_dlpc3500;
    Ui::MainWindow *ui;
    //相机及其线程
    QThread *m_camThread = nullptr;
    Hardware::CamControl m_camControl;
    //标定计算控制器及其线程
    QThread *m_calibComputeThread = nullptr;
    CalibrationComputeCtrl m_calibComputeCtrl;
    // 测量计算控制器及其线程
    QThread *m_measureComputeThread = nullptr;
    MeasureComputeCtrl m_measureComputeCtrl;
    // 保存线程
    QThread *m_saveThread = nullptr;
    Saver m_saver;
    // FPS计算
    QElapsedTimer m_frameTimer; // monotonic timer for inter-frame timing
    qint64 m_prevFrameMs = -1;  // last frame timestamp in ms (elapsed from m_frameTimer.start())
    double m_lastFps = 0.0;
    // 帧处理标志：用于丢弃旧帧，避免事件队列堆积
    std::atomic<bool> m_frameProcessing{false};
    // 
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
    void onImageGrabbed(const Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &frame);
    void updateMeasureMap(const Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &mua_8bit,
                            const Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &musp_8bit);
    void updateDmdIndex(unsigned int imgCount);
    //相机回调
    void onCamThreadStarted();
    void onCamThreadFinished();
    //计算回调
    void onCalibrateComplete();
    void onMeasureComplete();
};
