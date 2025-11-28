#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "CameraApi.h"
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
private slots:
    void on_scanCamBtn_clicked();
    void on_controlCamBtn_clicked();
};

#endif // MAINWINDOW_H