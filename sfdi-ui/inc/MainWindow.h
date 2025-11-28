#pragma once
#ifndef MAINWINDOWS_H
#define MAINWINDOWS_H

#include <QtWidgets/QMainWindow>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOWS_H