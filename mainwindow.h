#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <omp.h>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>
#include "project_common.h"
#include "ClWork/cloperators.h"
#include "IO/glwidget.h"
#include "IO/glvertexmanager.h"
#include "IO/rgbdfilerw.h"
#include "PCWork/pcworker.h"
#include "Share/sharedenums.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void RunFrame();
    void on_pushButton_replay_clicked();
    void on_checkBox_timer_toggled(bool checked);
    void on_pushButton_resetView_clicked();

private:
    void DrawBackground();
    void ConvertDepthToPoint3D(cv::Mat depthMat, cl_float4* pointCloud);
    void DrawPointCloud(cl_float4* pointCloud);
    int GetViewOptions();

    Ui::MainWindow *ui;
    GlWidget* glwidget;
    QGraphicsScene* colorScene;
    QGraphicsScene* depthScene;
    cl_float4* pointCloud;
    eDBID dbID;
    QTimer* timer;

    PCWorker* pcworker;
};

#endif // MAINWINDOW_H
