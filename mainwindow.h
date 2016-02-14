#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <omp.h>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>
#include "project_common.h"
#include "operators.h"
#include "IO/glwidget.h"
#include "IO/glvertexmanager.h"
#include "IO/rgbdfilerw.h"
#include "PCWork/pcworker.h"

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
    void on_pushButton_Replay_clicked();
    void on_checkBox_timer_toggled(bool checked);
    void RunFrame();

private:
    void DrawBackground();
    void ConvertDepthToPoint3D(cv::Mat depthMat, cl_float4* pointCloud);
    void DrawPointCloud(cl_float4* pointCloud);

    Ui::MainWindow *ui;
    GlWidget* m_glwidget;
    QGraphicsScene* m_colorScene;
    QGraphicsScene* m_depthScene;
    cl_float4* m_pointCloud;
    eDBID m_dbID;
    QTimer* m_timer;

    PCWorker* m_pcworker;
};

#endif // MAINWINDOW_H
