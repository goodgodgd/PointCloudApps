#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <omp.h>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>
#include <QMouseEvent>
#include "Share/project_common.h"
#include "IO/glwidget.h"
#include "IO/glvertexmanager.h"
#include "IO/rgbdfilerw.h"
#include "IO/imageconverter.h"
#include "Share/sharedenums.h"
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
    void RunFrame();
    void on_pushButton_replay_clicked();
    void on_checkBox_timer_toggled(bool checked);
    void on_pushButton_resetView_clicked();
    void on_radioButton_view_color_toggled(bool checked);
    void on_radioButton_view_descriptor_toggled(bool checked);
    void on_radioButton_view_segment_toggled(bool checked);
    void on_radioButton_view_object_toggled(bool checked);
    void on_checkBox_normal_toggled(bool checked);

protected:
    void mousePressEvent(QMouseEvent* e);

private:
    void DisplayImage(QImage& colorImg, QImage& depthImg);
    int GetViewOptions();
    void UpdateView();
    void CheckPixel(QPoint point);

    Ui::MainWindow *ui;
    GlWidget* glwidget;
    QGraphicsScene* colorScene;
    QGraphicsScene* depthScene;
    QImage colorImg;
    QImage depthImg;
    cl_float4* pointCloud;
    eDBID dbID;
    QTimer* timer;
    QPoint colorImgPos;
    QPoint depthImgPos;

    PCWorker* pcworker;
};

#endif // MAINWINDOW_H
