#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <omp.h>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>
#include <QMouseEvent>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/exceptions.h"
#include "IO/glwidget.h"
#include "IO/glvertexmanager.h"
#include "IO/imageconverter.h"
#include "IO/VirtualSensor/virtualrgbdsensor.h"
#include "IO/FileReaders/readerfactory.h"
#include "PCWork/pcworker.h"
#include "Test/testbed.h"
#include "Test/testclusterer.h"

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
    void TryFrame();
    void on_pushButton_replay_clicked();
    void on_checkBox_timer_toggled(bool checked);
    void on_pushButton_resetView_clicked();
    void on_radioButton_view_color_toggled(bool checked);
    void on_radioButton_view_descriptor_toggled(bool checked);
    void on_radioButton_view_segment_toggled(bool checked);
    void on_radioButton_view_object_toggled(bool checked);
    void on_checkBox_normal_toggled(bool checked);
    void on_pushButton_test_clicked();
    void on_pushButton_virtual_depth_clicked();
    void on_comboBox_dataset_currentIndexChanged(int index);
    void on_pushButton_focus_up_clicked();
    void on_pushButton_focus_down_clicked();
    void on_pushButton_focus_left_clicked();
    void on_pushButton_focus_right_clicked();

protected:
    void mousePressEvent(QMouseEvent* e);

private:
    void InitViewers();
    void InitUI();
    void RunFrame();
    void DisplayImage(QImage colorImg, QImage depthImg);
    int GetViewOptions();
    void UpdateView();
    void CheckPixel(QPoint pixel);
    RgbdReaderInterface* CreateReader(const int DSID);

    Ui::MainWindow *ui;
    GlWidget* glwidget;
    QGraphicsScene* colorScene;
    QGraphicsScene* depthScene;
    QImage colorImg;
    QImage depthImg;
    Pose6dof framePose;
    SharedData sharedData;
    QTimer* timer;
    QPoint colorImgPos;
    QPoint depthImgPos;
    QElapsedTimer eltimer;
    QPoint mousePixel;

    PCWorker* pcworker;
    RgbdReaderInterface* reader;
};

#endif // MAINWINDOW_H
