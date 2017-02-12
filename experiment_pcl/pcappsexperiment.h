#ifndef EXPERIMENTWINDOW_H
#define EXPERIMENTWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>
#include <QMouseEvent>
#include <QThread>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/shared_data.h"
#include "ShareExpm/expm_common.h"
#include "IO/glwidget.h"
#include "IO/glvertexmanager.h"
#include "IO/imageconverter.h"
#include "IO/drawutils.h"
#include "IO/VirtualSensor/virtualrgbdsensor.h"
#include "IOExpm/expmreaderfactory.h"
#include "Experiment/experimenter.h"

namespace Ui {
class PCAppsExperiment;
}

class PCAppsExperiment : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCAppsExperiment(QWidget *parent = 0);
    ~PCAppsExperiment();

private slots:
    void on_pushButton_virtual_depth_clicked();
    void on_pushButton_resetView_clicked();
    void on_radioButton_view_color_toggled(bool checked);
    void on_radioButton_view_our_curvature_toggled(bool checked);
    void on_radioButton_view_fpfh_toggled(bool checked);
    void on_radioButton_view_shot_toggled(bool checked);
    void on_checkBox_normal_toggled(bool checked);
    void on_pushButton_replay_clicked();
    void on_checkBox_timer_toggled(bool checked);
    void on_radioButton_data_scenes_toggled(bool checked);
    void on_radioButton_data_objects_toggled(bool checked);

    void on_comboBox_dataset_changed(int index);
    void TryFrame();

protected:
    void mousePressEvent(QMouseEvent* e);

private:
    void InitViewers();
    void InitUI();
    void InitPaths();
    void RunFrame();

    RgbdReaderInterface* CreateSceneReader(const bool checked, const int index);
    int GetCameraType(const int dataIndex);
    void DisplayImage(QImage colorImg, QImage depthImg);
    int GetViewOptions();
    void UpdateView();

    Ui::PCAppsExperiment *ui;
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

    Experimenter* experimenter;
    RgbdReaderInterface* reader;
    QStringList dataPaths;
};

#endif // EXPERIMENTWINDOW_H
