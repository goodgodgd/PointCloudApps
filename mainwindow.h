#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <omp.h>
#include "project_common.h"
#include "IO/glwidget.h"
#include "IO/glvertexmanager.h"
#include "IO/rgbdfilerw.h"

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

private:
    void DrawBackground();
    void ConvertDepthToPoint3D(cv::Mat depthMat, QVector3D** pointCloud);
    void DrawPointCloud(QVector3D** pointCloud);

    Ui::MainWindow *ui;
    GlWidget* m_glwidget;
    QGraphicsScene* m_colorScene;
    QGraphicsScene* m_depthScene;
    QVector3D** m_pointCloud;
    eDBID m_dbID;
};

#endif // MAINWINDOW_H
