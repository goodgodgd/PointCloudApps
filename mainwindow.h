#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QGraphicsScene>
#include <omp.h>
#include "PrjCommon/project_common.h"
#include "PrjCommon/glwidget.h"
#include "PrjCommon/glvertexmanager.h"
#include "rgbdfilerw.h"

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

    Ui::MainWindow *ui;
    GlWidget* m_glwidget;
    QGraphicsScene* m_colorScene;
    QGraphicsScene* m_depthScene;
    QVector3D** m_pointCloud;
};

#endif // MAINWINDOW_H
