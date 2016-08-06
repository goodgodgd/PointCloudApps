#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QGLFunctions>
#include <QGLShaderProgram>
#include <QBasicTimer>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include "Share/project_common.h"
#include "glvertexmanager.h"

class GlWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GlWidget(QWidget *parent = 0);
    void ResetView();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);
    void timerEvent(QTimerEvent *e);

    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void wheelEvent(QWheelEvent* e);
    void keyPressEvent(QKeyEvent* e);

private:
    QBasicTimer timer;
    QGLShaderProgram program;
    QMatrix4x4 projection;
    QMatrix4x4 viewPose;
    QVector3D lightpos;
    QQuaternion rot_default;
    float rot_radius;
    QVector3D trn_default;
    QVector2D mousePress;

signals:

public slots:

};

#endif // GLWIDGET_H
