#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QGLFunctions>
#include <QGLShaderProgram>
#include <QBasicTimer>
#include <QMouseEvent>
#include <QDebug>
#include "glvertexmanager.h"

class GlWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GlWidget(QWidget *parent = 0);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void timerEvent(QTimerEvent *e);


private:
    QBasicTimer m_timer;
    QGLShaderProgram m_program;
    QMatrix4x4 m_projection;
    QMatrix4x4 m_viewPose;
    QQuaternion m_rot_default;
    QQuaternion m_rotation;
    float m_rot_radius;
    QVector3D m_translation;
    QVector3D m_trn_default;


    QVector2D m_mousePress;

signals:

public slots:

};

#endif // GLWIDGET_H
