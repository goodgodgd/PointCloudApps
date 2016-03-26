#include "glwidget.h"

GlWidget::GlWidget(QWidget *parent) :
    QGLWidget(parent)
{
    lightpos = QVector3D(0, 0, 3);
    rot_default.setVector(0, 0, 0.01);
    rot_default.setScalar(1);
    rot_default.normalize();
    trn_default = QVector3D(-1, 0, 0);
    rot_radius = 0.1f;

    projection.setToIdentity();
    projection.perspective(45, 4.f/3.f, 0.1f, 100.f);
    viewPose.setToIdentity();
    viewPose.translate(trn_default);
    viewPose.rotate(rot_default);

    setFocusPolicy(Qt::StrongFocus);
}

void GlWidget::ResetView()
{
    viewPose.setToIdentity();
    viewPose.translate(trn_default);
    viewPose.rotate(rot_default);
}

void GlWidget::initializeGL()
{
    glClearColor(0,0,0,1);

    // link shaders
    program.addShaderFromSourceFile(QGLShader::Vertex, ":Resources/vertex.vsh");
    program.addShaderFromSourceFile(QGLShader::Fragment, ":Resources/fragment.fsh");
    program.link();

    // Use QBasicTimer because its faster than QTimer
    timer.start(30, this);
}

void GlWidget::paintGL()
{
    QPainter painter;
    painter.begin(this);
    painter.beginNativePainting();

    glClearColor(0,0,0,1); // this must be the first among gl_functions
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glFrontFace(GL_CW);
    glCullFace(GL_FRONT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    // set viewMatrix from viewPose
    QMatrix4x4 viewMatrix;
    QVector3D localPoint;
    localPoint = QVector3D(0,0,0);
    QVector3D eye = viewPose.map(localPoint);
    localPoint = QVector3D(1,0,0);
    QVector3D center = viewPose.map(localPoint);
    localPoint = QVector3D(0,0,1);
    QVector3D up = viewPose.map(localPoint) - eye;
    viewMatrix.setToIdentity();
    viewMatrix.lookAt(eye,center,up);

    program.bind();
    program.enableAttributeArray("in_posit");
    program.enableAttributeArray("in_normal");
    program.enableAttributeArray("in_color");
    program.enableAttributeArray("in_ptsize");

    program.setUniformValue("in_mvpmat", projection * viewMatrix);
    program.setUniformValue("in_lightpos", lightpos);
    program.setAttributeArray("in_posit", gvm::PositPtr());
    program.setAttributeArray("in_normal", gvm::NormalPtr());
    program.setAttributeArray("in_color", gvm::ColorPtr());
    program.setAttributeArray("in_ptsize", gvm::PtsizePtr(), 1);

    glDrawArrays(GL_POINTS, gvm::PtBegin(), gvm::PtNum());
    glDrawArrays(GL_LINES, gvm::LnBegin(), gvm::LnNum());
    glDrawArrays(GL_TRIANGLES, gvm::TrBegin(), gvm::TrNum());

    program.disableAttributeArray("in_posit");
    program.disableAttributeArray("in_normal");
    program.disableAttributeArray("in_color");
    program.disableAttributeArray("in_ptsize");
    program.release();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    painter.endNativePainting();
//    painter.drawLine(100,100,100,300);
    painter.end();

    swapBuffers();
}

void GlWidget::resizeGL(int w, int h)
{
    // Set OpenGL viewport to cover whole widget
    glViewport(0, 0, w, h);
}

void GlWidget::timerEvent(QTimerEvent *e)
{
    // Update scene
    updateGL();
}

void GlWidget::mousePressEvent(QMouseEvent *e)
{
    // Save mouse press position
    mousePress = QVector2D(e->localPos());
}

void GlWidget::mouseReleaseEvent(QMouseEvent *e)
{
    // compute mose movement
    QVector2D mouseMove = QVector2D(e->localPos()) - mousePress;

    const float degScale = 0.1f;
    float rotDegree=0;
    QMatrix4x4 rotMatrix;
    QVector3D transVec;

    // rotation and translation operations are applied in local frame
    // .rotate(add) -> Mat_old * Mat_add
    // matrix transforms local point to global point
    // translate first and then rotate

    // ignore small movement axis
    // when draged horizontally, rotate about Z-axis
    if(fabsf(mouseMove.x()) > fabsf(mouseMove.y()))
    {
        // set rotation
        rotDegree = -mouseMove.x()*degScale;
        rotMatrix.setToIdentity();
        rotMatrix.rotate(rotDegree, 0,0,1);
        // calculate translation (local frame)
        transVec = rot_radius*(QVector3D(1,0,0) - rotMatrix*QVector3D(1,0,0));
        // commit translation and rotation
        viewPose.translate(transVec);
        viewPose.rotate(rotDegree, 0,0,1);
    }
    // when draged vertically, rotate about Y-axis
    else
    {
        // set rotation
        rotDegree = mouseMove.y()*degScale;
        rotMatrix.setToIdentity();
        rotMatrix.rotate(rotDegree, 0,1,0);
        // calculate translation (local frame)
        transVec = rot_radius*(QVector3D(1,0,0) - rotMatrix*QVector3D(1,0,0));
        // commit translation and rotation
        viewPose.translate(transVec);
        viewPose.rotate(rotDegree, 0,1,0);
    }

//    qDebug() << "updated pose" << viewPose;
}

void GlWidget::wheelEvent(QWheelEvent* e)
{
    const float moveScale = 0.0002f;
    float move = moveScale * e->delta();
    viewPose.translate(move,0,0);

//    qDebug() << "updated pose" << viewPose;
}

void GlWidget::keyPressEvent(QKeyEvent* e)
{
    const float moveScale = 0.02f;
    if(e->key() == Qt::Key_Up)
        viewPose.translate(0,0,moveScale);
    else if(e->key() == Qt::Key_Down)
        viewPose.translate(0,0,-moveScale);
    else if(e->key() == Qt::Key_Left)
        viewPose.translate(0,moveScale,0);
    else if(e->key() == Qt::Key_Right)
        viewPose.translate(0,-moveScale,0);

    QVector3D localPoint;
    localPoint = QVector3D(0,0,0);
    QVector3D eye = viewPose.map(localPoint);
    localPoint = QVector3D(1,0,0);
    QVector3D center = viewPose.map(localPoint);
    localPoint = QVector3D(0,0,1);
    QVector3D up = viewPose.map(localPoint) - eye;
//    qDebug() << "eyecenterup" << eye << center << up;
//    qDebug() << "updated pose" << viewPose;
}
