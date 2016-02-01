#include "glwidget.h"

GlWidget::GlWidget(QWidget *parent) :
    QGLWidget(parent)
{
    m_rot_default.setScalar(0.f);
    m_rot_default.setVector(0, 0, 1);
    m_trn_default = QVector3D(2, 0, 0);
    m_rotation = m_rot_default;
    m_translation = m_trn_default;
    m_rot_radius = 1.f;

    m_projection.setToIdentity();
    m_projection.perspective(45, 4.f/3.f, 0.1f, 100.f);
    m_viewPose.setToIdentity();
    m_viewPose.translate(1,0,0);
    m_viewPose.rotate(180,0,0,1);

    setFocusPolicy(Qt::StrongFocus);
}

void GlWidget::initializeGL()
{
    glClearColor(0,0,0,1);

    // link shaders
    m_program.addShaderFromSourceFile(QGLShader::Vertex, ":Resources/vertex.vsh");
    m_program.addShaderFromSourceFile(QGLShader::Fragment, ":Resources/fragment.fsh");
    m_program.link();

    // Use QBasicTimer because its faster than QTimer
    m_timer.start(30, this);
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

    // set viewMatrix from m_viewPose
    QMatrix4x4 viewMatrix;
    QVector3D localPoint;
    localPoint = QVector3D(0,0,0);
    QVector3D eye = m_viewPose.map(localPoint);
    localPoint = QVector3D(1,0,0);
    QVector3D center = m_viewPose.map(localPoint);
    localPoint = QVector3D(0,0,1);
    QVector3D up = m_viewPose.map(localPoint) - eye;
    viewMatrix.setToIdentity();
    viewMatrix.lookAt(eye,center,up);

    m_program.bind();
    m_program.enableAttributeArray("in_posit");
    m_program.enableAttributeArray("in_normal");
    m_program.enableAttributeArray("in_color");
    m_program.enableAttributeArray("in_ptsize");

    m_program.setUniformValue("in_mvpmat", m_projection * viewMatrix);
    m_program.setAttributeArray("in_posit", gvm::PositPtr());
    m_program.setAttributeArray("in_normal", gvm::NormalPtr());
    m_program.setAttributeArray("in_color", gvm::ColorPtr());
    m_program.setAttributeArray("in_ptsize", gvm::PtsizePtr(), 1);

    glDrawArrays(GL_POINTS, gvm::PtBegin(), gvm::PtNum());
    glDrawArrays(GL_LINES, gvm::LnBegin(), gvm::LnNum());
    glDrawArrays(GL_TRIANGLES, gvm::TrBegin(), gvm::TrNum());

    m_program.disableAttributeArray("in_posit");
    m_program.disableAttributeArray("in_normal");
    m_program.disableAttributeArray("in_color");
    m_program.disableAttributeArray("in_ptsize");
    m_program.release();

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
    m_mousePress = QVector2D(e->localPos());
}

void GlWidget::mouseReleaseEvent(QMouseEvent *e)
{
    // compute mose movement
    QVector2D mouseMove = QVector2D(e->localPos()) - m_mousePress;

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
        transVec = m_rot_radius*(QVector3D(1,0,0) - rotMatrix*QVector3D(1,0,0));
        // commit translation and rotation
        m_viewPose.translate(transVec);
        m_viewPose.rotate(rotDegree, 0,0,1);
    }
    // when draged vertically, rotate about Y-axis
    else
    {
        // set rotation
        rotDegree = mouseMove.y()*degScale;
        rotMatrix.setToIdentity();
        rotMatrix.rotate(rotDegree, 0,1,0);
        // calculate translation (local frame)
        transVec = m_rot_radius*(QVector3D(1,0,0) - rotMatrix*QVector3D(1,0,0));
        // commit translation and rotation
        m_viewPose.translate(transVec);
        m_viewPose.rotate(rotDegree, 0,1,0);
    }

    qDebug() << "updated pose" << m_viewPose;
}

void GlWidget::wheelEvent(QWheelEvent* e)
{
    const float moveScale = 0.01f;
    float move = -moveScale * e->delta();
    m_viewPose.translate(move,0,0);

    qDebug() << "updated pose" << m_viewPose;
}

void GlWidget::keyPressEvent(QKeyEvent* e)
{
    const float moveScale = 0.05f;
    if(e->key() == Qt::Key_Up)
        m_viewPose.translate(0,0,moveScale);
    else if(e->key() == Qt::Key_Down)
        m_viewPose.translate(0,0,-moveScale);
    else if(e->key() == Qt::Key_Left)
        m_viewPose.translate(0,moveScale,0);
    else if(e->key() == Qt::Key_Right)
        m_viewPose.translate(0,-moveScale,0);

    QVector3D localPoint;
    localPoint = QVector3D(0,0,0);
    QVector3D eye = m_viewPose.map(localPoint);
    localPoint = QVector3D(1,0,0);
    QVector3D center = m_viewPose.map(localPoint);
    localPoint = QVector3D(0,0,1);
    QVector3D up = m_viewPose.map(localPoint) - eye;
    qDebug() << "eyecenterup" << eye << center << up;
    qDebug() << "updated pose" << m_viewPose;
}
