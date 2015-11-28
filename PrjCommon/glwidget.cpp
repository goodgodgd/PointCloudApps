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

    m_viewPose.setToIdentity();
    m_viewPose.translate(m_translation);
    m_viewPose.rotate(m_rotation);
    m_projection.setToIdentity();
    m_projection.perspective(45, 4.f/3.f, 0.3f, 10.f);

    setFocusPolicy(Qt::StrongFocus);
}

void GlWidget::initializeGL()
{
    glClearColor(0,0,0,1);

    m_projection.setToIdentity();
    m_projection.perspective(45, 4.f/3.f, 0.3f, 10.f);

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

    m_program.bind();
    m_program.setUniformValue("u_mvpmat", m_projection*m_viewPose);
    m_program.setAttributeArray("a_posit", gvm::PositPtr());
    m_program.enableAttributeArray("a_posit");
    m_program.setAttributeArray("a_normal", gvm::NormalPtr());
    m_program.enableAttributeArray("a_normal");
    m_program.setAttributeArray("a_color", gvm::ColorPtr());
    m_program.enableAttributeArray("a_color");
    m_program.setAttributeArray("a_ptsize", gvm::PtsizePtr(), 1);
    m_program.enableAttributeArray("a_ptsize");

    glDrawArrays(GL_POINTS, gvm::ptbegin, gvm::ptbegin + gvm::ptnum-1);

    m_program.disableAttributeArray("a_posit");
    m_program.disableAttributeArray("a_normal");
    m_program.disableAttributeArray("a_color");
    m_program.disableAttributeArray("a_ptsize");

    m_program.release();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    painter.endNativePainting();
    painter.drawLine(100,100,100,300);
    painter.end();

    swapBuffers();
}

void GlWidget::resizeGL(int w, int h)
{
    // Set OpenGL viewport to cover whole widget
    glViewport(0, 0, w, h);
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

    const float degScale = 1.f;
    float rotScalar=0;
    QMatrix4x4 rotMatrix;
    QVector3D transVec;

    // rotation and translation operations are applied in local frame
    // .rotate(add) -> Mat_old * Mat_add
    // matrix transforms local point to global point
    // translate first and then rotate

    // ignore small movement axis
    // when draged horizontally, rotate about Z-axis
    if(mouseMove.x() > mouseMove.y())
    {
        // set rotation
        rotScalar = mouseMove.x()*degScale;
        rotMatrix.setToIdentity();
        rotMatrix.rotate(rotScalar, 0,0,1);
        // calculate translation (local frame)
        transVec = m_rot_radius*(QVector3D(1,0,0) - rotMatrix*QVector3D(1,0,0));
        // commit translation and rotation
        m_viewPose.translate(transVec);
        m_viewPose *= rotMatrix;
    }
    // when draged vertically, rotate about Y-axis
    else
    {
        // set rotation
        rotScalar = mouseMove.y()*degScale;
        rotMatrix.setToIdentity();
        rotMatrix.rotate(rotScalar, 0,1,0);
        // calculate translation (local frame)
        transVec = m_rot_radius*(QVector3D(1,0,0) - rotMatrix*QVector3D(1,0,0));
        // commit translation and rotation
        m_viewPose.translate(transVec);
        m_viewPose *= rotMatrix;
    }

    qDebug() << "pose" << m_viewPose;

}

void GlWidget::timerEvent(QTimerEvent *e)
{
    // Update scene
    updateGL();
}
