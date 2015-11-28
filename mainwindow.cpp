#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int a[] = {0, 1, 2, 3, 4};
    for(int b : a)
    {
        qDebug() << b;
    }

#pragma omp parallel for
    for(int i=0; i<5; i++)
    {
        qDebug() << a[i];
    }

    m_glwidget = new GlWidget();
    ui->verticalLayout->addWidget(m_glwidget);

    gvm::InitVertices();
    QVector3D vpos = QVector3D(0,0,0);
    QVector3D vcol = QVector3D(1,0,0);
    QVector3D vnml = QVector3D(1,1,1);
    for(int i=0;i<100;i++)
    {
        vpos = QVector3D((rand()%1000)/1000.f-0.5f,(rand()%1000)/1000.f-0.5f,(rand()%1000)/1000.f-0.5f);
        gvm::AddVertex(eVertexType::point, vpos, vcol, vnml, 2);
    }
    gvm::SwapRW();

    QMatrix4x4 matrix;
    matrix.translate(1,1,1);
    matrix.rotate(45, 0,0,1); //
    qDebug() << matrix;
    matrix.translate(1,2,3); // translate in local coordinate
    qDebug() << matrix;
    // => rotation and translation operations are applied in local frame
    // matrix transforms local point to global point

    QMatrix4x4 tmatrix;
    tmatrix.rotate(30, 1,0,0);
    QMatrix4x4 rmatrix;
    rmatrix = matrix*tmatrix;
    qDebug() << rmatrix;
    matrix.rotate(30, 1,0,0);
    qDebug() << matrix;
    // => rotation and translation operations are the same as right-side matrix multiplication


}

MainWindow::~MainWindow()
{
    delete ui;
}
