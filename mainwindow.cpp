#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // select DB
    m_dbID = eDBID::DESK1;

    // allocate memory for data
    m_pointCloud = new QVector3D*[IMAGE_HEIGHT];
    for(int y=0; y<IMAGE_HEIGHT; y++)
        m_pointCloud[y] = new QVector3D[IMAGE_WIDTH];

    // add opengl widget
    m_glwidget = new GlWidget();
    ui->verticalLayout->addWidget(m_glwidget);
    // set graphics scene
    m_colorScene = new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    m_depthScene = new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    ui->graphicsView_Color->setScene(m_colorScene);
    ui->graphicsView_Depth->setScene(m_depthScene);

    // allocate memory for opengl vertices
    gvm::InitVertices();
    // draw points and lines
    DrawBackground();
    // show points and lines
    gvm::SwapRW();

    return;

    /// Test codes
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

    for(int y=0; y<IMAGE_HEIGHT; y++)
        delete[] m_pointCloud[y];
    delete[] m_pointCloud;

}

void MainWindow::on_pushButton_Replay_clicked()
{
    static QImage colorImg, depthImg;
    cv::Mat depthMat;
    g_frameIdx++;

    // read color and depth image in 320x240 size
    RgbdFileRW::ReadImage(m_dbID, g_frameIdx, colorImg, depthMat, depthImg);

    ConvertDepthToPoint3D(depthMat, m_pointCloud);

    // show point cloud on the screen
    DrawBackground();
    DrawPointCloud(m_pointCloud);
    gvm::SwapRW();

    // read annotation info
    vector<Annotation> annots;
//    RgbdFileRW::ReadAnnotations(m_dbID, g_frameIdx, annots);


    m_colorScene->addPixmap(QPixmap::fromImage(colorImg));
    m_depthScene->addPixmap(QPixmap::fromImage(depthImg));
}

void MainWindow::ConvertDepthToPoint3D(cv::Mat depthMat, QVector3D** pointCloud)
{
    const float fc = 300.f;
    const float fr = 300.f;
    const int pc = IMAGE_WIDTH/2;
    const int pr = IMAGE_HEIGHT/2;

    float depth;
    for(int r=0; r<IMAGE_HEIGHT; r++)
    {
        for(int c=0; c<IMAGE_WIDTH; c++)
        {
            depth = (float)depthMat.at<DepthType>(r,c) / 1000.f;
            pointCloud[r][c].setX(depth);
            pointCloud[r][c].setY(-(c - pc)/fc*depth);
            pointCloud[r][c].setZ(-(r - pr)/fr*depth);

//            if(r%50==0 && c%50==0)
//                qDebug() << r << c << depth << pointCloud[r][c];
        }
    }
}

void MainWindow::DrawPointCloud(QVector3D** pointCloud)
{
    // point normal vector
    QVector3D vnml = QVector3D(1,1,1);
    vnml.normalize();
    // point color: white
    QVector3D vcol = QVector3D(1,1,1);

    // add point cloud with size of 2
    for(int y=0; y<IMAGE_HEIGHT; y++)
        for(int x=0; x<IMAGE_WIDTH; x++)
            gvm::AddVertex(eVertexType::point, pointCloud[y][x], vcol, vnml, 2);
}

void MainWindow::DrawBackground()
{
    /// draw example points
    QVector3D pvpos;    // point vertex position
    QVector3D vcol;     // vertex color
    // point normal vector
    QVector3D vnml = QVector3D(1,1,1);  // vertex normal
    vnml.normalize();

    // set point position and color
    pvpos = QVector3D(0.2f,0.2f,0);
    vcol = QVector3D(1,0,0);    // red
    // add red point at (0.2f, 0.2f, 0) with size of 2
    gvm::AddVertex(eVertexType::point, pvpos, vcol, vnml, 2);
    // set point position and color
    pvpos = QVector3D(0,0.2f,0.2f);
    vcol = QVector3D(0,1,0);    // green
    // add green point at pvpos with size of 2
    gvm::AddVertex(eVertexType::point, pvpos, vcol, vnml, 2);
    // set point position and color
    pvpos = QVector3D(0.2f,0,0.2f);
    vcol = QVector3D(0,0,1);    // blue
    // add blue point at pvpos with size of 2
    gvm::AddVertex(eVertexType::point, pvpos, vcol, vnml, 2);


    /// draw coordinate axes
    QVector3D lnpos1;   // line end point1 vertex position at origin
    QVector3D lnpos2;   // line end point2 vertex position

    // draw X-axis with red line
    vcol = QVector3D(1,0,0);    // red
    // add line vertex 1 at origin
    lnpos1 = QVector3D(0,0,0);
    gvm::AddVertex(eVertexType::line, lnpos1, vcol, vnml, 1);
    // add line vertex 2 along X-axis
    lnpos2 = QVector3D(0.2f,0,0);
    gvm::AddVertex(eVertexType::line, lnpos2, vcol, vnml, 1, true); // when last vertex of line is added, last argument must be "true"

    // draw Y-axis with green line
    vcol = QVector3D(0,1,0);    // green
    // add line vertex 1 at origin
    lnpos1 = QVector3D(0,0,0);
    gvm::AddVertex(eVertexType::line, lnpos1, vcol, vnml, 1);
    // add line vertex 2 along Y-axis
    lnpos2 = QVector3D(0,0.2f,0);
    gvm::AddVertex(eVertexType::line, lnpos2, vcol, vnml, 1, true); // when last vertex of line is added, last argument must be "true"

    // draw Z-axis with blue line
    vcol = QVector3D(0,0,1);    // blue
    // add line vertex 1 at origin
    lnpos1 = QVector3D(0,0,0);
    gvm::AddVertex(eVertexType::line, lnpos1, vcol, vnml, 1);
    // add line vertex 2 along Z-axis
    lnpos2 = QVector3D(0,0,0.2f);
    gvm::AddVertex(eVertexType::line, lnpos2, vcol, vnml, 1, true); // when last vertex of line is added, last argument must be "true"

}






