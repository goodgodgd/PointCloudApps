#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // select DB
    m_dbID = eDBID::DESK1;

    // initalize instances
    m_pcworker = new PCWorker;

    // allocate memory for data
    m_pointCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];

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

    // set timer
    m_timer = new QTimer(this);
    m_timer->setInterval(100);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(RunFrame()));
}

MainWindow::~MainWindow()
{
    delete ui;

    delete[] m_pointCloud;
}

void MainWindow::RunFrame()
{
    static QImage colorImg, depthImg;
    cv::Mat depthMat;
    g_frameIdx++;

    // read color and depth image in 320x240 size
    RgbdFileRW::ReadImage(m_dbID, g_frameIdx, colorImg, depthMat, depthImg);
    // convert depth to point cloud
    ConvertDepthToPoint3D(depthMat, m_pointCloud);

    // point cloud work
    m_pcworker->SetInputs(colorImg, m_pointCloud);
    m_pcworker->Work();

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

void MainWindow::on_pushButton_Replay_clicked()
{
    RunFrame();
}

void MainWindow::ConvertDepthToPoint3D(cv::Mat depthMat, cl_float4* pointCloud)
{
    const float fc = 300.f;
    const float fr = 300.f;
    const int pc = IMAGE_WIDTH/2;
    const int pr = IMAGE_HEIGHT/2;

#pragma omp parallel for
    for(int r=0; r<IMAGE_HEIGHT; r++)
    {
        for(int c=0; c<IMAGE_WIDTH; c++)
        {
            float depth = (float)depthMat.at<DepthType>(r,c) / 1000.f;
            pointCloud[r*IMAGE_WIDTH + c].x = depth;
            pointCloud[r*IMAGE_WIDTH + c].y = -(c - pc)/fc*depth;
            pointCloud[r*IMAGE_WIDTH + c].z = -(r - pr)/fr*depth;
            pointCloud[r*IMAGE_WIDTH + c].w = 1.f;

//            if(r%100==0 && c%100==0)
//                qDebug() << r << c << depth << "point" << pointCloud[r*IMAGE_WIDTH + c].x << pointCloud[r*IMAGE_WIDTH + c].y << pointCloud[r*IMAGE_WIDTH + c].z;
        }
    }
}

void MainWindow::DrawPointCloud(cl_float4* pointCloud)
{
    // point normal vector
    QVector3D vnml = QVector3D(1,1,1);
    vnml.normalize();
    // point color: white
    QVector3D vcol = QVector3D(1,1,1);

    // add point cloud with size of 2
#pragma omp parallel for
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        QVector3D vpoint;
        vpoint << pointCloud[i];
        gvm::AddVertex(eVertexType::point, vpoint, vcol, vnml, 2);
    }
}

void MainWindow::DrawBackground()
{
    /// draw example points
    QVector3D pvpos;    // point vertex position
    QVector3D vcol;     // vertex color
    // point normal vector
    QVector3D vnml = QVector3D(1,1,1);  // vertex normal
    vnml.normalize();

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

void MainWindow::on_checkBox_timer_toggled(bool checked)
{
    if(checked)
        m_timer->start();
    else
        m_timer->stop();
}
