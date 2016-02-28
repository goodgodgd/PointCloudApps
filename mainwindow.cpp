#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // select DB
    dbID = eDBID::DESK1;

    // initalize instances
    pcworker = new PCWorker;

    // allocate memory for data
    pointCloud = new cl_float4[IMAGE_HEIGHT*IMAGE_WIDTH];

    // add opengl widget
    glwidget = new GlWidget();
    ui->verticalLayout->addWidget(glwidget);
    // set graphics scene
    colorScene = new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    depthScene = new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    ui->graphicsView_color->setScene(colorScene);
    ui->graphicsView_depth->setScene(depthScene);

    // allocate memory for opengl vertices
    gvm::InitVertices();
    // draw points and lines
    DrawBackground();
    // show points and lines
    gvm::SwapRW();

    // set timer
    timer = new QTimer(this);
    timer->setInterval(100);
    connect(timer, SIGNAL(timeout()), this, SLOT(RunFrame()));

    // set default UI
    ui->checkBox_wholeCloud->setChecked(true);
    ui->checkBox_color->setChecked(true);
    ui->checkBox_normal->setChecked(true);
}

MainWindow::~MainWindow()
{
    delete ui;

    delete[] pointCloud;
    delete pcworker;
}

void MainWindow::RunFrame()
{
    static QImage colorImg, depthImg;
    cv::Mat depthMat;
    g_frameIdx++;

    int viewOption = GetViewOptions();

    // read color and depth image in 320x240 size
    RgbdFileRW::ReadImage(dbID, g_frameIdx, colorImg, depthMat, depthImg);
    // convert depth to point cloud
    ConvertDepthToPoint3D(depthMat, pointCloud);

    // point cloud work
    pcworker->SetInputs(colorImg, pointCloud, viewOption);
    pcworker->Work();

    // show point cloud on the screen
    DrawBackground();
//    DrawPointCloud(pointCloud);
    gvm::SwapRW();

    // read annotation info
    vector<Annotation> annots;
//    RgbdFileRW::ReadAnnotations(dbID, g_frameIdx, annots);

    colorScene->addPixmap(QPixmap::fromImage(colorImg));
    depthScene->addPixmap(QPixmap::fromImage(depthImg));
}

void MainWindow::on_pushButton_replay_clicked()
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
    cl_float4 vnml = cl_float4{1,1,1,1};
    vnml = vnml/sqrtf(3.f);

    // point color: white
    cl_float4 vcol = cl_float4{1,1,1,1};

    // add point cloud with size of 2
#pragma omp parallel for
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        gvm::AddVertex(eVertexType::point, pointCloud[i], vcol, vnml, 2);
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
        timer->start();
    else
        timer->stop();
}

int MainWindow::GetViewOptions()
{
    int viewOption = ViewOpt::ViewNone;
    if(ui->checkBox_wholeCloud->isChecked())
    {
        viewOption |= ViewOpt::WholeCloud;
        if(ui->checkBox_color->isChecked())
            viewOption |= ViewOpt::WCColor;
        if(ui->checkBox_normal->isChecked())
            viewOption |= ViewOpt::WCNormal;
    }

    return viewOption;
}

void MainWindow::on_pushButton_resetView_clicked()
{
    glwidget->ResetView();
}
