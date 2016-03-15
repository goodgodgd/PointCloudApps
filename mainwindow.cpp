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
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();

    // set timer
    timer = new QTimer(this);
    timer->setInterval(100);
    connect(timer, SIGNAL(timeout()), this, SLOT(RunFrame()));

    // set default UI
    ui->checkBox_wholeCloud->setChecked(true);
    ui->radioButton_view_color->setChecked(true);
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
    gvm::ShowAddedVertices();
    gvm::AddCartesianAxes();

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
    const cl_float4 point0 = (cl_float4){0,0,0,0};

#pragma omp parallel for
    for(int r=0; r<IMAGE_HEIGHT; r++)
    {
        for(int c=0; c<IMAGE_WIDTH; c++)
        {
            float depth = (float)depthMat.at<DepthType>(r,c) / 1000.f;
            if(depth < DEAD_RANGE_M)
            {
                pointCloud[r*IMAGE_WIDTH + c] = point0;
                continue;
            }

            pointCloud[r*IMAGE_WIDTH + c].x = depth;
            pointCloud[r*IMAGE_WIDTH + c].y = -(c - pc)/fc*depth;
            pointCloud[r*IMAGE_WIDTH + c].z = -(r - pr)/fr*depth;
            pointCloud[r*IMAGE_WIDTH + c].w = 0.f;
        }
    }
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
        if(ui->radioButton_view_color->isChecked())
            viewOption |= ViewOpt::WCColor;
        else if(ui->radioButton_view_descriptor->isChecked())
            viewOption |= ViewOpt::WCDescriptor;
        else if(ui->radioButton_view_segment->isChecked())
            viewOption |= ViewOpt::WCSegment;
        else if(ui->radioButton_view_object->isChecked())
            viewOption |= ViewOpt::WCObject;
        if(ui->checkBox_normal->isChecked())
            viewOption |= ViewOpt::Normal;
    }

    return viewOption;
}

void MainWindow::on_pushButton_resetView_clicked()
{
    glwidget->ResetView();
}

void MainWindow::on_radioButton_view_color_toggled(bool checked)
{
    UpdateView();
}

void MainWindow::on_radioButton_view_descriptor_toggled(bool checked)
{
    UpdateView();
}

void MainWindow::on_radioButton_view_segment_toggled(bool checked)
{
    UpdateView();
}

void MainWindow::on_radioButton_view_object_toggled(bool checked)
{
    UpdateView();
}

void MainWindow::on_checkBox_normal_toggled(bool checked)
{
    UpdateView();
}

void MainWindow::UpdateView()
{
    int viewOption = GetViewOptions();
    pcworker->DrawPointCloud(viewOption);
    gvm::ShowAddedVertices();
    gvm::AddCartesianAxes();
}
