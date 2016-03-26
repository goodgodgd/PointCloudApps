#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // select DB
    dbID = eDBID::DESK1;
    colorImgPos = QPoint(12,556);
    depthImgPos = QPoint(352,556);

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
    g_frameIdx++;

    // read color and depth image in 320x240 size
    RgbdFileRW::ReadImage(dbID, g_frameIdx, colorImg, depthImg);
    // convert depth to point cloud
    ImageConverter::ConvertToPointCloud(depthImg, pointCloud);

    // point cloud work
    pcworker->Work(colorImg, pointCloud);

    // show point cloud on the screen
    UpdateView();

    // read annotation info
    vector<Annotation> annots;
//    RgbdFileRW::ReadAnnotations(dbID, g_frameIdx, annots);

    DisplayImage(colorImg, depthImg);
}

void MainWindow::DisplayImage(QImage& colorImg, QImage& depthImg)
{
    QImage depthGray;
    ImageConverter::ConvertToGrayImage(depthImg, depthGray);
    colorScene->addPixmap(QPixmap::fromImage(colorImg));
    depthScene->addPixmap(QPixmap::fromImage(depthGray));
}

void MainWindow::on_pushButton_replay_clicked()
{
    RunFrame();
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
    if(ui->radioButton_view_color->isChecked())
        viewOption |= ViewOpt::Color;
    else if(ui->radioButton_view_descriptor->isChecked())
        viewOption |= ViewOpt::Descriptor;
    else if(ui->radioButton_view_segment->isChecked())
        viewOption |= ViewOpt::Segment;
    else if(ui->radioButton_view_object->isChecked())
        viewOption |= ViewOpt::Object;
    if(ui->checkBox_normal->isChecked())
        viewOption |= ViewOpt::Normal;

    return viewOption;
}

void MainWindow::on_pushButton_resetView_clicked()
{
    glwidget->ResetView();
}

void MainWindow::on_radioButton_view_color_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void MainWindow::on_radioButton_view_descriptor_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void MainWindow::on_radioButton_view_segment_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void MainWindow::on_radioButton_view_object_toggled(bool checked)
{
    if(checked)
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
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();
}

void MainWindow::mousePressEvent(QMouseEvent* e)
{
    QPoint pixel = e->pos() - depthImgPos;
    CheckPixel(pixel);
}

void MainWindow::CheckPixel(QPoint point)
{
    if(point.x()<0 || point.x()>=IMAGE_WIDTH || point.y()<0 || point.y()>=IMAGE_HEIGHT)
        return;

    QImage depthGray;
    ImageConverter::ConvertToGrayImage(depthImg, depthGray);
    pcworker->MarkNeighborsOnImage(depthGray, point);
    QImage colorImage = colorImg;
    colorImage.setPixel(point, qRgb(255,0,0));

    int viewOption = GetViewOptions();
//    pcworker->DrawPointCloud(viewOption);
    pcworker->DrawOnlyNeighbors(point);
    pcworker->MarkPoint3D(point, viewOption);
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();

    depthScene->addPixmap(QPixmap::fromImage(depthGray));
    colorScene->addPixmap(QPixmap::fromImage(colorImage));
}
