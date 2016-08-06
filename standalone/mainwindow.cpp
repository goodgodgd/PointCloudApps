#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    reader(nullptr)
{
    ui->setupUi(this);

    InitViewers();
    InitUI();
//    reader = CreateReader(ui->comboBox_dataset->currentIndex());
    pcworker = new PCWorker;

    timer = new QTimer(this);
    timer->setInterval(100);
    connect(timer, SIGNAL(timeout()), this, SLOT(TryFrame()));

    g_frameIdx=0;
}

void MainWindow::InitViewers()
{
    // add opengl widget
    glwidget = new GlWidget();
    ui->verticalLayout->addWidget(glwidget);
    // set graphics scene
    colorScene = new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    depthScene = new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    ui->graphicsView_color->setScene(colorScene);
    ui->graphicsView_depth->setScene(depthScene);
    // image view position
    colorImgPos = QPoint(12,556);
    depthImgPos = QPoint(352,556);

    // allocate memory for opengl vertices
    gvm::InitVertices();
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();
}

void MainWindow::InitUI()
{
    ui->radioButton_view_color->setChecked(true);
    ui->checkBox_normal->setChecked(true);

    ui->comboBox_dataset->addItem("ICL_room1");
    ui->comboBox_dataset->addItem("ICL_room1_ns");
    ui->comboBox_dataset->addItem("ICL_office1");
    ui->comboBox_dataset->addItem("ICL_office1_ns");
    ui->comboBox_dataset->addItem("TUM_frei1_desk");
    ui->comboBox_dataset->addItem("TUM_frei1_room");
    ui->comboBox_dataset->addItem("TUM_frei2_desk");
    ui->comboBox_dataset->addItem("TUM_frei3_long");
}

MainWindow::~MainWindow()
{
    delete ui;
    delete pcworker;
}

void MainWindow::TryFrame()
{
    try {
        RunFrame();
    }
    catch(TryFrameException exception) {
        qDebug() << "TryFrameException:" << exception.msg;
    }
}

void MainWindow::RunFrame()
{
    eltimer.start();
    // read color and depth image in 320x240 size
    reader->ReadRgbdPose(g_frameIdx+1, colorImg, depthImg, framePose);
    qDebug() << "==============================";
    qDebug() << "FRAME:" << ++g_frameIdx;

    // point cloud work
    pcworker->Work(colorImg, depthImg, framePose, &sharedData);

    // show point cloud on the screen
    UpdateView();
    qDebug() << "This frame took" << eltimer.elapsed() << "ms";
}

void MainWindow::DisplayImage(QImage colorImg, QImage depthImg)
{
    QImage depthGray;
    int viewOption = GetViewOptions();
    if(viewOption & ViewOpt::Segment)
        depthGray = DrawUtils::GetColorMap();
    else if(viewOption & ViewOpt::Object)
        depthGray = DrawUtils::GetColorMap();
    else
        ImageConverter::ConvertToGrayImage(depthImg, depthGray);

    colorScene->addPixmap(QPixmap::fromImage(colorImg));
    depthScene->addPixmap(QPixmap::fromImage(depthGray));
}

void MainWindow::on_pushButton_replay_clicked()
{
    TryFrame();
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
    if(sharedData.NullData())
        return;
    int viewOption = GetViewOptions();
    DrawUtils::DrawPointCloud(viewOption, &sharedData);
    DisplayImage(colorImg, depthImg);
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();
}

void MainWindow::mousePressEvent(QMouseEvent* e)
{
    QPoint pixel;
    pixel = e->pos() - colorImgPos;
    if(INSIDEIMG(pixel.y(), pixel.x()))
        CheckPixel(pixel);
    pixel = e->pos() - depthImgPos;
    if(INSIDEIMG(pixel.y(), pixel.x()))
        CheckPixel(pixel);
}

void MainWindow::on_pushButton_test_clicked()
{
    DoTest();
//    QImage borderImg = CheckObjectCluster(pcworker->objectClusterer, colorImg);
//    DisplayImage(borderImg, depthImg);
}

void MainWindow::on_pushButton_virtual_depth_clicked()
{
    qDebug() << "==============================";
    qDebug() << "Virtual Frame:" << ++g_frameIdx;

    VirtualRgbdSensor sensor;
    const QString shapefile = QString(PCApps_PATH) + "/IO/VirtualConfig/shapes.txt";
    const QString camerafile = QString(PCApps_PATH) + "/IO/VirtualConfig/camera.txt";
    const QString noisefile = QString(PCApps_PATH) + "/IO/VirtualConfig/noise.txt";
    sensor.MakeVirtualDepth(shapefile, camerafile, noisefile);
    sensor.GrabFrame(colorImg, depthImg);
    DisplayImage(colorImg, depthImg);

    // point cloud work
    pcworker->Work(colorImg, depthImg, framePose, &sharedData);

    // show point cloud on the screen
    UpdateView();
}

void MainWindow::CheckPixel(QPoint pixel)
{
    int viewOption = GetViewOptions();
    QImage depthGrayMarked;
    if(viewOption & ViewOpt::Segment)
        depthGrayMarked = DrawUtils::GetColorMap().copy();
    else
        ImageConverter::ConvertToGrayImage(depthImg, depthGrayMarked);
    QImage colorImgMarked = sharedData.ConstColorImage().copy();
    pcworker->MarkNeighborsOnImage(colorImgMarked, pixel);
    pcworker->MarkNeighborsOnImage(depthGrayMarked, pixel);

    pcworker->DrawOnlyNeighbors(sharedData, pixel);
    DrawUtils::MarkPoint3D(&sharedData, pixel);
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();

    depthScene->addPixmap(QPixmap::fromImage(depthGrayMarked));
    colorScene->addPixmap(QPixmap::fromImage(colorImgMarked));

    const int ptidx = IMGIDX(pixel.y(),pixel.x());
    const cl_int* segmap = sharedData.ConstObjectMap();
    QRgb color = DrawUtils::colorMap.pixel(pixel);
    qDebug() << "picked pixel" << pixel << sharedData.ConstPointCloud()[ptidx]
                << "descriptor" << sharedData.ConstDescriptors()[ptidx]
                   << "color" << qRed(color) << qGreen(color) << qBlue(color)
                      << "object" << segmap[ptidx];
}

void MainWindow::on_comboBox_dataset_currentIndexChanged(int index)
{
    if(index < DSetID::DSetEnd)
        reader = CreateReader(index);
}

RgbdReaderInterface* MainWindow::CreateReader(const int DSID)
{
    try {
        if(reader != nullptr)
            delete reader;
        reader = ReaderFactory::GetInstance(DSID);
    }
    catch(TryFrameException exception) {
        qDebug() << "ICLReaderException:" << exception.msg;
    }

    g_frameIdx=0;

    return reader;
}
