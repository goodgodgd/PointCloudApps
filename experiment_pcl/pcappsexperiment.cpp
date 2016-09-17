#include "pcappsexperiment.h"
#include "ui_pcappsexperiment.h"

PCAppsExperiment::PCAppsExperiment(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCAppsExperiment),
    reader(nullptr)
{
    ui->setupUi(this);

    InitViewers();
    InitUI();
//    reader = CreateReader(ui->comboBox_dataset->currentIndex());
    experimenter = new Experimenter;
    CameraParam::dsetType = DSetID::Rgbd_Objects;

    timer = new QTimer(this);
    timer->setInterval(10);
    connect(timer, SIGNAL(timeout()), this, SLOT(TryFrame()));

//    TestPose();
//    TestRotAxes();
    TestPoseTri();
    g_frameIdx=0;
}

PCAppsExperiment::~PCAppsExperiment()
{
    delete ui;
}

void PCAppsExperiment::InitViewers()
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

void PCAppsExperiment::InitUI()
{
    ui->radioButton_view_color->setChecked(true);
    ui->checkBox_normal->setChecked(true);

    ui->comboBox_dataset->addItem("Corbs_cabinet");
    ui->comboBox_dataset->addItem("Corbs_desk");
    ui->comboBox_dataset->addItem("Corbs_human");
    ui->comboBox_dataset->addItem("ICL_room1");
    ui->comboBox_dataset->addItem("ICL_room1_ns");
    ui->comboBox_dataset->addItem("ICL_office1");
    ui->comboBox_dataset->addItem("ICL_office1_ns");
    ui->comboBox_dataset->addItem("TUM_frei1_desk");
    ui->comboBox_dataset->addItem("TUM_frei1_room");
    ui->comboBox_dataset->addItem("TUM_frei2_desk");
    ui->comboBox_dataset->addItem("TUM_frei3_long");
}

void PCAppsExperiment::TryFrame()
{
    try {
        RunFrame();
    }
    catch(TryFrameException exception) {
        qDebug() << "TryFrameException:" << exception.msg;

        ui->checkBox_timer->setChecked(false);

        if(exception.msg.startsWith(QString("dataset finished"))
                && ui->checkBox_every_dataset->isChecked()
                && ui->comboBox_dataset->currentIndex() < DSetID::Corbs_human
                && g_frameIdx > 5)
        {
            ui->comboBox_dataset->setCurrentIndex(ui->comboBox_dataset->currentIndex()+1);
            QThread::msleep(100);
            qDebug() << "restart" << ui->comboBox_dataset->currentIndex();
            ui->checkBox_timer->setChecked(true);
        }
        else if(exception.msg.startsWith(QString("invalid object")))
        {
            reader->ChangeInstance();
            ui->checkBox_timer->setChecked(true);
        }
    }
}

void PCAppsExperiment::RunFrame()
{
    if(reader==nullptr)
        throw TryFrameException("reader is not constructed yet");
    if(g_frameIdx >= 100 && ui->radioButton_data_scenes->isChecked())
        throw TryFrameException("dataset finished");
    // read color and depth image in 320x240 size
    reader->ReadRgbdPose(g_frameIdx+1, colorImg, depthImg, framePose);
    qDebug() << "==============================";
    qDebug() << "FRAME:" << ++g_frameIdx;

    // point cloud work
    experimenter->Work(colorImg, depthImg, framePose, &sharedData, ui->radioButton_data_objects->isChecked());

    // show point cloud on the screen
    UpdateView();
}

void PCAppsExperiment::DisplayImage(QImage colorImg, QImage depthImg)
{
    QImage depthGray;
    ImageConverter::ConvertToGrayImage(depthImg, depthGray);

    colorScene->addPixmap(QPixmap::fromImage(colorImg));
    depthScene->addPixmap(QPixmap::fromImage(depthGray));
}

int PCAppsExperiment::GetViewOptions()
{
    int viewOption = ViewOpt::ViewNone;
    if(ui->radioButton_view_color->isChecked())
        viewOption |= ViewOpt::Color;
    else if(ui->radioButton_view_our_curvature->isChecked())
        viewOption |= ViewOpt::CURVATURE;
    else if(ui->radioButton_view_fpfh->isChecked())
        viewOption |= ViewOpt::FPFH;
    else if(ui->radioButton_view_shot->isChecked())
        viewOption |= ViewOpt::SHOT;
    if(ui->checkBox_normal->isChecked())
        viewOption |= ViewOpt::Normal;

    return viewOption;
}

void PCAppsExperiment::on_pushButton_virtual_depth_clicked()
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
    experimenter->Work(colorImg, depthImg, framePose, &sharedData);

    // show point cloud on the screen
    UpdateView();
}

void PCAppsExperiment::on_pushButton_resetView_clicked()
{
    glwidget->ResetView();
}

void PCAppsExperiment::on_radioButton_view_color_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::on_radioButton_view_our_curvature_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::on_radioButton_view_fpfh_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::on_radioButton_view_shot_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::on_checkBox_normal_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::UpdateView()
{
    if(sharedData.NullData())
        return;
    int viewOption = GetViewOptions();
    const QImage colorImg = sharedData.ConstColorImage();
    DrawUtils::DrawPointCloud(viewOption, &sharedData);
    DisplayImage(colorImg, depthImg);
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();
}

void PCAppsExperiment::on_pushButton_replay_clicked()
{
    TryFrame();
}

void PCAppsExperiment::on_checkBox_timer_toggled(bool checked)
{
    if(checked)
        timer->start();
    else
        timer->stop();
}

void PCAppsExperiment::on_comboBox_dataset_currentIndexChanged(int index)
{
    if(index < DSetID::Rgbd_Objects && ui->radioButton_data_scenes->isChecked())
        reader = CreateReader(index);
    CameraParam::dsetType = ui->comboBox_dataset->currentIndex();
    qDebug() << "index change";
}

RgbdReaderInterface* PCAppsExperiment::CreateReader(const int DSID)
{
    try {
        if(reader != nullptr)
            delete reader;
        reader = ExpmReaderFactory::GetInstance(DSID);
    }
    catch(TryFrameException exception) {
        qDebug() << "CreateReaderException:" << exception.msg;
    }

    g_frameIdx=0;

    return reader;
}

void PCAppsExperiment::on_radioButton_data_scenes_toggled(bool checked)
{
    if(checked==false)
        return;
    if(ui->comboBox_dataset->currentIndex() < DSetID::Rgbd_Objects)
        reader = CreateReader(ui->comboBox_dataset->currentIndex());
    CameraParam::dsetType = ui->comboBox_dataset->currentIndex();
    qDebug() << "dsetType" << CameraParam::dsetType << ui->comboBox_dataset->currentIndex() << DSetID::ICL_NUIM_room1;
}

void PCAppsExperiment::on_radioButton_data_objects_toggled(bool checked)
{
    if(checked==false)
        return;
    reader = CreateReader(DSetID::Rgbd_Objects);
    CameraParam::dsetType = DSetID::Rgbd_Objects;
}

void PCAppsExperiment::mousePressEvent(QMouseEvent* e)
{
    static QPoint colorImgPos = QPoint(12,556);
    static QPoint depthImgPos = QPoint(352,556);

    QPoint pixel = e->pos() - colorImgPos;
    const cl_float4* pointCloud = sharedData.ConstPointCloud();
    const cl_float4 selPoint = pointCloud[IMGIDX(pixel.y(), pixel.x())];
    Pose6dof curPose = sharedData.ConstGlobalPose();
    cl_float4 glbPoint = curPose.Local2Global(selPoint);
    if(INSIDEIMG(pixel.y(), pixel.x()))
        qDebug() << "clicked" << pixel << selPoint << glbPoint;
}
