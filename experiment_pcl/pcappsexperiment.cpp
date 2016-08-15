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
    CameraParam::camType = CameraType::OBJECT;

    timer = new QTimer(this);
    timer->setInterval(100);
    connect(timer, SIGNAL(timeout()), this, SLOT(TryFrame()));

//    TestPose();
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

        if(exception.msg.startsWith(QString("color image is not valid")))
        {
            if(ui->checkBox_every_dataset->isChecked()
                    && ui->comboBox_dataset->currentIndex() < ui->comboBox_dataset->count()-1
                    && g_frameIdx > 10)
            {
                ui->comboBox_dataset->setCurrentIndex(ui->comboBox_dataset->currentIndex()+1);
                ui->checkBox_timer->setChecked(true);
            }
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

    if(ui->comboBox_dataset->currentIndex() < DSetID::TUM_freiburg1_desk)
        CameraParam::camType = CameraType::SCENE_ICL;
    else if(ui->comboBox_dataset->currentIndex() < DSetID::Rgbd_Objects)
        CameraParam::camType = CameraType::SCENE_TUM;
}

void PCAppsExperiment::on_radioButton_data_objects_toggled(bool checked)
{
    if(checked==false)
        return;
    if(ui->radioButton_data_objects->isChecked())
        reader = CreateReader(DSetID::Rgbd_Objects);
    CameraParam::camType = CameraType::OBJECT;
}
