#include "pcappsexperiment.h"
#include "ui_pcappsexperiment.h"

PCAppsExperiment::PCAppsExperiment(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCAppsExperiment),
    reader(nullptr),
    noiseGenerator(nullptr),
    indexScale(1)
{
    ui->setupUi(this);

    InitViewers();
    InitUI();

    experimenter = new Experimenter;

    timer = new QTimer(this);
    timer->setInterval(10);
    connect(timer, SIGNAL(timeout()), this, SLOT(TryFrame()));
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
//    ui->checkBox_normal->setChecked(true);

    ui->comboBox_dataset->addItem("Corbs_cabinet");
    ui->comboBox_dataset->addItem("Corbs_desk");
    ui->comboBox_dataset->addItem("Corbs_human");
    ui->comboBox_dataset->addItem("Corbs_racingcar");
    ui->comboBox_dataset->addItem("Rgbd_desk");
    ui->comboBox_dataset->addItem("Rgbd_kitchen");
    ui->comboBox_dataset->addItem("Rgbd_meeting");
    ui->comboBox_dataset->addItem("Rgbd_table");
    ui->comboBox_dataset->addItem("Rgbd_tabsml");

    dataPaths.clear();
    dataPaths << QString("/CoRBS/cabinet");
    dataPaths << QString("/CoRBS/desk");
    dataPaths << QString("/CoRBS/human");
    dataPaths << QString("/CoRBS/racingcar");
    dataPaths << QString("/rgbd-scenes/desk");
    dataPaths << QString("/rgbd-scenes/kitchen_small");
    dataPaths << QString("/rgbd-scenes/meeting_small");
    dataPaths << QString("/rgbd-scenes/table");
    dataPaths << QString("/rgbd-scenes/table_small");

    assert(ui->comboBox_dataset->count()==dataPaths.size());
    connect(ui->comboBox_dataset, SIGNAL(currentIndexChanged(int)), this, SLOT(on_comboBox_dataset_changed(int)));

    ui->comboBox_dataset->setCurrentIndex(0);
    ui->radioButton_data_scenes->setChecked(true);
//    ui->checkBox_every_dataset->setChecked(true);
//    ui->checkBox_save_descriptors->setChecked(true);
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
                && ui->comboBox_dataset->currentIndex() < ui->comboBox_dataset->count()-1
                && g_frameIdx > 5)
        {
            ui->comboBox_dataset->setCurrentIndex(ui->comboBox_dataset->currentIndex()+1);
            QThread::msleep(100);
            qDebug() << "restart" << ui->comboBox_dataset->currentIndex();
            ui->checkBox_timer->setChecked(true);
        }
    }
}

void PCAppsExperiment::RunFrame()
{
    eltimer.start();
    ReadFrame(sharedData);

    // point cloud work
    experimenter->Work(&sharedData, GetUiOptions());

    // show point cloud on the screen
    UpdateView();
    qDebug() << "This frame took" << eltimer.elapsed() << "ms";
}

void PCAppsExperiment::ReadFrame(SharedData& shdDat)
{
    if(reader==nullptr)
        throw TryFrameException("reader is not constructed yet");

    // read color and depth image
    reader->ReadRgbdPose(g_frameIdx+indexScale, colorImg, depthImg, framePose);

    // add noise if checked
    if(ui->checkBox_add_noise->isChecked())
        AddNoiseToDepth(depthImg);

    // set target pixel
    if(ui->checkBox_read_sample->isChecked())
    {
        std::vector<int> imgpt = reader->GetSamplePixel(g_frameIdx+indexScale);
        if(imgpt.size()<2 || imgpt[0]<0 || imgpt[0]>=320 || imgpt[1]<0 || imgpt[1]>=240)
            throw TryFrameException("sample pixel is not valid");
        sharedData.SetTargetPixel(imgpt);
    }

    shdDat.SetGlobalPose(framePose);
    shdDat.SetColorImage(colorImg);
    const cl_float4* pointCloud = ImageConverter::ConvertToPointCloud(depthImg);
    shdDat.SetPointCloud(pointCloud);

    g_frameIdx += indexScale;
    qDebug() << "==============================";
    qDebug() << "Current Frame:" << g_frameIdx;
}

void PCAppsExperiment::AddNoiseToDepth(QImage& depthImg)
{
    const float stdev_offset = 1.1f;
    const float stdev_slope = 1.4f;
    const float stdev_mindepth = 0.6f;
    const float depth_min_range = CameraParam::RangeBeg_m();
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            QRgb rgb = depthImg.pixel(x, y);
            uint depth = (uint)(rgb & 0xffff);
            float stdev = stdev_offset + stdev_slope*(depth/1000.f - stdev_mindepth);
            if(depth > depth_min_range)
                depth += (int)(stdev*noiseGenerator->Generate());
            if(x==100 && y==100)
                qDebug() << "add depth noise" << (uint)(rgb & 0xffff) << depth;
            QRgb rgbnoise = qRgb(0,0,0) + depth;
            depthImg.setPixel(x,y,rgbnoise);
        }
    }
}

ExpmUiOptions PCAppsExperiment::GetUiOptions()
{
    ExpmUiOptions option;
    if(ui->radioButton_data_objects->isChecked())
        option.target = ExpmTarget::OBJECT;
    else if(ui->radioButton_data_scenes->isChecked() && ui->checkBox_read_sample->isChecked())
        option.target = ExpmTarget::SAMPLE;
    else if(ui->radioButton_data_scenes->isChecked())
        option.target = ExpmTarget::SCENE;
    option.writeDesc = ui->checkBox_save_descriptors->isChecked();
    return option;
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
    ReadVirtualFrame(sharedData);
    // point cloud work
    experimenter->Work(&sharedData, GetUiOptions());
    // show point cloud on the screen
    UpdateView();
}

void PCAppsExperiment::ReadVirtualFrame(SharedData& shdDat)
{
    VirtualRgbdSensor sensor;
    const QString shapefile = QString(PCApps_PATH) + "/IO/VirtualConfig/shapes.txt";
    const QString camerafile = QString(PCApps_PATH) + "/IO/VirtualConfig/camera.txt";
    const QString noisefile = QString(PCApps_PATH) + "/IO/VirtualConfig/noise.txt";
    sensor.MakeVirtualDepth(shapefile, camerafile, noisefile);
    sensor.GrabFrame(colorImg, depthImg);

    shdDat.SetGlobalPose(framePose);
    shdDat.SetColorImage(colorImg);
    const cl_float4* pointCloud = ImageConverter::ConvertToPointCloud(depthImg);
    shdDat.SetPointCloud(pointCloud);

    qDebug() << "==============================";
    qDebug() << "Virtual Frame:" << ++g_frameIdx;
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

void PCAppsExperiment::on_comboBox_dataset_changed(int index)
{
    CreateSceneReader();
}

void PCAppsExperiment::on_radioButton_data_scenes_toggled(bool checked)
{
    CreateSceneReader();
}

void PCAppsExperiment::on_radioButton_data_objects_toggled(bool checked)
{
    if(checked==false)
        return;
    reader = new ObjectReader;
    CameraParam::SetCameraType(CameraType::OBJECT);
}

void PCAppsExperiment::CreateSceneReader()
{
    const int dataIndex = ui->comboBox_dataset->currentIndex();
    if(!ui->radioButton_data_scenes->isChecked() || dataIndex>=dataPaths.size())
        return;
    if(reader!=nullptr)
        delete reader;

    try {
        reader = ReaderFactory::GetInstance(dataPaths[dataIndex], ui->checkBox_read_sample->isChecked());
    }
    catch(TryFrameException exception) {
        qDebug() << "CreateReaderException:" << exception.msg;
    }

    indexScale = smax(reader->GetLength()/500, 1);
    CameraParam::SetCameraType(GetCameraType(dataIndex));
    g_frameIdx=0;
}

CameraType PCAppsExperiment::GetCameraType(const int dataIndex)
{
    if(dataPaths[dataIndex].contains("CoRBS"))
        return CameraType::SCENE_CoRBS;
    else if(dataPaths[dataIndex].contains("rgbd-scenes"))
        return CameraType::SCENE_Washington;
    return CameraType::SCENE_CoRBS;
}

void PCAppsExperiment::mousePressEvent(QMouseEvent* e)
{
    static QPoint colorImgPos = QPoint(12,556);
    static QPoint depthImgPos = QPoint(352,556);

    QPoint pixel = e->pos() - depthImgPos;
    const cl_float4* pointCloud = sharedData.ConstPointCloud();
    const cl_float4* normalCloud = sharedData.ConstNormalCloud();
    const int pxidx = IMGIDX(pixel.y(), pixel.x());
    if(INSIDEIMG(pixel.y(), pixel.x()))
        qDebug() << "clicked" << pixel << pointCloud[pxidx] << normalCloud[pxidx];
}

void PCAppsExperiment::on_checkBox_add_noise_toggled(bool checked)
{
    if(!checked) return;
    if(noiseGenerator!=nullptr)
        delete noiseGenerator;
    const QString noisefile = QString(PCApps_PATH) + "/IO/VirtualConfig/noise.txt";
    noiseGenerator = NoiseReader::ReadNoiseGenerator(noisefile);
}

void PCAppsExperiment::on_checkBox_read_sample_toggled(bool checked)
{
    CreateSceneReader();
}
