#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

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

void MainWindow::on_pushButton_Replay_clicked()
{
    static int frameIndex=0;
    frameIndex++;

    // read color and depth image in 320x240 size
    QString folderpath = "/home/odroid/Work/PointCloudApps/"; // TODO: set folderpath
    QImage color_img, depth_rgb, depth_gray;
//    RgbdFileRW::ReadImage(folderpath, frameIndex, color_img, depth_rgb, depth_gray);

    // read annotation info
    vector<Annotation> annots;
    RgbdFileRW::ReadAnnotations(folderpath, frameIndex, annots);


    // convert depth_rgb into QVector3D**
    // to study depth encoding in rgb channels, go to http://blog.daum.net/goodgodgd/7
    // TODO: implement this function
//    ConvertDepthToPoint3D(depth_rgb, m_pointCloud);

    // show point cloud on the screen using gvm::
    // TODO: implement this function
//    DrawPointCloud(m_pointCloud);


    QPixmap colormap("../desk_1_1.png");
    QPixmap depthmap("../desk_1_1_depth.png");
    QImage colorimg("../desk_1_1.png");
    QImage depthimg("../desk_1_1_depth.png");
    char filename[20];
    sprintf(filename, "annotation_%d.txt", frameIndex);
    QString filepath = folderpath + QString(filename);
    qDebug() << filepath;
    qDebug() << colormap.width() << colormap.height() << colormap.depth();
    qDebug() << depthmap.width() << depthmap.height() << depthmap.depth() << depthimg.width() << depthimg.height() << depthimg.depth() << depthimg.format();

    qDebug() << "frameIndex :" << frameIndex << ", # of instance :" << annots.size();
    if(annots.size()){
        for(int n=0;n<annots.size();n++){
            qDebug() <<  annots[n].name << annots[n].instance << annots[n].xl <<annots[n].xh << annots[n].yl << annots[n].yh;
        }
    }

    m_colorScene->addPixmap(colormap);
    m_depthScene->addPixmap(QPixmap::fromImage(depthimg));
}







