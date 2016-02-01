#include "rgbdfilerw.h"
//#include "png.h"
RgbdFileRW::RgbdFileRW()
{
}

void RgbdFileRW::ReadImage(QString folderpath, const int index, QImage& color, QImage& depth_rgb, QImage& depth_gray)
{
    // create file name
    char file[10];
    sprintf(file, "desk_1_%d.png", index);
    QString filepath = folderpath + QString(file);

    /*png_struct *png;
    png_info *info;
    char depthfile[20];
    sprintf(depthfile, "desk_depth_1_%d.png", index);
    QString depthpath = folderpath + QString(depthfile);
    //QFile fp(depthpath);
    FILE *fp = fopen(depthfile, "rb");

    png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    info = png_create_info_struct(png);
    setjmp(png_jmpbuf(png));
    png_init_io(png, fp);
    png_read_info(png, info);

    int width      = png_get_image_width(png, info);
    //int height = png_get_image_height(png, info);
    png_byte color_type = png_get_color_type(png, info);
    png_byte bit_depth  = png_get_bit_depth(png, info);
    if(bit_depth == 16)
       png_set_strip_16(png);
    png_bytep *row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * IMAGE_HEIGHT);

    for(int y = 0; y < IMAGE_HEIGHT; y++) {
        row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
      }

      png_read_image(png, row_pointers);
      fclose(fp);
      */


    // To study QImage, go to http://doc.qt.io/qt-5/qimage.html#load

    // load color image in QImage::Format_RGB32
    color = QImage(filepath);
    // TODO: resize image into 320x240
//    color = ??
    color = color.scaled(IMAGE_WIDTH,IMAGE_HEIGHT,Qt::KeepAspectRatio);

    depth_rgb = QImage(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);


    // load depth image in QImage::Format_RGB32
    int depth;
    QRgb rgb;
    uchar gray;
    float maxrange = 3000.f;    // maximum distance 3000mm
    for(int y=0; y<IMAGE_HEIGHT; y++)
    {
        for(int x=0; x<IMAGE_WIDTH; x++)
        {
            // TODO: load depth image anyway and read depth at (x,y) in mm unit
            // depth = ???(x,y)

            // set depth on RGB channels
            rgb = depth;
            depth_rgb.setPixel(x, y, rgb);
            // set gray scale depth
            gray = (uchar)(depth / maxrange * 255.f);
            rgb = qRgb(gray, gray, gray);
            depth_gray.setPixel(x, y, rgb);
        }
    }

}

void RgbdFileRW::WriteImage(QString folderpath, const int index, QImage& color, QImage& depth_rgb)
{

}

void RgbdFileRW::ReadAnnotations(QString folderpath, const int index, vector<Annotation>& annots)
{
    // TODO: fill in this function to read annotation info at frame of index

    char filename[20];
    sprintf(filename, "annotation_%d.txt", index);
    QString filepath = folderpath + QString(filename);
    //QFile file("/home/odroid/Work/PointCloudApps/annotation_1.txt");
    QFile file(filepath);
    int xl, xh, yl, yh, instance;
    char name[20];
    file.open(QIODevice::ReadOnly|QIODevice::Text);
    QTextStream in(&file);

    QString line = in.readLine();
    int number = line.toInt();
    if(number){
        for(int i=0; i<number; i++){
            QString Annotation_data = in.readLine();
            QStringList tmpList = Annotation_data.split(",");
            sprintf(name,"%s",tmpList.value(0).toStdString().data());
            instance = tmpList.value(1).toInt();
            yl = tmpList.value(2).toInt();
            yh = tmpList.value(3).toInt();
            xl = tmpList.value(4).toInt();
            xh = tmpList.value(5).toInt();
            annots.emplace_back(name, instance, xl, xh, yl, yh);
        }

    }
    file.flush();
    in.flush();
    file.close();
    // ....

    

}
