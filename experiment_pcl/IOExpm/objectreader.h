#ifndef OBJECTREADER_H
#define OBJECTREADER_H

#include <QStringList>
#include <QDir>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "IO/FileReaders/rgbdreaderinterface.h"
#include "IO/imageconverter.h"

struct PointXYZRGBIM
{
  union
  {
    struct
    {
      float x;
      float y;
      float z;
      float rgb;
      float imX;
      float imY;
    };
    float data[6];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBIM,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, rgb, rgb)
                                    (float, imX, imX)
                                    (float, imY, imY)
)

union FloatRGB
{
    float data;
    uchar rgb[4];
};


typedef PointXYZRGBIM               ObjPoint;
typedef pcl::PointCloud<ObjPoint>   ObjPointCloud;

class ObjectReader : public RgbdReaderInterface
{
public:
    ObjectReader();
    virtual ~ObjectReader() {}
    virtual void ReadRgbdPose(const int index, QImage& color, QImage& depth, Pose6dof& pose);
    virtual void ChangeInstance();

    static constexpr char* dsroot = "/media/hyukdoo/Edrive/PaperData/rgbd-object-dataset";
    static QString objectID;

protected:
    QString GetCategoryPath();
    QString GetInstancePath();
    QStringList ListSubPaths(QString parentPath);
    std::vector<QStringList> ListVideoFrames();
    QStringList GetVideoFrameNames(const int videoIdx);

    void UpdateIndices();
    QString PcdFilePath();
    QString ParseObjectID(QString filePath);
    ObjPointCloud::Ptr ReadPointCloud(QString filePath);
    void ExtractRgbDepth(ObjPointCloud::Ptr pointCloud, QImage& colorImgOut, QImage& depthImgOut);

    QStringList categoryNames;
    QStringList instanceNames;
    std::vector<QStringList> videoFrames;

    int categoryIndex;
    int instanceIndex;
    int videoIndex;
    int frameIndex;
    const int videosUpto = 3+1;
    const int framesUpto = 5+1;
};

#endif // OBJECTREADER_H
