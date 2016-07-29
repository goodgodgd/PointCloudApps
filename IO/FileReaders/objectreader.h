#ifndef OBJECTREADER_H
#define OBJECTREADER_H

#include <QStringList>
#include <QDir>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "rgbdreaderinterface.h"
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

    static QStringList categoryDirs;
    static int categoryIndex;
    static int instanceIndex;
    static int videoIndex;
    static int frameIndex;

protected:
    QStringList ListSubPaths(QString parentPath);
    void UpdateIndices();
    ObjPointCloud::Ptr ReadPointCloud(QString fileName);
    QString PCDName();
    void ExtractRgbDepth(ObjPointCloud::Ptr pointCloud, QImage& colorImgOut, QImage& depthImgOut);

    static constexpr char* dsroot = "/media/hyukdoo/Edrive/PaperData/rgbd-object-dataset";
    QStringList categoryNames;
    QStringList instanceDirs;
    const int videosUpto = 3;
    const int frameBegin = 1;
    const int frameEnd = 30;
};

#endif // OBJECTREADER_H
