#ifndef DESCRIPTORTESTER_H
#define DESCRIPTORTESTER_H

#include "descriptorproto.h"

class DescriptorTester : public DescriptorProto
{
public:
    DescriptorTester();
    void TestDescriptor();

private:
    int CreatePointCloud(cl_float4 trueDesc, cl_float4* pointCloud);
    void TransformPointCloud(cl_float4 rotation, cl_float4 translation, cl_float4* pointCloud, int num_pts, cl_float4& normalvt);
};

#endif // DESCRIPTORTESTER_H
