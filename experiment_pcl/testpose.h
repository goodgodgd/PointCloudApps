#ifndef TESTPOSE_H
#define TESTPOSE_H

#include "Share/pose6dof.h"

inline void TestPose()
{
    Eigen::Vector3f pos;
    Eigen::Quaternionf quat;
    Pose6dof pose1, pose2;

//    quat = Eigen::AngleAxisf(PI_F/6.f, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(PI_F/6.f, Eigen::Vector3f::UnitY());
//    pos = Eigen::Vector3f(1,2,3);
    quat = Eigen::AngleAxisf(PI_F/6.f, Eigen::Vector3f::UnitZ());
    pos = Eigen::Vector3f(3,4,0);
    pose1.SetPos(pos);
    pose1.SetQuat(quat);

//    quat = Eigen::AngleAxisf(PI_F/3.f, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(PI_F/6.f, Eigen::Vector3f::UnitZ());
//    pos = Eigen::Vector3f(3,2,1);
    quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
    pos = Eigen::Vector3f(1,2,0);
    pose2.SetPos(pos);
    pose2.SetQuat(quat);

    qDebug() << "poses" << pose1 << pose2;

    cl_float4 glbPoint = (cl_float4){5,6,7,0};
    cl_float4 locPoint, recPoint;
    locPoint = pose1.Global2Local(glbPoint);
    recPoint = pose1.Local2Global(locPoint);
    qDebug() << "pose1" << glbPoint << locPoint << recPoint;

    locPoint = pose2.Global2Local(glbPoint);
    recPoint = pose2.Local2Global(locPoint);
    qDebug() << "pose2" << glbPoint << locPoint << recPoint;

    cl_float4 lpoint1, lpoint2, gpoint;
    lpoint1 = pose1.Global2Local(glbPoint);
    lpoint2 = pose2.Global2Local(glbPoint);
    Pose6dof relPose = pose1 / pose2;
    gpoint = relPose.Local2Global(lpoint1);
    qDebug() << "tri" << relPose << lpoint1 << lpoint2 << gpoint;

    cl_float4 srcdir = (cl_float4){0,1,0,0};
    cl_float4 dstdir = pose1.Rotate2Global(srcdir);
    qDebug() << "local" << srcdir << "global" << dstdir;
}

#endif // TESTPOSE_H
