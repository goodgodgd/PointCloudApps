#ifndef POSE6DOF_H
#define POSE6DOF_H

#include <iostream>
#include <Eigen/Eigen>
#include "Share/project_common.h"
#include "ClUtils/cloperators.h"

#define POSEDIM   7

// x~q3까지 한번에 복사하기 위해 union으로 선언
union Pose6dof{
    // strucured pose
    struct {
        float x, y, z;
        float q0, q1, q2, q3;
    };
    // array data
    float data[8];

    Pose6dof(void)
    {
        SetZeroP6D();
    }

    void NormalizeAngle()
    {
        // convert to angle axis
        Eigen::AngleAxisf anax(GetQuat());
        // normalize angle
        float angle = atan2(sin(anax.angle()), cos(anax.angle()));
        // set quaternion
        anax = Eigen::AngleAxisf(angle, anax.axis());
        Eigen::Quaternionf quat(anax);
        SetQuat(quat);
    }

    void SetZeroP6D(void)
    {
        memset(data, 0x00, sizeof(data));
        q0 = 1.0f;
    }

    void Invert(void)
    {
        Eigen::Matrix3f rot = GetRotation();
        Eigen::Quaternionf quat(rot.transpose());
        Eigen::Vector3f posi = -rot.transpose()*GetPos();
        SetPose6Dof(posi, quat);
    }

    Pose6dof Inverse(void) const
    {
        Pose6dof invpose;
        Eigen::Matrix3f rot = GetRotation();
        Eigen::Quaternionf quat(rot.transpose());
        Eigen::Vector3f posi = -rot.transpose()*GetPos();
        invpose.SetPose6Dof(posi, quat);
        return invpose;
    }

    void SetPose6Dof(Eigen::Affine3f& affn)
    {
        Eigen::Translation3f trns;
        Eigen::Quaternionf quat;
        // translation 추출
        trns = Eigen::Translation3f(affn.translation());
        x = trns.x();
        y = trns.y();
        z = trns.z();
        // quaternion 추출
        quat = Eigen::Quaternionf(affn.rotation());
        q0 = quat.w();
        q1 = quat.x();
        q2 = quat.y();
        q3 = quat.z();
    }

    // VectorXf로 들어온 값을 그대로 복사
    void SetPose6Dof(Eigen::VectorXf state)
    {
        for(int i=0;i<POSEDIM;i++)
            data[i] = state(i);
    }

    // Vector3f와 Quaternionf로 들어온 값을 그대로 복사
    void SetPose6Dof(Eigen::Vector3f& pos, Eigen::Quaternionf& quat)
    {
        x = pos(0);
        y = pos(1);
        z = pos(2);
        q0 = quat.w();
        q1 = quat.x();
        q2 = quat.y();
        q3 = quat.z();
    }

    void SetQuat(Eigen::Quaternionf& quat)
    {
        q0 = quat.w();
        q1 = quat.x();
        q2 = quat.y();
        q3 = quat.z();
    }

    void SetPos(Eigen::Vector3f& pos)
    {
        x = pos(0);
        y = pos(1);
        z = pos(2);
    }

    // Affine3f로 변환
    Eigen::Affine3f GetAffine(void) const
    {
        Eigen::Affine3f affn;
        Eigen::Translation3f trns = Eigen::Translation3f(x, y, z);
        Eigen::AngleAxisf anax = Eigen::AngleAxisf(Eigen::Quaternionf(q0, q1, q2, q3));
        affn = trns * anax;
        return affn;
    }

    Eigen::VectorXf GetVector() const
    {
        Eigen::VectorXf vec(POSEDIM);
        for(int i=0;i<POSEDIM;i++)
            vec(i) = data[i];
        return vec;
    }

    Eigen::Vector3f GetPos() const
    {
        return Eigen::Vector3f(x, y, z);
    }

    Eigen::Quaternionf GetQuat() const
    {
        return Eigen::Quaternionf(q0, q1, q2, q3);
    }

    float GetAngle() const
    {
        Eigen::AngleAxisf anax(GetQuat());
        return anax.angle();
    }

    cl_float4 Local2Global(const cl_float4& lpoint) const
    {
        Eigen::Vector3f elpoint(lpoint.x, lpoint.y, lpoint.z);
        Eigen::Vector3f egpoint = GetAffine() * elpoint;
        cl_float4 gpoint = (cl_float4){egpoint(0), egpoint(1), egpoint(2), 0};
        return gpoint;
    }

    cl_float4 Rotate2Global(const cl_float4& ldir) const
    {
        Eigen::Vector3f eldir(ldir.x, ldir.y, ldir.z);
        Eigen::Vector3f egdir = GetRotation() * eldir;
        cl_float4 gdir = (cl_float4){egdir(0), egdir(1), egdir(2), 0.f};
        return gdir;
    }

    cl_float4 Global2Local(const cl_float4& gpoint) const
    {
        Eigen::Vector3f egpoint(gpoint.x, gpoint.y, gpoint.z);
        Pose6dof invPose = this->Inverse();
        Eigen::Vector3f elpoint = invPose.GetAffine() * egpoint;
        cl_float4 lpoint = (cl_float4){elpoint(0), elpoint(1), elpoint(2), 0};
        return lpoint;
    }

    Eigen::Matrix3f GetRotation() const
    {
        Eigen::Quaternionf quat(q0, q1, q2, q3);
        Eigen::Matrix3f rotation = quat.matrix();
        return rotation;
    }

    Pose6dof& operator=(const Pose6dof& srcpose)
    {
        if(this != &srcpose)
            memcpy(this->data, srcpose.data, sizeof(this->data));
        return *this;
    }

    // T_2_in_global = T_1_in_global * T_2_in_T_1_local
    // dstPose = this * localPose
    Pose6dof operator*(Pose6dof& localPose)
    {
        Eigen::Vector3f dstPosit = this->GetRotation()*localPose.GetPos() + this->GetPos();
        Eigen::Quaternionf dstQuat = this->GetQuat() * localPose.GetQuat();
        // quaternion nomalization
        Eigen::AngleAxisf anax(dstQuat);
        dstQuat = anax;

        Pose6dof dstPose;
        dstPose.SetPose6Dof(dstPosit, dstQuat);
        return dstPose;
    }

    // T_1_in_global^-1 * T_2_in_global = T_2_in_T_1_local
    // T_1_in_global / T_2_in_global = T_2_in_T_1_local
    // this / globalPose = dstPose
    Pose6dof operator/(Pose6dof& globalPose)
    {
        Eigen::Vector3f dstPosit = this->GetRotation().transpose()*(globalPose.GetPos() - this->GetPos());
        Eigen::Quaternionf dstQuat = this->GetQuat().conjugate() * globalPose.GetQuat();
        // quaternion nomalization
        Eigen::AngleAxisf anax(dstQuat);
        dstQuat = anax;

        Pose6dof dstPose;
        dstPose.SetPose6Dof(dstPosit, dstQuat);
        return dstPose;
    }
};

inline QDebug operator <<(QDebug debug, const Pose6dof &pose)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "pose(" << pose.x << ", " << pose.y << ", " << pose.z << ", "
                    << pose.q0 << ", " << pose.q1 << ", " << pose.q2 << ", " << pose.q3 << ')';
    return debug.space();
}

#endif // POSE6DOF_H
