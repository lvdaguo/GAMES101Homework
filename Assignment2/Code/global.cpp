#include <eigen3/Eigen/Eigen>
#include "global.hpp"

bool ssaa_on = true;

// get rotation matrix by rotating around x axis by some degrees
Eigen::Matrix4f get_rotation_matrix_x(const double x_deg)
{
    Eigen::Matrix4f res;
    const double x = deg2rad(x_deg);
    const double sinx = sin(x), cosx = cos(x);
    res << 1, 0, 0, 0,
           0, cosx, -sinx, 0,
           0, sinx, cosx, 0,
           0, 0, 0, 1;
    return res;
}

// get rotation matrix by rotating around y axis by some degrees
Eigen::Matrix4f get_rotation_matrix_y(const double y_deg)
{
    Eigen::Matrix4f res;
    const double y = deg2rad(y_deg);
    const double siny = sin(y), cosy = cos(y);
    res << cosy, 0, siny, 0,
           0, 1, 0, 0,
           -siny, 0, cosy, 0,
           0, 0, 0, 1;
    return res;
}

// get rotation matrix by rotating around z axis by some degrees
Eigen::Matrix4f get_rotation_matrix_z(const double z_deg)
{
    Eigen::Matrix4f res;
    const double z = deg2rad(z_deg);
    const double sinz = sin(z), cosz = cos(z);
    res << cosz, -sinz, 0, 0,
           sinz, cosz, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    return res;
}

// the axis order is x -> y -> z
// get rotation matrix combined from x, y, z axis
Eigen::Matrix4f get_rotation_matrix(const double x, const double y, const double z)
{
    const Eigen::Matrix4f x_rot = get_rotation_matrix_x(x);
    const Eigen::Matrix4f y_rot = get_rotation_matrix_y(y);
    const Eigen::Matrix4f z_rot = get_rotation_matrix_z(z);
    return z_rot * y_rot * x_rot;
}

// get rotation matrix combined from x, y, z axis
Eigen::Matrix4f get_rotation_matrix(const Eigen::Vector3f euler_angle)
{
    const double x = euler_angle.x(), y = euler_angle.y(), z = euler_angle.z();
    return get_rotation_matrix(x, y, z);
}

// get rotation matrix rotating around any axis for some degrees
// ref:
// 三维旋转：欧拉角、四元数、旋转矩阵、轴角之间的转换 - 鸡哥的文章 - 知乎
// https://zhuanlan.zhihu.com/p/45404840
Eigen::Matrix4f get_rotation_matrix(const Eigen::Vector3f axis, const double angle)
{
    Eigen::Matrix4f res;
    const double x = axis.x(), y = axis.y(), z = axis.z();
    const double si = sin(deg2rad(angle)), co = cos(deg2rad(angle));
    res << co+(1-co)*x*x, -si*z+(1-co)*x*y, si*y+(1-co)*x*z, 0,
           si*z+(1-co)*x*y, co+(1-co)*y*y, -si*x+(1-co)*y*z, 0,
           -si*y+(1-co)*x*z, si*x+(1-co)*y*z, co+(1-co)*z*z, 0,
           0, 0, 0, 1;
    return res; 
}

Eigen::Matrix4f get_translation_matrix(const double x, const double y, const double z)
{
    Eigen::Matrix4f res;
    res << 1, 0, 0, x,
           0, 1, 0, y,
           0, 0, 1, z,
           0, 0, 0, 1;
    return res;
}

Eigen::Matrix4f get_translation_matrix(const Eigen::Vector3f translation)
{
    const double x = translation.x(), y = translation.y(), z = translation.z();
    return get_translation_matrix(x, y, z);
}

Eigen::Matrix4f get_scale_matrix(const double x, const double y, const double z)
{
    Eigen::Matrix4f res;
    res << x, 0, 0, 0,
           0, y, 0, 0,
           0, 0, z, 0,
           0, 0, 0, 1;
    return res;
}

Eigen::Matrix4f get_scale_matrix(const Eigen::Vector3f scale)
{
    const double x = scale.x(), y = scale.y(), z = scale.z();
    return get_scale_matrix(x, y, z);
}