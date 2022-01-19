//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H

extern bool ssaa_on;
constexpr int ssaa_ratio = 4;
const float sample_displacement[4][2] = { {-0.25f, 0.25f}, {0.25f, 0.25f}, {0.25f, -0.25f}, {-0.25f, -0.25f} };

constexpr double PI = 3.1415926;
const Eigen::Vector3f x_axis = Eigen::Vector3f(1, 0, 0);
const Eigen::Vector3f y_axis = Eigen::Vector3f(0, 1, 0);
const Eigen::Vector3f z_axis = Eigen::Vector3f(0, 0, 1);

const Eigen::Vector3f black = Eigen::Vector3f(0, 0, 0);
const Eigen::Vector3f white = Eigen::Vector3f(255, 255, 255);

auto deg2rad = [](double deg) { return deg / 180 * PI; };

// get rotation matrix by rotating around x axis by some degrees
Eigen::Matrix4f get_rotation_matrix_x(const double x_deg);

// get rotation matrix by rotating around y axis by some degrees
Eigen::Matrix4f get_rotation_matrix_y(const double y_deg);

// get rotation matrix by rotating around z axis by some degrees
Eigen::Matrix4f get_rotation_matrix_z(const double z_deg);

// the axis order is x -> y -> z
// get rotation matrix combined from x, y, z axis
Eigen::Matrix4f get_rotation_matrix(const double x, const double y, const double z);

// get rotation matrix combined from x, y, z axis
Eigen::Matrix4f get_rotation_matrix(const Eigen::Vector3f euler_angle);

// get rotation matrix rotating around any axis for some degrees
// ref:
// 三维旋转：欧拉角、四元数、旋转矩阵、轴角之间的转换 - 鸡哥的文章 - 知乎
// https://zhuanlan.zhihu.com/p/45404840
Eigen::Matrix4f get_rotation_matrix(const Eigen::Vector3f axis, const double angle);

Eigen::Matrix4f get_translation_matrix(const double x, const double y, const double z);

Eigen::Matrix4f get_translation_matrix(const Eigen::Vector3f translation);

Eigen::Matrix4f get_scale_matrix(const double x, const double y, const double z);

Eigen::Matrix4f get_scale_matrix(const Eigen::Vector3f scale);
#endif //RASTERIZER_GLOBAL_H
