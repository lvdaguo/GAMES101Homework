#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
const Eigen::Vector3f x_axis = Eigen::Vector3f(1, 0, 0);
const Eigen::Vector3f y_axis = Eigen::Vector3f(0, 1, 0);
const Eigen::Vector3f z_axis = Eigen::Vector3f(0, 0, 1);

auto deg2rad = [](double deg) { return deg / 180 * MY_PI; };

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

// homework stuff
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0,-eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f z_rot = get_rotation_matrix(z_axis, rotation_angle);
    model = z_rot * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    const double half_fov_deg = eye_fov / 2, half_fov_rad = deg2rad(half_fov_deg);
    const double n = -zNear, f = -zFar;
    const double t = abs(n * tan(half_fov_rad)), b = -t;
    const double r = abs(t * aspect_ratio), l = -r;
    Eigen::Matrix4f persp;
    persp << n, 0, 0, 0,
             0, n, 0, 0,
             0, 0, n + f, -n * f,
             0, 0, 1, 0;

    Eigen::Matrix4f ortho_translation = get_translation_matrix(-(l + r) / 2, -(b + t) / 2, -(f + n) / 2);
    Eigen::Matrix4f ortho_scale = get_scale_matrix(2 / (r - l), 2 / (t - b), 2 / (n - f));
    Eigen::Matrix4f ortho = ortho_scale * ortho_translation;

    return ortho * persp * projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
