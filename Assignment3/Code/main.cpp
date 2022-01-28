#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

constexpr double PI = 3.1415926;
const Eigen::Vector3f x_axis = Eigen::Vector3f(1, 0, 0);
const Eigen::Vector3f y_axis = Eigen::Vector3f(0, 1, 0);
const Eigen::Vector3f z_axis = Eigen::Vector3f(0, 0, 1);

const Eigen::Vector3f black = Eigen::Vector3f(0, 0, 0);
const Eigen::Vector3f white = Eigen::Vector3f(255, 255, 255);

auto deg2rad = [](double deg) { return deg / 180 * PI; };

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

// GAMES101-作业1、作业2、作业3的解题和框架分析 - BSC5622的文章 - 知乎
// https://zhuanlan.zhihu.com/p/425153734

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
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

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        float u = payload.tex_coords.x(), v = payload.tex_coords.y();
        return_color = payload.texture->getColor(u, v);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f& intensity = light.intensity;
        Eigen::Vector3f vec_l = light.position - point;
        Eigen::Vector3f vec_v = -point;

        float r2 = vec_l.squaredNorm();

        vec_l.normalize();
        vec_v.normalize();

        Eigen::Vector3f diffuse = kd.cwiseProduct(intensity / r2 * std::max(0.f, normal.dot(vec_l)));

        Eigen::Vector3f ref = reflect(vec_l, normal);
        Eigen::Vector3f specular = ks.cwiseProduct(intensity / r2 * std::pow(std::max(0.f, ref.dot(vec_v)), p));
        
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        // Then, accumulate that result on the *result_color* object.
        // for each light
        result_color += diffuse + specular + ambient;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    // struct light
    // {
    //     Eigen::Vector3f position;
    //     Eigen::Vector3f intensity;
    // };

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. 
        Eigen::Vector3f& intensity = light.intensity;
        Eigen::Vector3f vec_l = light.position - point;
        Eigen::Vector3f vec_v = -point;

        // 这时候的eye_pos应该是0，0，0
        // 所以这里应该不需要再给定一个eye_pose变量，视线的向量应该是-point
        // 而不是网上很多内容写的eye_pos - point

        float r2 = vec_l.squaredNorm(); // r * r

        vec_l.normalize();
        vec_v.normalize();

        Eigen::Vector3f diffuse = kd.cwiseProduct(intensity / r2 * std::max(0.f, normal.dot(vec_l)));

        Eigen::Vector3f ref = reflect(vec_l, normal);
        Eigen::Vector3f specular = ks.cwiseProduct(intensity / r2 * std::pow(std::max(0.f, ref.dot(vec_v)), p));
        
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        // Then, accumulate that result on the *result_color* object.
        // for each light
        result_color += diffuse + specular + ambient;
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    auto& n = normal;
    float x = n.x(), y = n.y(), z = n.z();
    Eigen::Vector3f t = {x*y/sqrt(x*x+z*z), sqrt(x*x+z*z), z*y/sqrt(x*x+z*z)};
    Eigen::Vector3f b = n.cross(t);
    
    Eigen::Matrix3f TBN;
    TBN <<  t.x(), b.x(), n.x(),
            t.y(), b.y(), n.y(),
            t.z(), b.z(), n.z();

    auto texture = payload.texture;
    float u = payload.tex_coords.x(), v = payload.tex_coords.y();
    float w = texture->width, h = texture->height;

    float dU = kh * kn * (texture->getColor(u+1.f/w, v).norm() - texture->getColor(u, v).norm());
    float dV = kh * kn * (texture->getColor(u, v+1.f/h).norm() - texture->getColor(u, v).norm());
    Eigen::Vector3f ln = {-dU, -dV, 1};

    point += kn * n * texture->getColor(u, v).norm();
    n = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        Eigen::Vector3f& intensity = light.intensity;
        Eigen::Vector3f vec_l = light.position - point;
        Eigen::Vector3f vec_v = -point;

        float r2 = vec_l.squaredNorm();

        vec_l.normalize();
        vec_v.normalize();

        Eigen::Vector3f diffuse = kd.cwiseProduct(intensity / r2 * std::max(0.f, normal.dot(vec_l)));

        Eigen::Vector3f ref = reflect(vec_l, normal);
        Eigen::Vector3f specular = ks.cwiseProduct(intensity / r2 * std::pow(std::max(0.f, ref.dot(vec_v)), p));
        
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color += diffuse + specular + ambient;
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    auto& n = normal;
    float x = n.x(), y = n.y(), z = n.z();
    Eigen::Vector3f t = {x*y/sqrt(x*x+z*z), sqrt(x*x+z*z), z*y/sqrt(x*x+z*z)};
    Eigen::Vector3f b = n.cross(t);
    
    Eigen::Matrix3f TBN;
    TBN <<  t.x(), b.x(), n.x(),
            t.y(), b.y(), n.y(),
            t.z(), b.z(), n.z();

    auto texture = payload.texture;
    float u = payload.tex_coords.x(), v = payload.tex_coords.y();
    float w = texture->width, h = texture->height;

    float dU = kh * kn * (texture->getColor(u+1.f/w, v).norm() - texture->getColor(u, v).norm());
    float dV = kh * kn * (texture->getColor(u, v+1.f/h).norm() - texture->getColor(u, v).norm());
    Eigen::Vector3f ln = {-dU, -dV, 1};
    n = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    // set up default shader type
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a' )
        {
            angle -= 10;
        }
        else if (key == 'd')
        {
            angle += 10;
        }

    }
    return 0;
}
