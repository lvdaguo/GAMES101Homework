//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;

        // for bilinear
        // down sampling the texture
        // width = image_data.cols / 2;
        // height = image_data.rows / 2;
        // cv::pyrDown(image_data, image_data, cv::Size(width, height));
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;

        int u_min = floor(u_img), u_max = ceil(u_img);
        int v_min = floor(v_img), v_max = ceil(v_img);

        cv::Vec3b u00 = image_data.at<cv::Vec3b>(v_min, u_min);
        cv::Vec3b u01 = image_data.at<cv::Vec3b>(v_max, u_min);
        cv::Vec3b u10 = image_data.at<cv::Vec3b>(v_min, u_max);
        cv::Vec3b u11 = image_data.at<cv::Vec3b>(v_max, u_max);
        
        auto lerp = [](const cv::Vec3b& v0, const cv::Vec3b& v1, float x) -> cv::Vec3b
        {
            return (1.f - x) * v0 + x * v1;
        };

        float s = (u_img - u_min) / (u_max - u_min);
        float t = (v_img - v_min) / (v_max - v_min);

        cv::Vec3b u0 = lerp(u00, u10, s);
        cv::Vec3b u1 = lerp(u01, u11, s);

        cv::Vec3b color = lerp(u0, u1, t);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
