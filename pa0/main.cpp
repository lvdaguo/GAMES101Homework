#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

const double pi = std::acos(-1);
auto deg2rad = [](double deg) { return deg / 180.0 * pi; };

Eigen::Matrix3f rotation3f(const double);
Eigen::Matrix3f translation3f(const double, const double);

int main() {

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";

    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 
         4.0, 5.0, 6.0, 
         7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0,
         4.0, 6.0, 5.0,
         9.0, 7.0, 8.0;
    
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    
    // matrix add i + j
    std::cout << "Example of add \n";
    std::cout << i + j << std::endl;

    // matrix scalar multiply i * 2.0
    std::cout << "Example of scalar multiply \n";
    std::cout << i * 2.0 << std::endl;

    // matrix multiply i * j
    std::cout << "Example of matrix-matrix multiply \n";
    std::cout << i * j << std::endl;

    // matrix multiply vector i * v
    std::cout << "Example of matrix-vector multiply \n";
    std::cout << i * v << std::endl;

    // homework
    // 给定一个点 P =(2,1), 将该点绕原点先逆时针旋转 45 ◦ ,再平移 (1,2), 计算出
    // 变换后点的坐标(要求用齐次坐标进行计算)。
    Eigen::Vector3f P = {2.0f, 1.0f, 1.0f};
    Eigen::Matrix3f rotation = rotation3f(45.0), translation = translation3f(1.0, 2.0);

    std::cout << "Original Point P: \n";
    std::cout << P << std::endl;
    std::cout << "Point P After Transfomation: \n";
    std::cout << translation * rotation * P << std::endl;

    return 0;
}

Eigen::Matrix3f rotation3f(const double deg) {
    Eigen::Matrix3f res;
    res << cos(deg2rad(deg)), -sin(deg2rad(deg)), 0.0,
           sin(deg2rad(deg)), cos(deg2rad(deg)), 0.0,
           0.0, 0.0, 1.0; 
    return res;
}

Eigen::Matrix3f translation3f(const double x, const double y) {
    Eigen::Matrix3f res;
    res << 1.0, 0.0, x,
           0.0, 1.0, y,
           0.0, 0.0, 1.0;
    return res;
}
