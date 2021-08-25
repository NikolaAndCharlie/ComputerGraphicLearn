#include<cmath>
#include <Eigen/Eigen>
#include<iostream>

int main(){

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
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v

    using namespace std;
    Eigen::Vector3f P(2.0, 1.0, 1);
    Eigen::Matrix3f rotate_mat;
    rotate_mat << std::cos(45.0/180.0 * acos(-1)), -sin(45.0 / 180.0 * acos(-1)), 0, 
    sin(45.0/180.0 * acos(-1)), cos(45.0/180.0*acos(-1)),
    0, 0, 0, 1;
    Eigen::Matrix3f translate_mat;
    double tx = 1.0;
    double ty = 2.0;
    cout << "旋转矩阵" << endl;
    cout << rotate_mat << endl;

    translate_mat << 1, 0, tx,
                     0, 1, ty,
                     0, 0, 1;
    cout << "平移矩阵" << endl;
    cout << translate_mat << endl;
    Eigen::Vector3f result_after_rotate(0, 0, 0);
    result_after_rotate = rotate_mat * P;

    std::cout <<"旋转变换的结果"<< endl;
    std::cout << result_after_rotate << endl;
    Eigen::Vector3f result_after_translate;
    result_after_translate = translate_mat * result_after_rotate;
    cout << "平移变换结果" << endl;
    cout << result_after_translate << endl;
    cin.get();
    cin.get();
    return 0;
}