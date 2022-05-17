#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <opencv2/highgui.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
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
    float cos_ = cos(rotation_angle);
    float sin_ = sin(rotation_angle);
    model << cos_, (-1)*sin_, 0, 0,
             sin_, cos_, 0, 0,
             0, 0, 1.0, 0,
             0, 0, 0, 1.0 ;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    float tan_ = tan(eye_fov / 2);
    float hNear = tan_ * zNear;
    float wNear = hNear * aspect_ratio;
    float hFar = tan_ * zFar;
    float wFar = aspect_ratio * hFar;
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pro_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f viewport = Eigen::Matrix4f::Identity();
    pro_ortho << zNear, 0, 0, 0,
                 0, zNear, 0, 0,
                 0, 0, zNear + zFar, zNear* zFar* (-1.0),
                 0, 0, 1.0, 0;
    viewport << 1 / wNear, 0, 0, 0,
                0, 1 / hNear, 0, 0,
                0, 0, 2 / (zFar - zNear), 0,
                0, 0, 0, 1.0;

    projection = viewport * pro_ortho;
    return projection;
}

int main()
{
    float angle = 70;
    bool command_line = false;
    std::string filename = "output.png";

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
        key = cv::waitKey(100);
        
        std::cout << "frame count: " << frame_count++ << '\n';
        angle += 0.1;
        if (key == 'a') {
            std::cout << key;
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
