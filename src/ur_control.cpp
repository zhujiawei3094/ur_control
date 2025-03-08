//
// Created by zhu on 25-2-26.
//
#include "../include/ur_control/ur_controller.h"
#include "fstream"

std::string filename;
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    if (argc == 2) {
        filename = argv[1];
        std::cout<<"open "<<filename<<std::endl;
    }

    auto controller = std::make_shared<URController>();

    // // 示例：发送到初始位置
    // std::vector<double> home_position = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
    // controller->send_joint_goal(home_position, 5.0);

    std::ifstream infile;
    infile.open(filename);
    if (!infile.is_open()) {
        std::cerr << "Error opening" << std::endl;
        return 1;
    }
    std::queue<std::vector<double>> positions;
    while (!infile.eof()) {
        std::vector<double> position(6);
        infile >> position[0] >> position[1] >> position[2] >> position[3] >> position[4] >> position[5];
        positions.push(position);
    }
    controller->register_data(positions);
    infile.close();

    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}