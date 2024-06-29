#pragma once

// この3つはROS2 CPPを書くとき必須と思っておくと
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>

// #include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_lite/msg/laser_scan_lite.hpp"

#include <cmath>
#include <random>
// #include <matplotlib-cpp/matplotlibcpp.h>
#include "UserTypes.hpp"

namespace osw_no_heya{

#define DAITAI_SAIDAI_PARTICLE_NUM (1800)

class Ransac_node : public rclcpp::Node
{
public:
    Ransac_node();
    // ~Ransac_node();

    /// @param _return_data return a, b : y = ax + b
    /// @return this->is_line if(Ransac_node::get_line(&a, &b))とかでいい感じに処理できると思う
    bool get_line(float* _return_data);

private:
    void lidar_callback(const laser_scan_lite::msg::LaserScanLite& msg);
    void ransac();

    void calc_param(float* _abc, Point2D _point1, Point2D _point2);

    float calc_distance_point2line(float* _abc, Point2D point);
    float model(float* _params, float _x);

    std::unique_ptr<Point2D[]> points_;
    std::unique_ptr<Point2D[]> plot_points_;

    rclcpp::Subscription<laser_scan_lite::msg::LaserScanLite>::SharedPtr sub_;

    std::unique_ptr<Point2D> new_point_;

    // for LiDAR processing
    float min_rad_;
    float Max_rad_;
    uint32_t Max_loop_;
    float threshould_;

    unsigned int num_of_particles_;
    unsigned int MIN_LINE_PARTICLES_;
    float best_abc_[3];
    bool is_line_;

    // for ransac
    std::random_device seed_gen;
    std::default_random_engine engine;
    std::uniform_int_distribution<> generator;

}; // Ransac_osw

}; // namespace osw_no_heya
