#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <cmath>
#include <random>

// #include "matplotlib-cpp/matplotlibcpp.h"
#include "laser_scan_lite/msg/laser_scan_lite.hpp"

// namespace plt = matplotlibcpp;
using namespace std::chrono_literals;

class tester_node : public rclcpp::Node
{
private:
    rclcpp::Publisher<laser_scan_lite::msg::LaserScanLite>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;


    std::random_device seed_gen;
    std::default_random_engine engine;
    std::uniform_real_distribution<> generator;

    float m[3] = {5, -3, 9}, n[3] = {16, -6, -46};

    float delta_deg = 0.1f;
    float min_deg = 0, max_deg = 270;

    float range[1800];

    float get_range(float _deg){
        /** 
         * y = tan(_deg * M_PI / 180) * x;
         * y = m1 * x + n1
        */
        float x[3], y[3];
        for(int i = 0; i < 3; i++){
            x[i] = this->n[i] / (tanf(_deg * M_PI / 180) - m[i]);
            y[i] = this->m[i] * x[i] + this->n[i];   

            if(_deg >= 0 && _deg < 90)
            {
                if(x[i] >= 0 && y[i] >= 0){
                    return sqrtf( powf(x[i], 2.0f) + powf(y[i], 2.0f));
                }
            }
            else if (_deg >= 90 && _deg < 180)
            {
                if(x[i] <= 0 && y[i] >= 0){
                    return sqrtf( powf(x[i], 2.0f) + powf(y[i], 2.0f));
                }
            }
            else{
                if(x[i] <= 0 && y[i] <= 0){
                    return sqrtf( powf(x[i], 2.0f) + powf(y[i], 2.0f));
                }
            }
            
        }
    }

    void timer_callback(){
        laser_scan_lite::msg::LaserScanLite pub_data;

        pub_data.angle_min = this->min_deg;
        pub_data.angle_max = this->max_deg;
        pub_data.angle_increment = this->delta_deg * M_PI / 180;

        for(int i = 0; i < 1800; i++){
            pub_data.ranges[i] = this->range[i];
        }

        this->publisher_->publish(pub_data);
    }

public:
    tester_node()
        : Node("tester_node")
        , engine(seed_gen())
        , generator(-0.5f, 0.5f)
    {
        for(int i = 0; i < 1800; i++){
            this->range[i] = this->get_range(i);
            this->range[i] += generator(engine);
        }
        
        publisher_ = this->create_publisher<laser_scan_lite::msg::LaserScanLite>("laser_fullscan", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&tester_node::timer_callback, this));

    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tester_node>());
    rclcpp::shutdown();
    return 0;
}

