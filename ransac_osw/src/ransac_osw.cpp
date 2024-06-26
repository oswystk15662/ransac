#include "ransac_osw.hpp"

using namespace osw_no_heya;

Ransac_node::Ransac_node()
    : Node("Ransac_node")
    , engine(seed_gen())
    , generator(0, DAITAI_SAIDAI_PARTICLE_NUM)
{
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>( "topic", 10, std::bind(&Ransac_node::lidar_callback, this, std::placeholders::_1));

    // yamlから読み込む変数の名前（stringで記述）をros2に教える。第２引数はなんでもいい
    declare_parameter("min_rad", 0);
    declare_parameter("Max_rad", M_PI);
    declare_parameter("Max_loop", 100);
    declare_parameter("threshould", 0.5f);

    declare_parameter("min_line_particles", 100);


    this->min_rad_ = get_parameter("min_rad").as_double();
    this->Max_rad_ = get_parameter("Max_rad").as_double();

    this->Max_loop_ = get_parameter("Max_loop").as_int();
    this->threshould_ = get_parameter("threshould").as_double();

    this->MIN_LINE_PARTICLES_ = get_parameter("min_line_particles").as_int();
}

void Ransac_node::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    this->num_of_particles_ = msg->ranges.size();
    RCLCPP_INFO(this->get_logger(), "num of particles = %d\r\n", this->num_of_particles_);

    for (int i = 0; i < (int)msg->ranges.size(); i++) {
        float angle = msg->angle_min + i * msg->angle_increment;

        if(angle >= this->min_rad_ && angle <= this->Max_rad_){
            this->points_[i].x = msg->ranges[i] * cosf(-angle);
            this->points_[i].y = msg->ranges[i] * sinf(-angle);
        }
    }

    this->ransac();
}

void Ransac_node::ransac(){
    int best_p_cnt = 0;
    float best_param;

    for(int i = 0; i < this->Max_loop_; i++)
    {
        int random_num_1 = generator(engine);
        int random_num_2 = generator(engine);
        
        if(random_num_1 == random_num_2){
            continue; // 次のループへ
        }

        Point2D point_1 = this->points_[random_num_1];
        Point2D point_2 = this->points_[random_num_2];
        
        float abc[3];
        this->calc_param(abc, point_1, point_2);
        
        int p_cnt = 0;

        for(int j = 0; j < this->num_of_particles_; j++){
            float d = this->calc_distance_point2line(abc, this->points_[j]);
            if(d <= this->threshould_){
                p_cnt++;
            }
        }

        if(p_cnt > best_p_cnt){
            best_p_cnt = p_cnt;
            for(int k = 0; k < 3; k++){
                this->best_abc_[k] = abc[k];
            }
        }
        RCLCPP_INFO(this->get_logger(), "best is %d", best_p_cnt);
    }

    if(best_p_cnt <= 150){
        RCLCPP_INFO(this->get_logger(), "No line\r\n");
        this->is_line_ = false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "%3.2f * x + %3.2f * y + %3.2f = 0\r\n", this->best_abc_[0], this->best_abc_[1], this->best_abc_[2]);
        this->is_line_ = true;
    }
    
}

void Ransac_node::calc_param(float* _abc, Point2D _point1, Point2D _point2){
    if(_abc != nullptr){
        float dx = _point2.x - _point1.x;
        float dy = _point2.y - _point1.y;
        
        // y = x * A + B
        // A = dy / dx
        // y1 = x1 * dy / dx + B <=> B =  y1 - x1 * dy / dx
        // dx * y = x * dy + A * dx
        // dy * x + -dx * y + dx * B = 0
        // a  * x + b   * y + c = 0
        _abc[0] = dy;
        _abc[1] = -dx;
        _abc[2] = _point1.y * dx - _point1.x * dy;
    }
}

float Ransac_node::calc_distance_point2line(float* _abc , Point2D _point){
    float noum =  sqrt( powf(_abc[0], 2.0f) + powf(_abc[1],2.0f) );
    
    return fabs(_abc[0] * _point.x + _abc[1] * _point.y + _abc[2]) / noum;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<osw_no_heya::Ransac_node>());
    rclcpp::shutdown();

    return 0;
}

// float Ransac_node::model(float* _params, float _x){
//     // float a = params[0]
//     // float b = params[1]
//     // float c = params[2]
//     // float katakumi = -a / b;
//     // float seppenn = -c / b;
//     // return katakumi*x+seppenn;
// }
