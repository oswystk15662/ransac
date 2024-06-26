#pragma once

#include <cmath>

class Point2D{
public:
    float x;
    float y;

private:
    Point2D operator+(const Point2D& _p) const {return {x + _p.x, y + _p.y};}; // const修飾子をつけると定数が計算に入っても大丈夫
    Point2D operator-(const Point2D& _p) const {return {x - _p.x, y - _p.y};};
    Point2D operator*(const Point2D& _p) const {return {x * _p.x, y * _p.y};};
    Point2D operator/(const Point2D& _p) const {return {x / _p.x, y / _p.y};};
};

class Twist{
public:
    float vx;
    float vy;
    float omega;

    Twist operator+(const Twist& _twist) const {return {vx + _twist.vx, vy + _twist.vy, omega + _twist.omega};};
    Twist operator-(const Twist& _twist) const {return {vx - _twist.vx, vy - _twist.vy, omega - _twist.omega};};
    Twist operator*(const Twist& _twist) const {return {vx * _twist.vx, vy * _twist.vy, omega * _twist.omega};};
    Twist operator/(const Twist& _twist) const {return {vx / _twist.vx, vy / _twist.vy, omega / _twist.omega};};

    Twist operator*(const float& _f) const {return {vx *_f, vy * _f, omega * _f};};
    Twist operator/(const float& f) const {return {vx / f, vy / f, omega / f};};
};

class Pose{
public:
    float x;
    float y;
    float theta; // 機体は最初0を向いている。

    Pose operator+(const Pose& _pose) const { return {x + _pose.x, y + _pose.y, theta + _pose.theta}; };
    Pose operator-(const Pose& _pose) const { return {x - _pose.x, y - _pose.y, theta - _pose.theta}; };
    Pose operator*(const Pose& _pose) const { return {x * _pose.x, y * _pose.y, theta * _pose.theta}; };
    Pose operator/(const Pose& _pose) const { return {x / _pose.x, y / _pose.y, theta / _pose.theta}; };
    Twist operator/(const float& _f) const { return {x / _f, y / _f, theta / _f}; };
};


typedef struct {
    float wheel_radius; // 車輪の半径 [mm]
    float wheel_x; // 車輪の中心のx座標 [mm]
    float wheel_y; // 車輪の中心のy座標 [mm]
    float wheel_theta; // 車輪の方向 [rad]
} WheelConfig;

typedef struct {
    float kx; // x方向の速度に対する車輪の速度の寄与
    float ky; // y方向の速度に対する車輪の速度の寄与
    float ktheta; // 角速度に対する車輪の速度の寄与
} WheelVector;


typedef struct {
    float kx; // x方向の速度への車輪の速度の寄与
    float ky; // y方向の速度に対する車輪の速度の寄与
    float ktheta; // 角速度に対する車輪の速度の寄与
} WheelVectorInv;

inline constexpr float radiansMod(float value, float min=-M_PI, float max=M_PI) {
    return fmod(fmod(value + min, max - min) + max - min, max - min) + min;
}

// 車輪の位置から車輪のベクトル(vx, vy, omegaそれぞれの係数)を計算する
// 車輪の速度ベクトルが(vx, vy) + omega * (-wheel_y, wheel_x)で、これの車輪の方向(cos(theta), sin(theta))との内積を取り、そのvx, vy, omegaの係数を求める
inline constexpr WheelVector getWheelVector(const WheelConfig wheel_config) {
    float wheel_circumference = 2 * (float)M_PI * wheel_config.wheel_radius;
    float kx = cos(wheel_config.wheel_theta) / wheel_circumference;
    float ky = sin(wheel_config.wheel_theta) / wheel_circumference;
    return WheelVector{
        kx: kx,
        ky: ky,
        ktheta: (wheel_config.wheel_x * ky - wheel_config.wheel_y * kx)
    };
}

// 車輪の速度を計算する
inline float getWheelSpeedRelative(const Twist twist, const WheelVector wheel_vector) {
    return wheel_vector.kx * twist.vx + wheel_vector.ky * twist.vy + wheel_vector.ktheta * twist.omega;
}

// フィールド座標系での速度をロボット座標系の速度に変換する
inline Twist absoluteTwistToRelativeTwist(const Twist twist, const WheelVector wheel_vector, const Pose pose) {
    return Twist{
        vx: twist.vx * cos(pose.theta) + twist.vy * sin(pose.theta),
        vy: -twist.vx * sin(pose.theta) + twist.vy * cos(pose.theta),
        omega: twist.omega
    };
}

// ロボット座標系での速度をフィールド座標系の速度に変換する
inline Twist relativeTwistToAbsoluteTwist(const Twist twist, const WheelVector wheel_vector, const Pose pose) {
    return Twist{
        vx: twist.vx * cos(pose.theta) - twist.vy * sin(pose.theta),
        vy: twist.vx * sin(pose.theta) + twist.vy * cos(pose.theta),
        omega: twist.omega
    };
}

