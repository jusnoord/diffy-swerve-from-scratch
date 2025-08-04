#include <cmath>

using namespace std;
#include <array>
#include "translation2d.h"

std::array<double, 2> pose;

translation2d::translation2d() {
    pose = {0.0, 0.0};
}

translation2d::translation2d(array<double,2> inputpose) {
    pose = inputpose;
}
translation2d::translation2d(double speed, double rotation) {
    pose = {speed * std::cos(rotation), speed * std::sin(rotation)};
}
double translation2d::GetX() const {
    return pose[0];
}
double translation2d::GetY() const {
    return pose[1];
}
double translation2d::GetRotation() const {
    return std::atan2(pose[1], pose[0]);
}
double translation2d::GetDistance() const {
    return std::sqrt(pose[0] * pose[0] + pose[1] * pose[1]); 
}
array<double,2> translation2d::GetCoords() {
    return pose;
}
translation2d translation2d::Plus(translation2d const other) {
    return translation2d(std::array<double, 2>{pose[0] + other.GetX(), pose[1] + other.GetY()});
}
void translation2d::Scale(double scalar) {
    pose = {pose[0] * scalar, pose[1] * scalar};
}
translation2d translation2d::RotateBy(double rotation) {
    rotation += std::atan2(pose[1], pose[0]);
    double speed = std::sqrt(pose[0] * pose[0] + pose[1] * pose[1]);
    return translation2d(speed * std::cos(rotation), speed * std::sin(rotation));
}