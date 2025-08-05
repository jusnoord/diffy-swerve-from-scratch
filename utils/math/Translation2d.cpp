#include <cmath>

#include <array>
#include "Translation2d.h"


Translation2d::Translation2d() {
    pose = {0.0, 0.0};
}
Translation2d::Translation2d(std::array<double,2> inputpose) {
    pose = inputpose;
}
Translation2d::Translation2d(double speed, double rotation) {
    pose = {speed * std::cos(rotation), speed * std::sin(rotation)};
}
double Translation2d::GetX() const {
    return pose[0];
}
double Translation2d::GetY() const {
    return pose[1];
}
double Translation2d::GetRotation() const {
    return std::atan2(pose[1], pose[0]);
}
double Translation2d::GetDistance() const {
    return std::sqrt(pose[0] * pose[0] + pose[1] * pose[1]); 
}
std::array<double,2> Translation2d::GetCoords() {
    return pose;
}
Translation2d Translation2d::Plus(Translation2d const other) {
    return Translation2d(std::array<double, 2>{pose[0] + other.GetX(), pose[1] + other.GetY()});
}
Translation2d Translation2d::Scale(double const scalar) {
    return Translation2d(pose[0] * scalar, pose[1] * scalar);
}
Translation2d Translation2d::RotateBy(double const rotation) {
    double speed = std::sqrt(pose[0] * pose[0] + pose[1] * pose[1]);
    return Translation2d(speed * std::cos(rotation + std::atan2(pose[1], pose[0])), speed * std::sin(rotation + std::atan2(pose[1], pose[0])));
}