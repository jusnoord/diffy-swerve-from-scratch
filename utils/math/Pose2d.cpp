#include <cmath>
#include "Translation2d.h"
#include <array>
#include "Pose2d.h"

Pose2d::Pose2d(std::array<double,3> inputspeeds) {
    speeds = inputspeeds;
}

Pose2d::Pose2d(double x, double y, double rotation) {
    speeds = {x, y, rotation};
}

Pose2d::Pose2d(Translation2d translation, double rotation) {
    speeds = {translation.GetX(), translation.GetY(), rotation};
}

Translation2d Pose2d::GetTranslation() const {
    return Translation2d(speeds[0], speeds[1]);
}
double Pose2d::GetX() const {
    return speeds[0];
}
double Pose2d::GetY() const {
    return speeds[1];
}
double Pose2d::GetTurn() const {
    return speeds[2];
}
std::array<double,3> Pose2d::GetSpeeds() const {
    return speeds;
}