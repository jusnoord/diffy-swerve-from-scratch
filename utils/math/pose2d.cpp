#include <cmath>
#include "translation2d.h"
#include <array>
#include "pose2d.h"

using namespace std;

pose2d::pose2d(std::array<double,3> inputspeeds) {
    speeds = inputspeeds;
}

pose2d::pose2d(double x, double y, double rotation) {
    speeds = {x, y, rotation};
}

pose2d::pose2d(translation2d translation, double rotation) {
    speeds = {translation.GetX(), translation.GetY(), rotation};
}

translation2d pose2d::GetTranslation() {
    return translation2d(speeds[0], speeds[1]);
}
double pose2d::GetX() {
    return speeds[0];
}
double pose2d::GetY() {
    return speeds[1];
}
double pose2d::getTurn() {
    return speeds[2];
}
array<double,3> pose2d::getSpeeds() {
    return speeds;
}