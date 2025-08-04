#pragma once

#include <cmath>
#include "translation2d.h"

using namespace std;
#include <array>

class pose2d{
private:
    std::array<double, 3> speeds; // x, y, rotation
public:
    pose2d(std::array<double,3>);
    pose2d(double, double, double);
    pose2d(translation2d, double);
    translation2d GetTranslation();
    double GetX();
    double GetY();
    double getTurn();
    std::array<double,3> getSpeeds();

};