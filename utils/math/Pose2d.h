#pragma once

#include <cmath>
#include "Translation2d.h"

#include <array>

class Pose2d{
private:
    std::array<double, 3> speeds; // x, y, rotation
public:
    Pose2d(std::array<double,3>);
    Pose2d(double, double, double);
    Pose2d(Translation2d, double);
    Translation2d GetTranslation() const;
    double GetX() const;
    double GetY() const;
    double GetTurn() const;
    std::array<double,3> GetSpeeds() const;

};