#pragma once

#include <cmath>

#include <array>

class Translation2d{
public:
    Translation2d(std::array<double,2>);
    Translation2d(double, double);
    Translation2d();
    double GetX() const;
    double GetY() const;
    double GetRotation() const;
    double GetDistance() const;
    std::array<double,2> GetCoords();
    Translation2d Plus(Translation2d const other);
    Translation2d Scale(double const scalar);
    Translation2d RotateBy(double const rotation);
private:
    std::array<double, 2> pose;
};