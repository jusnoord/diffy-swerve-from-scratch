#pragma once

#include <cmath>

using namespace std;
#include <array>

class translation2d{
public:
    translation2d(std::array<double,2>);
    translation2d(double, double);
    translation2d();
    double GetX() const;
    double GetY() const;
    double GetRotation() const;
    double GetDistance() const;
    std::array<double,2> GetCoords();
    translation2d Plus(translation2d const other);
    void Scale(double scalar);
    translation2d RotateBy(double const rotation);
};