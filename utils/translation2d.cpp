
#include "utils/Teleplot.h"
#include <cmath>

using namespace std;
#include <array>

class translation2d{
private:

    std::array<double, 2> pose;

    translation2d(std::array<double,2> inputpose) {
        pose = inputpose;
    }
    translation2d(double speed, double rotation) {
        pose = {speed * Math::cos(rotation), speed * Math::sin(rotation)};
    }
    double getx() {
        return pose[0];
    }
    double gety() {
        return pose[1];
    }
    double getRotation() {
        return Math::atan2(pose[1], pose[0]);
    }
    double getDistance() {
        return Math::sqrt(pose[0] * pose[0] + pose[1] * pose[1]); 
    }
    std::array<double,2> getCoords() {
        return pose;
    }
    translation2d add(translation2d const& other) {
        return translation2d({pose[0] + other.getx(), pose[1] + other.gety()});
    }
    
    Teleplot teleplot = Teleplot("127.0.0.1", 47269);


public:

};







