#include "utils/Teleplot.h"
#include <cmath>
#include "utils/translation2d.h"

using namespace std;
#include <array>

class pose2d{
private:

    std::array<double, 3> speeds;

    pose2d(std::array<double,3> inputspeeds) {
        speeds = inputspeeds;
    }
    translation2d getTranslation() {
        return translation2d(speeds[0], speeds[1]);
    }
    double getx() {
        return speeds[0];
    }
    double gety() {
        return speeds[1];
    }
    double getTurn() {
        return speeds[2];
    }
    std::array<double,3> getSpeeds() {
        return speeds;
    }
    
    Teleplot teleplot = Teleplot("127.0.0.1", 47269);


public:

};