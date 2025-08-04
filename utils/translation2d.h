#include "utils/Teleplot.h"
#include <cmath>

using namespace std;
#include <array>

class translation2d{
private:
    


public:

    translation2d(std::array<double,2>);
    translation2d(double, double);
    double getx();
    double gety();
    double getRotation();
    double getDistance();
    std::array<double,2> getCoords();
    translation2d add(translation2d const& other);

};



#endif