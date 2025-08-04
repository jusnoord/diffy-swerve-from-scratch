#include "utils/Teleplot.h"
#include <cmath>
#include "utils/translation2d.h"

using namespace std;
#include <array>

class pose2d{
private:
    


public:

    pose2d(std::array<double,3>);
    translation2d getTranslation();
    double getx();
    double gety();
    double getTurn();
    std::array<double,3> getSpeeds();

};



#endif