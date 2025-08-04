#include "ctre/phoenix6/TalonFXS.hpp"
#include "ctre/phoenix6/CANcoder.hpp"
#include "RobotBase.hpp"
#include "utils/Joystick.hpp"
#include "utils/pid.h"
#include "utils/Teleplot.h"
#include "utils/pose2d.h"
#include "utils/translation2d.h"
#include "swerve/PodState.h"
#include "subsystems/DrivePod.h"
#include <cmath>

using namespace std;
#include <vector>
#include <array>

class diffySwerve{
private:
    Teleplot teleplot = Teleplot("127.0.0.1", 47269);


public:

    void SetRobotSpeed(pose2d); 
    pose2d GetRobotSpeed();
    pose2d GetRobotPosition();
    pose2d CalculatePodSpeed(pose2d, drivePod);
};

