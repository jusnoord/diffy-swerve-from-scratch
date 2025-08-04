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
    std::vector<DrivePod> drivePods;
    drivePods.add(DrivePod(translation2d({-1.0, 0.0})));
    drivePods.add(DrivePod(translation2d({1.0, 0.0})));
    
    Teleplot teleplot = Teleplot("127.0.0.1", 47269);


public:

    void SetRobotSpeed(pose2d speeds) {
        for (DrivePod pod : drivePods) {
            translation2d podSpeed = CalculatePodSpeed(speeds, pod);
            pod.SetPodState(PodState(podSpeed.getDistance(), podSpeed.getRotation()));
            teleplot.update("pod" + std::to_string(i) + "speed", podSpeed.getCoords(), "m/s");
        }

    }
    translation2d CalculatePodSpeed(pose2d speeds, DrivePod pod) {
        translation2d podLocation = pod.GetPosition();
        translation2d turnAmount = new translation2d(speeds.getTurn() * podLocation.getDistance(), podLocation.getRotation());
        return turnAmount.add(speeds.getTranslation());
    }
    pose2d getRobotSpeed(){
        pose2d speeds = pose2d({0.0, 0.0, 0.0}); // Initialize with zero speeds
        return speeds;
    }




};