#include "../utils/telemetry/Teleplot.h"
#include "../utils/math/pose2d.h"
#include "../utils/math/translation2d.h"
#include "../utils/swerve/PodState.h"
#include "DrivePod.h"
#include <cmath>
#include "../Constants.h"


#include "diffySwerve.h"
#include "ctre/phoenix6/Pigeon2.hpp"
using namespace ctre::phoenix6;
using namespace std;
#include <vector>
#include <array>

diffySwerve::diffySwerve() : gyro(25, CANBUS_NAME) {
    // Initialize drive pods with their configurations
    Constants::PodConfig podConfig1(1, 2, 3, -0.2073, translation2d(-1.0, 0.0));
    Constants::PodConfig podConfig2(4, 5, 6, -0.2073, translation2d(1.0, 0.0));
    
    drivePods.push_back(std::make_unique<DrivePod>(podConfig1, globalOutputScalar, isTurningSupplier));
    drivePods.push_back(std::make_unique<DrivePod>(podConfig2, globalOutputScalar, isTurningSupplier));
}

void diffySwerve::SetRobotSpeed(pose2d speeds) {
    for (size_t i = 0; i < drivePods.size(); ++i) {
        DrivePod& pod = *drivePods[i];
        translation2d podSpeed = CalculatePodSpeed(speeds, pod);
        PodState targetState(podSpeed.GetDistance(), podSpeed.GetRotation());
        pod.SetPodState(targetState);
        auto coords = podSpeed.GetCoords();
        std::string coordsStr = std::to_string(coords[0]) + "," + std::to_string(coords[1]);
        teleplot.update("pod" + std::to_string(i) + "speed", coordsStr, "m/s");
    }

}
translation2d diffySwerve::CalculatePodSpeed(pose2d speeds, const DrivePod& pod) {
    translation2d podLocation = pod.GetPosition();
    translation2d turnAmount = translation2d(speeds.getTurn() * podLocation.GetDistance(), podLocation.GetRotation());
    return turnAmount.Plus(speeds.GetTranslation());
}
pose2d diffySwerve::GetRobotSpeed() const {
    translation2d translationalSpeed = translation2d(0.0, 0.0);
    for (size_t i = 0; i < drivePods.size(); i++) {
        DrivePod& pod = *drivePods[i];
        translationalSpeed.Plus(translation2d(pod.GetPodState().PodSpeed * std::cos(pod.GetPodState().Angle), pod.GetPodState().PodSpeed * std::sin(pod.GetPodState().Angle)));
    }
    translationalSpeed.Scale(1.0/drivePods.size());
    double rotationalSpeed = 0.0;
    for (size_t i = 0; i < drivePods.size(); i++) {
        DrivePod& pod = *drivePods[i];
        rotationalSpeed += pod.GetPodState().PodSpeed * std::sin(pod.GetPodState().Angle - pod.GetPosition().GetRotation()) / pod.GetPosition().GetDistance();
    }
    rotationalSpeed /= drivePods.size();
    pose2d speeds = pose2d(translationalSpeed.GetX(), translationalSpeed.GetY(), rotationalSpeed);
    return speeds;
}
void diffySwerve::Periodic() {
    position = pose2d(position.GetTranslation().Plus(translation2d(GetRobotSpeed().GetTranslation()).RotateBy(GetGyro())), GetGyro());

    //temporary boolean to check if any pod is turning
    bool anyPodTurning = false;
    for(auto& pod : drivePods) {
        if (pod->isPodTurning()) {
            anyPodTurning = true;
        }
        
        //also run periodic here as all of the object exist here
        pod->Periodic();
    }

    isTurningSupplier = anyPodTurning;
}
double diffySwerve::GetGyro() {
    return gyro.GetYaw().GetValueAsDouble();
}