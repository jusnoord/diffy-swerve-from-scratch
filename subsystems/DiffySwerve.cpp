#include "../utils/telemetry/Teleplot.h"
#include "../utils/math/Pose2d.h"
#include "../utils/math/Translation2d.h"
#include "../utils/swerve/PodState.h"
#include "DrivePod.h"
#include <cmath>
#include "../Constants.h"

#include "DiffySwerve.h"
#include "ctre/phoenix6/Pigeon2.hpp"
#include <vector>
#include <array>
#include <chrono>
using namespace ctre::phoenix6;

// Returns milliseconds since program start
uint32_t DiffySwerve::millis()
{
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

DiffySwerve::DiffySwerve() : gyro(Constants::RobotMap::PIGEON_ID, Constants::RobotMap::CANBUS_NAME)
{
    drivePods.push_back(std::make_unique<DrivePod>(Constants::RobotMap::podConfig1, globalOutputScalar, isTurningSupplier, teleplot));
    drivePods.push_back(std::make_unique<DrivePod>(Constants::RobotMap::podConfig2, globalOutputScalar, isTurningSupplier, teleplot));
}

void DiffySwerve::SetRobotSpeed(Pose2d speeds)
{
    globalOutputScalar = 1.0; // Reset the global output scalar for each new speed command
    
    for (auto &pod : drivePods)
    {
        Translation2d podSpeed = CalculatePodSpeed(speeds, *pod);
        PodState targetState(podSpeed.GetDistance(), podSpeed.GetRotation());
        pod->SetPodState(targetState);
        // auto coords = podSpeed.GetCoords();
        // std::string coordsStr = std::to_string(coords[0]) + "," + std::to_string(coords[1]);
        // teleplot.update("pod speed", coordsStr, "m/s");
    }
}
Translation2d DiffySwerve::CalculatePodSpeed(Pose2d speeds, const DrivePod &pod)
{
    Translation2d podLocation = pod.GetPosition();
    Translation2d turnAmount = Translation2d(speeds.GetTurn() * podLocation.GetDistance(), podLocation.GetRotation());
    return turnAmount.Plus(speeds.GetTranslation());
}
Pose2d DiffySwerve::GetRobotSpeed() const
{
    Translation2d translationalSpeed = Translation2d(0.0, 0.0);
    for (auto &pod : drivePods)
    {
        translationalSpeed.Plus(Translation2d(pod->GetPodState().PodSpeed * std::cos(pod->GetPodState().Angle), pod->GetPodState().PodSpeed * std::sin(pod->GetPodState().Angle)));
    }
    translationalSpeed.Scale(1.0 / drivePods.size());
    double rotationalSpeed = 0.0;
    for (auto &pod : drivePods)
    {
        rotationalSpeed += pod->GetPodState().PodSpeed * std::sin(pod->GetPodState().Angle - pod->GetPosition().GetRotation()) / pod->GetPosition().GetDistance();
    }
    rotationalSpeed /= drivePods.size();
    Pose2d speeds = Pose2d(translationalSpeed.GetX(), translationalSpeed.GetY(), rotationalSpeed);
    return speeds;
}
void DiffySwerve::Periodic()
{
    position = Pose2d(position.GetTranslation().Plus(GetRobotSpeed().GetTranslation().Scale(millis() - loopTime).RotateBy(GetGyro())), GetGyro());
    loopTime = millis();

    // temporary boolean to check if any pod is turning
    bool anyPodTurning = false;
    for (auto &pod : drivePods)
    {
        if (pod->isPodTurning())
        {
            anyPodTurning = true;
        }

        // also run periodic here as all of the object exist here
        pod->Periodic();
    }

    isTurningSupplier = anyPodTurning;
}
double DiffySwerve::GetGyro()
{
    return gyro.GetYaw().GetValueAsDouble();
}