#pragma once

#include "ctre/phoenix6/TalonFXS.hpp"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "../RobotBase.hpp"
#include "../utils/controller/Joystick.hpp"
#include "../utils/math/pid.h"
#include "../utils/telemetry/Teleplot.h"
#include "../utils/math/pose2d.h"
#include "../utils/math/translation2d.h"
#include "../utils/swerve/PodState.h"
#include "DrivePod.h"
#include <cmath>

using namespace std;
#include <vector>
#include <array>

class diffySwerve{
private:
    hardware::Pigeon2 gyro;
    auto loopTime = millis();
    Teleplot teleplot = Teleplot("127.0.0.1", 47269);
    std::vector<std::unique_ptr<DrivePod>> drivePods;
    pose2d position = pose2d(0.0, 0.0, 0.0);
    double globalOutputScalar = 1.0; 
    bool isTurningSupplier = false; //reference to a boolean that indicates if any pod is turning. should be read-only within other classes
    translation2d CalculatePodSpeed(pose2d, const DrivePod&);
    static constexpr char const *CANBUS_NAME = "*";


public:
    diffySwerve();

    void SetRobotSpeed(pose2d); 
    pose2d GetRobotSpeed() const;
    pose2d GetRobotPosition();
    double GetGyro();
    void Periodic();
};

