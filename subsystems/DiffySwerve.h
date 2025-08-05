#pragma once

#include "ctre/phoenix6/TalonFXS.hpp"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "../RobotBase.hpp"
#include "../utils/controller/Joystick.hpp"
#include "../utils/math/Pid.h"
#include "../utils/telemetry/Teleplot.h"
#include "../utils/math/Pose2d.h"
#include "../utils/math/Translation2d.h"
#include "../utils/swerve/PodState.h"
#include "DrivePod.h"
#include <cmath>

#include <vector>
#include <array>

class DiffySwerve{
private:
    hardware::Pigeon2 gyro;
    uint32_t millis();
    int loopTime = millis();
    Teleplot teleplot = Teleplot("127.0.0.1", 47269);
    std::vector<std::unique_ptr<DrivePod>> drivePods;
    Pose2d position = Pose2d(0.0, 0.0, 0.0);
    double globalOutputScalar = 1.0; 
    bool isTurningSupplier = false; //reference to a boolean that indicates if any pod is turning. should be read-only within other classes
    Translation2d CalculatePodSpeed(Pose2d, const DrivePod&);
    static constexpr char const *CANBUS_NAME = "*";


public:
    DiffySwerve();

    void SetRobotSpeed(Pose2d); 
    Pose2d GetRobotSpeed() const;
    Pose2d GetRobotPosition();
    double GetGyro();
    void Periodic();
};

