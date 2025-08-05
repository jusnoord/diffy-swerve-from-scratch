#pragma once

#include "ctre/phoenix6/TalonFXS.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

#include "../utils/math/Pid.h"
#include "../utils/telemetry/Teleplot.h"
#include <cmath>
#include "../utils/swerve/PodState.h"
#include "../utils/math/Translation2d.h"
#include "../Constants.h"

using namespace ctre::phoenix6;

class DrivePod
{
public:
    DrivePod(const Constants::PodConfig &config, double &globalOutputScalar, const bool &isTurningSupplier, Teleplot &teleplot);
    Translation2d GetPosition() const;
    void Initialize();
    void Periodic();
    void SetPodState(PodState &targetState);
    PodState GetPodState() const;
    bool isPodTurning() const;

private:
    Translation2d position{0.0, 0.0};
    PID myPID{0.02, 1.0, -1.0, Constants::DrivetrainConstants::turnkP, Constants::DrivetrainConstants::turnkI, Constants::DrivetrainConstants::turnkD};
    double encoderOffset;
    double &globalOutputScalar;
    const bool &otherPodTurning;
    bool isTurning = false;
    PodState currentState{0.0, 0.0};

    static constexpr char const *CANBUS_NAME = "*";
    hardware::TalonFXS left;
    hardware::TalonFXS right;
    hardware::CANcoder encoder;
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};
    Teleplot &teleplot;

    void NormalizePodState(double &leftOutput, double &rightOutput, double currentAngle);
    void OptimizePodState(PodState &targetState, PodState currentState);
    void LimitPodState(PodState &targetState);
    double GetLeftSpeed();
    double GetRightSpeed();
    double GetAngle();
};
