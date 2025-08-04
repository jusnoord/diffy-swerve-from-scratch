#pragma once

#include "ctre/phoenix6/TalonFXS.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

#include "../utils/math/pid.h"
#include "../utils/telemetry/Teleplot.h"
#include <cmath>
#include "../utils/swerve/PodState.h"
#include "../utils/math/translation2d.h"
#include "../Constants.h"

using namespace ctre::phoenix6;
using namespace std;

class DrivePod {
public:
    translation2d GetPosition() const;
    void Initialize() ;
    void Periodic();
    void SetPodState(PodState &targetState);
    PodState GetPodState() const;
    DrivePod(Constants::PodConfig &config, double &globalOutputScalar, const bool &isTurningSupplier);
    bool isPodTurning() const;

private:
    translation2d position{0.0, 0.0};
    double deadZone = 55.0 / 180.0 * 3.14159;
    double encoderOffset = -0.2073;
    PID myPID{0.02, 1.0, -1.0, 1.2, 0.0, 0.0};
    static constexpr char const *CANBUS_NAME = "*";
    double &globalOutputScalar;
    const bool &otherPodTurning;
    bool isTurning = false;
    PodState currentState{0.0, 0.0};
    hardware::TalonFXS left;
    hardware::TalonFXS right;
    hardware::CANcoder encoder;
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};
    Teleplot teleplot = Teleplot("127.0.0.1", 47269);

    void NormalizePodState(double &leftOutput, double &rightOutput, double currentAngle);
    void OptimizePodState(PodState &targetState, PodState currentState);
    void LimitPodState(PodState &targetState);
    void stopWhileTurning(PodState &targetState, PodState currentState);
    double getLeftSpeed();
    double getRightSpeed();
    double getAngle();
};
