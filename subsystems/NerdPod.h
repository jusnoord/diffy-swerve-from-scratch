#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANcoder.hpp"


#include "../utils/telemetry/Teleplot.h"
#include <cmath>
#include "../utils/swerve/PodState.h"
#include "../utils/math/Translation2d.h"
#include "../Constants.h"
#include "units/angle.h"

using namespace units::angle;
using namespace ctre::phoenix6;

class NerdPod {
    public:
        NerdPod(const Constants::PodConfig &config, double &globalOutputScalar, const bool &isTurningSupplier, Teleplot &teleplot);
        Translation2d GetPosition() const;
        void Initialize();
        void Periodic();
        void SetPodState(PodState &targetState);
        PodState GetPodState() const;
        bool isPodTurning() const;

    private:
        Translation2d position{0.0, 0.0};
        double encoderOffset;
        double &globalOutputScalar;
        const bool &otherPodTurning;
        bool isTurning = false;
        PodState currentState{0.0, 0.0};

        static constexpr char const *CANBUS_NAME = "*";
        hardware::TalonFX drive;
        hardware::TalonFX azimuth;
        hardware::CANcoder encoder;
        controls::DutyCycleOut driveOut{0};
        controls::PositionDutyCycle azimuthOut{0_deg};
        Teleplot &teleplot;

        void NormalizePodState(double &driveOutput, double currentAngle);
        void OptimizePodState(PodState &targetState, PodState currentState);
        void LimitPodState(PodState &targetState);
        double GetSpeed();
        double GetAngle();
};
