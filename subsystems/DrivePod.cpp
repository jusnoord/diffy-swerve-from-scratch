#include "ctre/phoenix6/TalonFXS.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

#include "../utils/math/pid.h"
#include "../utils/telemetry/Teleplot.h"
#include <cmath>
#include "../utils/swerve/PodState.h"
#include "../Constants.h"
#include "../utils/math/translation2d.h"

#include "DrivePod.h"

using namespace ctre::phoenix6;
using namespace std;

DrivePod::DrivePod(Constants::PodConfig &config, double &globalOutputScalar, const bool &isTurningSupplier)
    : position(config.position),
      globalOutputScalar(globalOutputScalar),
      otherPodTurning(isTurningSupplier),
      left(config.leftMotorID, CANBUS_NAME),
      right(config.rightMotorID, CANBUS_NAME),
      encoder(config.encoderID, CANBUS_NAME) {
    // TODO: Set motor configurations
}

translation2d DrivePod::GetPosition() const {
    return position;
}

void DrivePod::Initialize() {
    // Initialization logic if needed
}

void DrivePod::Periodic() {
    // update current state
    currentState = PodState((getLeftSpeed() + getRightSpeed()) / 2.0, getAngle());

    // apply motor outputs
    left.SetControl(leftOut);
    right.SetControl(rightOut);

    teleplot.update("encoder", getAngle(), "rotations");
    // If targetAngle is needed, ensure it's defined and updated appropriately
    // teleplot.update("targetAngle", targetAngle, "rad");
    teleplot.update("left rotor velocity", left.GetRotorVelocity().GetValueAsDouble(), "rotations/s");
}

void DrivePod::SetPodState(PodState &targetState) {
    OptimizePodState(targetState, currentState);
    LimitPodState(targetState);

    double turnSpeed = myPID.calculate(targetState.Angle, currentState.Angle);

    // Check if the pod is moving
    bool isMoving = currentState.PodSpeed > Constants::DrivetrainConstants::isMovingThreshold;
    double error = abs(targetState.Angle - currentState.Angle);
    isTurning = isMoving ? error > Constants::DrivetrainConstants::stopWhileTurning : error > Constants::DrivetrainConstants::moveAfterTurning;

    // If this or any other pod is turning, stop moving first
    if (isTurning || otherPodTurning) {
        if (isMoving) {
            targetState.PodSpeed = 0.0; // stop moving if turning
            turnSpeed = 0.0;
        } else {
            targetState.PodSpeed = 0.0;
        }
    }

    double leftOutput = targetState.PodSpeed - turnSpeed;
    double rightOutput = -(targetState.PodSpeed + turnSpeed);

    NormalizePodState(leftOutput, rightOutput, currentState.Angle);

    leftOut.Output = leftOutput;
    rightOut.Output = rightOutput;

    teleplot.update("turnSpeed", turnSpeed, "%");
    teleplot.update("speed", targetState.PodSpeed, "%");
}

PodState DrivePod::GetPodState() const {
    return currentState;
}

bool DrivePod::isPodTurning() const {
    return isTurning;
}


// Private methods

void DrivePod::NormalizePodState(double &leftOutput, double &rightOutput, double currentAngle) {
    double podMaxOutput = max(abs(leftOutput), abs(rightOutput));


    if (podMaxOutput > 1) {
        double podOutputScalar = 1 / podMaxOutput;

        if(podOutputScalar > globalOutputScalar) {
            globalOutputScalar = podOutputScalar;
        }
        
        //scale by the largest normalization factor
        leftOutput *= globalOutputScalar;
        rightOutput *= globalOutputScalar;
    }
}

void DrivePod::OptimizePodState(PodState &targetState, PodState currentState) {
    double angleDifference = targetState.Angle - currentState.Angle;
    while (angleDifference > M_PI * 0.5) {
        targetState.Angle -= M_PI;
        targetState.flip();
        angleDifference = targetState.Angle - currentState.Angle;
    }
    while (angleDifference < -M_PI * 0.5) {
        targetState.Angle += M_PI;
        targetState.flip();
        angleDifference = targetState.Angle - currentState.Angle;
    }
}

void DrivePod::LimitPodState(PodState &targetState) {
    double positiveLimit = M_PI - deadZone;
    double negativeLimit = -M_PI + deadZone;
    while (targetState.Angle > positiveLimit) {
        targetState.Angle -= M_PI;
        targetState.flip();
    }
    while (targetState.Angle < negativeLimit) {
        targetState.Angle += M_PI;
        targetState.flip();
    }
}

void DrivePod::stopWhileTurning(PodState &targetState, PodState currentState) {
    bool isMoving = currentState.PodSpeed > Constants::DrivetrainConstants::isMovingThreshold;
    double error = abs(targetState.Angle - currentState.Angle);
    bool isTurning = isMoving ? error > Constants::DrivetrainConstants::stopWhileTurning : error > Constants::DrivetrainConstants::moveAfterTurning;
    if (isTurning) {
        targetState.PodSpeed = 0.0;
    }
}

double DrivePod::getLeftSpeed() {
    return left.GetRotorVelocity().GetValueAsDouble();
}

double DrivePod::getRightSpeed() {
    return right.GetRotorVelocity().GetValueAsDouble();
}

double DrivePod::getAngle() {
    return encoder.GetPosition().GetValueAsDouble() - encoderOffset;
}
