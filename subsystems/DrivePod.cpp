#include "ctre/phoenix6/TalonFXS.hpp"
#include "ctre/phoenix6/CANcoder.hpp"

#include "../utils/math/pid.h"
#include "../utils/telemetry/Teleplot.h"
#include <cmath>
#include "../utils/swerve/PodState.h"
#include "../Constants.h"

using namespace ctre::phoenix6;
using namespace std;

class DrivePod {
private: 
    double deadZone = 55.0 / 180.0 * 3.14159; // 75 degrees in radians of absolute deadzone at directly backwards
    double encoderOffset = -0.2073; // rotations
    PID myPID{0.02, 1.0, -1.0, 1.2, 0.0, 0.0};
    /* This can be a CANivore name, CANivore serial number,
     * SocketCAN interface, or "*" to select any CANivore. */
    static constexpr char const *CANBUS_NAME = "*";
    double targetAngle = 0.0;
    bool backwards = false;

    PodState currentState{0.0, 0.0}; // Current state of the pod

    /* devices */
    hardware::TalonFXS left;
    hardware::TalonFXS right;

    hardware::CANcoder encoder;

    /* control requests */
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};

    Teleplot teleplot = Teleplot("127.0.0.1", 47269);


    void NormalizePodState(double &leftOutput, double &rightOutput, double currentAngle) {
        double podMaxOutput = max(abs(leftOutput), abs(rightOutput));
        if (podMaxOutput > 1) {
            double podOutputScalar = 1 / podMaxOutput;

            leftOutput *= podOutputScalar;
            leftOutput *= podOutputScalar;
        }
    }

    // modifies the target angle and output to enable shortspin
    void OptimizePodState(PodState &targetState, PodState currentState) {
        double angleDifference = targetState.Angle - currentState.Angle;
        while (angleDifference > M_PI * 0.5) {
            targetState.Angle -= M_PI;
            targetState.flip();    
        }
        while (angleDifference < -M_PI * 0.5) {
            targetState.Angle += M_PI;
            targetState.flip();
        }
    }

    // Limits the target angle to within the operating zone
    void LimitPodState(PodState &targetState) {
        // Limit the target angle to within the operating zone
        double positiveLimit = M_PI - deadZone; // 180 degrees in radians minus deadzone
        double negativeLimit = -M_PI + deadZone; // -180 degrees in radians plus deadzone
        while (targetState.Angle > positiveLimit) {
            targetState.Angle -= M_PI; 
            targetState.flip(); // flip direction
        } 
        while (targetState.Angle < negativeLimit) {
            targetState.Angle += M_PI; 
            targetState.flip(); // flip direction
        }
    }

    void stopWhileTurning(PodState &targetState, PodState currentState) {
        bool isMoving = currentState.PodSpeed > Constants::DrivetrainConstants::isMovingThreshold;
        double error = abs(targetState.Angle - currentState.Angle);
        bool isTurning = isMoving ? error > Constants::DrivetrainConstants::stopWhileTurning : error > Constants::DrivetrainConstants::moveAfterTurning;
        if (isTurning) {
            if (isMoving) {
                targetState.PodSpeed = 0.0; // stop moving if turning
            } else {
                targetState.PodSpeed = 0.0;
            }
        }
    }

    double getLeftSpeed() {
        return left.GetRotorVelocity().GetValueAsDouble();
    }

    double getRightSpeed() {
        return right.GetRotorVelocity().GetValueAsDouble();
    }

    double getAngle() {
        return encoder.GetPosition().GetValueAsDouble() - encoderOffset; // rotations
    }
    
    

public:
    DrivePod(Constants::PodConfig &config) 
        : left(config.leftMotorID, CANBUS_NAME),
          right(config.rightMotorID, CANBUS_NAME),
          encoder(config.encoderID, CANBUS_NAME) {
        // Set motor configurations
        
        
    }

    void Periodic() {
        // update current state
        currentState = PodState(getLeftSpeed() + getRightSpeed() / 2.0, getAngle());

        // apply motor outputs
        left.SetControl(leftOut);
        right.SetControl(rightOut);

        teleplot.update("encoder", getAngle(), "rotations");
        teleplot.update("targetAngle", targetAngle, "rad");
        teleplot.update("left rotor velocity", left.GetRotorVelocity().GetValueAsDouble(), ("rotations/s"));
    }

    void SetPodState(PodState &targetState) {
        OptimizePodState(targetState, currentState);
        LimitPodState(targetState);

        double turnSpeed = myPID.calculate(targetState.Angle, currentState.Angle);

        // Check if the pod is moving
        bool isMoving = currentState.PodSpeed > Constants::DrivetrainConstants::isMovingThreshold;
        double error = abs(targetAngle - currentState.Angle);
        bool isTurning = isMoving ? error > Constants::DrivetrainConstants::stopWhileTurning : error > Constants::DrivetrainConstants::moveAfterTurning;
        if (isTurning) {
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

    PodState GetPodState() const {
        return currentState;
    }
};
