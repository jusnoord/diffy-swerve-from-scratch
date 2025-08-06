#include "NerdPod.h"
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <units/angle.h>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/dimensionless.h>

NerdPod::NerdPod(const Constants::PodConfig &config, double &globalOutputScalar, const bool &isTurningSupplier, Teleplot &teleplot)
    :
      position(config.position),
      globalOutputScalar(globalOutputScalar),
      otherPodTurning(isTurningSupplier),
      drive(config.leftMotorID, CANBUS_NAME),
      azimuth(config.rightMotorID, CANBUS_NAME),
      encoder(config.encoderID, CANBUS_NAME),
      encoderOffset(config.encoderOffset),
      teleplot(teleplot)
{
    configs::TalonFXConfiguration driveConfig;
    configs::TalonFXConfiguration azimuthConfig;
    configs::CANcoderConfiguration coderConfig;
    
    driveConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    //set current limits; supply current limits are hardcoded because they are almost always the same
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = units::ampere_t(40);

    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = units::ampere_t(80);
    
    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = units::second_t(0.2);
    driveConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = units::second_t(0.2);
    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = units::second_t(0.2); 
    driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = units::second_t(0.2);
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = units::second_t(0.2);
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = units::second_t(0.2);
    
    driveConfig.Feedback.SensorToMechanismRatio =  1/(0.0508 * M_PI / 1.36); //Wheel Diameter M * PI / Enc Count Per Rev / Gear Ratio
    driveConfig.Feedback.RotorToSensorRatio = 1;
    
    drive.GetConfigurator().Apply(driveConfig);
    
    
    azimuthConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    azimuthConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    
    //set current limits; supply current limits are hardcoded because they are almost always the same
    azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    azimuthConfig.CurrentLimits.SupplyCurrentLimit = units::ampere_t(40);
    
    azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    azimuthConfig.CurrentLimits.StatorCurrentLimit = units::ampere_t(80); //TODO: finish
        
    azimuthConfig.Feedback.SensorToMechanismRatio = 1.0;
    azimuthConfig.Feedback.RotorToSensorRatio =  2.200000047683716;

    azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;
    azimuthConfig.Feedback.FeedbackRemoteSensorID = config.encoderID;
    azimuthConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
    
    azimuthConfig.Slot0.kP = 1.2;
    azimuthConfig.Slot0.kI = 0.02;
    azimuthConfig.Slot0.kD = 0.001;

    azimuth.GetConfigurator().Apply(azimuthConfig); 
    
    coderConfig.MagnetSensor.SensorDirection = true;
    coderConfig.MagnetSensor.MagnetOffset = units::turn_t(encoderOffset);

    //not applying magnet config directly in order to overwrite other settings
    encoder.GetConfigurator().Apply(coderConfig);
}

Translation2d NerdPod::GetPosition() const
{
    return position;
}

void NerdPod::Initialize()
{
    // Initialization logic if needed
}

PodState NerdPod::GetPodState() const
{
    return currentState;
}

bool NerdPod::isPodTurning() const
{
    return isTurning;
}

void NerdPod::LimitPodState(PodState &targetState)
{
    double positiveLimit = M_PI - Constants::DrivetrainConstants::deadZone;
    double negativeLimit = -M_PI + Constants::DrivetrainConstants::deadZone;

    while (targetState.Angle > positiveLimit)
    {
        targetState.Angle -= M_PI;
        targetState.flip();
    }
    while (targetState.Angle < negativeLimit)
    {
        targetState.Angle += M_PI;
        targetState.flip();
    }
}
void NerdPod::Periodic()
{
    // update current state
    currentState = PodState(GetSpeed(), GetAngle());

    // apply motor outputs
    drive.SetControl(driveOut);
    azimuth.SetControl(azimuthOut);

    teleplot.update("encoder", GetAngle(), "rotations");
    // If targetAngle is needed, ensure it's defined and updated appropriately
    // teleplot.update("targetAngle", targetAngle, "rad");
    teleplot.update("drive rotor velocity", drive.GetRotorVelocity().GetValueAsDouble(), "rotations/s");
}

void NerdPod::SetPodState(PodState &targetState)
{
    OptimizePodState(targetState, currentState);
    LimitPodState(targetState);


    // Check if the pod is moving
    bool isMoving = currentState.PodSpeed > Constants::DrivetrainConstants::isMovingThreshold;
    double error = abs(targetState.Angle - currentState.Angle);
    isTurning = isMoving ? error > Constants::DrivetrainConstants::stopWhileTurning : error > Constants::DrivetrainConstants::moveAfterTurning;

    // If this or any other pod is turning, stop moving first
    if (isTurning || otherPodTurning)
    {
        if (isMoving)
        {
            targetState.PodSpeed = 0.0; // stop moving if turning
            targetState.Angle = currentState.Angle;
        }
        else
        {
            targetState.PodSpeed = 0.0;
        }
    }

    double driveOutput = targetState.PodSpeed;
    units::radian_t azimuthOutput = units::radian_t(targetState.Angle);

    NormalizePodState(driveOutput, currentState.Angle);

    driveOut.Output = driveOutput;
    azimuthOut.WithPosition(azimuthOutput);

    teleplot.update("turnSpeed", targetState.Angle, "%");
    teleplot.update("speed", targetState.PodSpeed, "%");
}

void NerdPod::OptimizePodState(PodState &targetState, PodState currentState)
{
    double angleDifference = targetState.Angle - currentState.Angle;
    while (angleDifference > M_PI * 0.5)
    {
        targetState.Angle -= M_PI;
        targetState.flip();
        angleDifference = targetState.Angle - currentState.Angle;
    }
    while (angleDifference < -M_PI * 0.5)
    {
        targetState.Angle += M_PI;
        targetState.flip();
        angleDifference = targetState.Angle - currentState.Angle;
    }
}

void NerdPod::NormalizePodState(double &output, double currentAngle)
{

    // If the maximum output exceeds the maximum allowed output, scale down both outputs
    if (output > 1 || globalOutputScalar < 1)
    {
        double podOutputScalar = 1 / output;

        // If our max output is 2.0, we need a scaling of 0.5.
        // The global scalar should be the minimum of all required scalings.
        if (podOutputScalar < globalOutputScalar)
        {
            globalOutputScalar = podOutputScalar;
        }

        // scale by the largest normalization factor
        output *= globalOutputScalar;
    }
}

// Private methods

double NerdPod::GetSpeed()
{
    return drive.GetRotorVelocity().GetValueAsDouble();
}

double NerdPod::GetAngle()
{
    return encoder.GetPosition().GetValueAsDouble() - encoderOffset;
}