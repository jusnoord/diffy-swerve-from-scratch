#pragma once
#include "utils/math/Translation2d.h"

class Constants
{
public:
    class DrivetrainConstants
    {
    public:
        static constexpr double isMovingThreshold = 4.0; // threshold for determining if the robot is moving
        static constexpr double stopWhileTurning = 1.57; // threshold for determining if the robot is turning enough to stop moving first
        static constexpr double moveAfterTurning = 0.1;  // threshold for determining if the robot is turning enough to start moving again after stopping

        static constexpr double deadZone = 55.0 / 180.0 * 3.14159;

        static constexpr double turnkP = 1.2;
        static constexpr double turnkI = 0.0;
        static constexpr double turnkD = 0.0;
    };

    /**
     * @brief Configuration class for a drive pod.
     *
     * PodConfig encapsulates all necessary configuration parameters for a drive pod,
     * including motor IDs, encoder settings, and physical position.
     *
     * Members:
     * - leftMotorID: ID of the left motor.
     * - rightMotorID: ID of the right motor.
     * - encoderID: ID of the encoder.
     * - encoderOffset: Offset value for the encoder.
     * - position: Physical position of the pod (Translation2d).
     *
     * Static Constants:
     * - encoderInvert: Indicates if the encoder direction is inverted.
     * - leftMotorInvert: Indicates if the left motor direction is inverted.
     * - rightMotorInvert: Indicates if the right motor direction is inverted.
     * - motorGearing: Gear ratio of the motors.
     * - motorsBrake: Whether motors are set to brake mode.
     * - ampLimit: Current limit for the motors (in Amps).
     * - maxOutput: Maximum output value for the motors.
     * - rampRate: Rate at which motor output ramps up.
     *
     * @param leftMotorID ID of the left motor.
     * @param rightMotorID ID of the right motor.
     * @param encoderID ID of the encoder.
     * @param encoderOffset Offset value for the encoder.
     * @param position Physical position of the pod.
     */
    class PodConfig
    {
    public:
        int leftMotorID;
        int rightMotorID;
        int encoderID;
        double encoderOffset;
        Translation2d position{};
        PodConfig(int leftMotorID, int rightMotorID, int encoderID, double encoderOffset, Translation2d position)
            : leftMotorID(leftMotorID), rightMotorID(rightMotorID), encoderID(encoderID), encoderOffset(encoderOffset), position(position) {}

        static constexpr bool encoderInvert = true;
        static constexpr bool leftMotorInvert = false;
        static constexpr bool rightMotorInvert = true;

        static constexpr double motorGearing = 50.0;

        static constexpr bool motorsBrake = true;
        static constexpr int ampLimit = 80;
        static constexpr double maxOutput = 1.0;
        static constexpr double rampRate = 0.2;
    };

    class RobotMap
    {
    public:
        static constexpr char const *CANBUS_NAME = "*";
        static constexpr int PIGEON_ID = 25;

        inline static const PodConfig podConfig1{1, 2, 3, -0.2073, Translation2d(-1.0, 0.0)};
        inline static const PodConfig podConfig2{4, 5, 6, -0.2073, Translation2d(1.0, 0.0)};
    };
};