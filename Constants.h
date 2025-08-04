class Constants {
public:
       class RobotMap{};

       class DrivetrainConstants {
        public:
            static constexpr double isMovingThreshold = 4.0; // threshold for determining if the robot is moving
            static constexpr double stopWhileTurning = 1.57; // threshold for determining if the robot is turning enough to stop moving first
            static constexpr double moveAfterTurning = 0.1; // threshold for determining if the robot is turning enough to start moving again after stopping
       };

       class PodConfig {
        public:
            int leftMotorID;
            int rightMotorID;
            int encoderID;
            double encoderOffset;
            PodConfig(int leftMotorID, int rightMotorID, int encoderID, double encoderOffset)
                : leftMotorID(leftMotorID), rightMotorID(rightMotorID), encoderID(encoderID), encoderOffset(encoderOffset) {}

            static constexpr bool encoderInvert = true;
            static constexpr bool leftMotorInvert = false;
            static constexpr bool rightMotorInvert = true;

            static constexpr double motorGearing = 50.0;

            static constexpr bool motorsBrake = true;
            static constexpr int ampLimit = 80;
            static constexpr double maxOutput = 1.0;
            static constexpr double rampRate = 0.2;
       };

};