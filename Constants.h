class Constants {
public:
       static class RobotMap{}

       static class DrivetrainConstants {
            static constexpr double isMovingThreshold = 4.0; // threshold for determining if the robot is moving
            static constexpr double stopWhileTurning = 1.57; // threshold for determining if the robot is turning enough to stop moving first
            static constexpr double moveAfterTurning = 0.1; // threshold for determining if the robot is turning enough to start moving again after stopping
       };

};