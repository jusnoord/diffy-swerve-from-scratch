
#include "RobotBase.hpp"
#include "utils/controller/Joystick.hpp"
#include "utils/math/pid.h"
#include "utils/telemetry/Teleplot.h"
#include <cmath>
#include "subsystems/diffySwerve.h"
#include "utils/math/pose2d.h"

using namespace std;
#include <chrono>
/**
 * This is the main robot. Put all actuators, sensors,
 * game controllers, etc. in this class.
 */

// Returns milliseconds since program start
uint32_t millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

class Robot : public RobotBase {
private:    
    /* joystick */
    Joystick joy{0};
    diffySwerve swerve{};

public:
    /* main robot interface */
    void RobotInit() override;
    void RobotPeriodic() override;

    bool IsEnabled() override;
    void EnabledInit() override;
    void EnabledPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;
};

/**
 * Runs once at code initialization.
 */
void Robot::RobotInit()
{
    // configs::TalonFXConfiguration fx_cfg{};

    // /* the left motor is CCW+ */
    // fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    // left.GetConfigurator().Apply(fx_cfg);

    // /* the right motor is CW+ */
    // fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    // right.GetConfigurator().Apply(fx_cfg);

    // /* set follower motors to follow leaders; do NOT oppose the leaders' inverts */
    // leftFollower.SetControl(controls::Follower{left.GetDeviceID(), false});
    // rightFollower.SetControl(controls::Follower{right.GetDeviceID(), false});
}

/**
 * Runs periodically during program execution.
 */
void Robot::RobotPeriodic()
{
    /* periodically check that the joystick is still good */
    joy.Periodic();

    swerve.Periodic();

}

/**
 * Returns whether the robot should be enabled.
 */
bool Robot::IsEnabled()
{
    bool currentButtonState = joy.GetButton(5); // Right bumper
    return currentButtonState; // For testing, always return true when the button is pressed
}

/**
 * Runs when transitioning from disabled to enabled.
 */
void Robot::EnabledInit() {}

/**
 * Runs periodically while enabled.
 */
void Robot::EnabledPeriodic()
{

    pose2d speed = pose2d(joy.GetAxis(0), joy.GetAxis(1), joy.GetAxis(2));
    swerve.SetRobotSpeed(speed);
}

/**
 * Runs when transitioning from enabled to disabled,
 * including after robot startup.
 */
void Robot::DisabledInit() {}

/**
 * Runs periodically while disabled.
 */
void Robot::DisabledPeriodic()
{
}

// double getEncoder()
// {
//     return encoder.GetPosition().GetValueAsDouble() - encoderOffset;
// }

/* ------ main function ------ */
int main()
{
    /* create and run robot */
    Robot robot{};
    // robot.SetLoopTime(20_ms); // optionally change loop time for periodic calls
    return robot.Run();
}

