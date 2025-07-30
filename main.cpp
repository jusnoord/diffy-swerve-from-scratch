#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANCoder.hpp"
#include "RobotBase.hpp"
#include "Joystick.hpp"
#include <cmath>
#include "pid.h"

using namespace ctre::phoenix6;

/**
 * This is the main robot. Put all actuators, sensors,
 * game controllers, etc. in this class.
 */
class Robot : public RobotBase {
private:
    double deadZone = 75.0 / 180.0 * 3.14159; // 75 degrees in radians of absolute deadzone at directly backwards
    double encoderOffset = 0.0 * 2.0 * 3.14159; // 0 degrees in radians
    PID myPID = new PID(0.02, 1.0, -1.0, 0.1, 0.01, 0.01);
    /* This can be a CANivore name, CANivore serial number,
     * SocketCAN interface, or "*" to select any CANivore. */
    static constexpr char const *CANBUS_NAME = "*";

    /* devices */
    hardware::TalonFX left{13, CANBUS_NAME};
    hardware::TalonFX right{14, CANBUS_NAME};

    hardware::CANCoder encoder{22, CANBUS_NAME};

    /* control requests */
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};

    /* joystick */
    Joystick joy{0};

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
    // joy.Periodic();
}

/**
 * Returns whether the robot should be enabled.
 */
bool Robot::IsEnabled()
{
    /* enable while joystick is an Xbox controller (6 axes),
     * and we are holding the right bumper */
    // if (joy.GetNumAxes() < 6) return false;
    // return joy.GetButton(5); // SDL_CONTROLLER_BUTTON_RIGHTSHOULDER

    return true;
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
    /* arcade drive */
    // double speed = -joy.GetAxis(1); // SDL_CONTROLLER_AXIS_LEFTY
    // double turn = joy.GetAxis(4); // SDL_CONTROLLER_AXIS_RIGHTX
    double targetAngle = std::atan2(joy.GetAxis(1),joy.GetAxis(2)); //radians -pi to pi
    double currentAngle = (encoder::GetPosition()::GetValue() * 2.0 * 3.14159) - encoderOffset; // radians 0 to 2pi
    bool backwards = false;

    // optimize shortspin
    while (targetAngle - currentAngle > 3.14159 * 0.5) {
        targetAngle -= 3.14159;
        backwards = !backwards;
    }
    while (targetAngle - currentAngle < -3.14159 * 0.5) {
        targetAngle += 3.14159;
        backwards = !backwards;
    }

    // limit the target angle to within the operating zone
    double positiveLimit = 3.14159 - deadZone; // 180 degrees in radians minus deadzone
    double negativeLimit = -3.14159 + deadZone; // -180 degrees in radians plus deadzone
    if (targetAngle > positiveLimit) {
        targetAngle -= 3.14159; 
        backwards = !backwards; // flip direction
    } else if (targetAngle < negativeLimit) {
        targetAngle += 3.14159; 
        backwards = !backwards; // flip direction
    }


    double speed = joy.GetAxis(4) * (backwards ? -1 : 1);
    double turnSpeed = myPID.calculate(targetAngle, currentAngle);

    leftOut.Output = speed - turnSpeed;
    rightOut.Output = speed + turnSpeed;

    left.SetControl(leftOut);
    right.SetControl(rightOut);
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
    left.SetControl(controls::NeutralOut{});
    // right.SetControl(controls::NeutralOut{});
}

/* ------ main function ------ */
int main()
{
    /* create and run robot */
    Robot robot{};
    // robot.SetLoopTime(20_ms); // optionally change loop time for periodic calls
    return robot.Run();
}
