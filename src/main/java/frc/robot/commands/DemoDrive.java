
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.DemoConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.InputGetter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Path;
import frc.robot.util.WingPoseEstimator;

public class DemoDrive extends SequentialCommandGroup {
    BooleanSubscriber isOtherRobotFinished;
    /** Creates a new DemoDrive. */
    public DemoDrive(Swerve swerve, WingPoseEstimator wingPoseEstimator, InputGetter inputGetter) {
        isOtherRobotFinished = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.getOpposite().toString()).getBooleanTopic("autodrive at target").subscribe(false);

        Pose2d wingApproximate = Constants.IS_MASTER ? DemoConstants.masterWingApproximate : DemoConstants.slaveWingApproximate;
        addCommands(
            new AutoDrive(swerve, () -> wingApproximate, true), // follow path to approximate wing position
            new AutoDrive(swerve, wingPoseEstimator::getEstimatedPose, false), // PID to exact wing position
            new WaitUntilCommand(isOtherRobotFinished::get), // wait for other robot to finish
            // new WaitCommand(5), // wait 5 seconds for demo purposes
            new SyncOffsets(swerve).withTimeout(1), // sync offsets TODO: 1 second is random
            // new TandemDrive(swerve, inputGetter::getJoystickVelocity).until(inputGetter::getRightBumper), // tandem drive with other robot manually
            new FollowPath(swerve, new Path(PathConstants.wayPoints, PathConstants.defaultSpeed, PathConstants.lookAhead, PathConstants.rotationalLookAhead), DemoConstants.stationPosition) // follow path back to start
        );
    }
}
