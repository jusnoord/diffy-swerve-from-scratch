
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.DemoConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.RobotConfig;
import frc.robot.subsystems.InputGetter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Path;
import frc.robot.util.WingPoseEstimator;

public class DemoDrive extends SequentialCommandGroup {
    BooleanSubscriber isOtherRobotFinished;
    

    Supplier<Integer> wingApproximateToUse;

    /** Creates a new DemoDrive. */
    public DemoDrive(Swerve swerve, WingPoseEstimator wingPoseEstimator, InputGetter inputGetter) {
        isOtherRobotFinished = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.getOpposite().toString()).getBooleanTopic("autodrive at target").subscribe(false);
        
        // have the master determine which wing position is closer
        if(Constants.IS_MASTER) {
            BooleanPublisher useMasterTagPublisher = NetworkTableInstance.getDefault().getTable("DemoMode").getBooleanTopic("use master tag").publish();
            wingApproximateToUse = () -> {
            if(Math.abs(swerve.getPose().minus(DemoConstants.wingApproximates[0]).getTranslation().getNorm()) < (Math.abs(swerve.getPose().minus(DemoConstants.wingApproximates[1]).getTranslation().getNorm()))) {
                useMasterTagPublisher.accept(false);
                return 0;
            } else {
                useMasterTagPublisher.accept(true);
                return 1;
            }};
        } else {
            BooleanSubscriber useMasterTagSubscriber = NetworkTableInstance.getDefault().getTable("DemoMode").getBooleanTopic("use master tag").subscribe(true);
            wingApproximateToUse = () -> useMasterTagSubscriber.get() ? 0 : 1;
        }

        
        addCommands(
            new AutoDrive(swerve, () -> DemoConstants.wingApproximates[wingApproximateToUse.get()], true), // follow path to approximate wing position
            new AutoDrive(swerve, () -> wingPoseEstimator.getEstimatedPose().plus(DemoConstants.wingRelativeFormationOffsets[wingApproximateToUse.get()]), false), // PID to exact wing position
            new WaitUntilCommand(isOtherRobotFinished::get), // wait for other robot to finish
            // new WaitCommand(5), // wait 5 seconds for demo purposes
            // new SyncOffsets(swerve).withTimeout(1), // sync offsets TODO: 1 second is random
            new InstantCommand(RobotConfig::resetOffsetPositions),
            // new TandemDrive(swerve, inputGetter::getJoystickVelocity).until(inputGetter::getRightBumper), // tandem drive with other robot manually
            new FollowPath(swerve, new Path(PathConstants.wayPoints, PathConstants.defaultSpeed, PathConstants.lookAhead, PathConstants.rotationalLookAhead), DemoConstants.stationPosition) // follow path back to start
        );
    }
}
