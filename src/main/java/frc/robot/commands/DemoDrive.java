
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DemoConstants;
import frc.robot.subsystems.InputGetter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Path;
import frc.robot.util.WingPoseEstimator;

public class DemoDrive extends SequentialCommandGroup {
    /** Creates a new DemoDrive. */
    public DemoDrive(Swerve swerve, WingPoseEstimator wingPoseEstimator, InputGetter inputGetter) {
        Pose2d wingApproximate = Constants.IS_MASTER ? DemoConstants.masterWingApproximate : DemoConstants.slaveWingApproximate;
        addCommands(
            new AutoDrive(swerve, () -> wingApproximate, true), // follow path to approximate wing position
            new AutoDrive(swerve, wingPoseEstimator::getEstimatedPose, false), // PID to exact wing position
            new SyncOffsets(swerve).withTimeout(1), // sync offsets TODO: 1 second is random
            new TandemDrive(swerve, inputGetter::getJoystickVelocity).until(inputGetter::getRightBumper) // tandem drive with other robot manually
            
            // 
        );
    }
}
