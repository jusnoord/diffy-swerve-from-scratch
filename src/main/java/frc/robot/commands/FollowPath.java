// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConfig;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Path;
import frc.robot.util.TunableNumber;

/**drives two robots in tandem */
public class FollowPath extends Command {
    private final Swerve swerve;

    private Path path;

    private StructEntry<Pose2d> targetPosePublisher;
    private StructSubscriber<Pose2d> masterPoseSubscriber;



    //PID values
    private TunableNumber kP = new TunableNumber("tandem kP", RobotConstants.tandemkP);
    private TunableNumber kI = new TunableNumber("tandem kI", RobotConstants.tandemkI);
    private TunableNumber kD = new TunableNumber("tandem kD", RobotConstants.tandemkD);
    private TunableNumber kP_angle = new TunableNumber("tandem kP_angle", RobotConstants.tandemkP_angle);
    private TunableNumber kI_angle = new TunableNumber("tandem kI_angle", RobotConstants.tandemkI_angle);
    private TunableNumber kD_angle = new TunableNumber("tandem kD_angle", RobotConstants.tandemkD_angle);

    private final PIDController anglePIDRobot = new PIDController(kP_angle.getDefault(), kI_angle.getDefault(), kD_angle.getDefault());

    private final PIDController anglePID = new PIDController(kP_angle.getDefault(), kI_angle.getDefault(), kD_angle.getDefault());
    private final PIDController xPID = new PIDController(kP.getDefault(), kI.getDefault(), kD.getDefault());
    private final PIDController yPID = new PIDController(kP.getDefault(), kI.getDefault(), kD.getDefault());

    public FollowPath(Swerve swerve, Path path) {
        this.swerve = swerve;
        this.path = path;
        masterPoseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.RobotType.master.toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        anglePID.enableContinuousInput(0, Math.PI * 2);
        anglePID.setTolerance(0.02);
        xPID.setTolerance(0.01);
        yPID.setTolerance(0.01);

        
        if(Constants.IS_MASTER) {
            //reset the initial pose to the robot-system pose
            // only do this if on the master robot, as slaves are synced to master automatically
            // Pose2d robotTargetPose = targetPose.plus(new Transform2d(RobotConfig.offsetPositions[Constants.IS_MASTER ? 0 : 1], Rotation2d.kZero));
            // swerve.resetPose(new Pose2d(5, 8, new Rotation2d()));
        }

        //initialize telemetry
        targetPosePublisher = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.toString()).getStructTopic("targetPose", Pose2d.struct).getEntry(new Pose2d());
    }

    @Override
    public void execute() {
        Pose2d offsetPosition = RobotConfig.offsetPositions[Constants.IS_MASTER ? 0 : 1];
        Pose2d currentPose = swerve.getPose(); // replace with getting the formation ppose
        Pose2d masterPose = Constants.IS_MASTER ? currentPose : masterPoseSubscriber.get();
        Pose2d robotVelocity = path.getVelocity(currentPose, anglePIDRobot); // add formation offset
        Pose2d robotTargetPose = masterPose.plus(RobotConfig.offsetPositions[1].minus(RobotConfig.offsetPositions[0]));

        double xOut = robotVelocity.getX() + xPID.calculate(currentPose.getX(), robotTargetPose.getX());
        double yOut = robotVelocity.getY() + yPID.calculate(currentPose.getY(), robotTargetPose.getY());
        double rOut = robotVelocity.getRotation().getRadians() + anglePID.calculate(currentPose.getRotation().getRadians(), robotTargetPose.getRotation().getRadians());
        
        boolean PIDAtTolerance = anglePID.atSetpoint() && xPID.atSetpoint() && yPID.atSetpoint();
        if (!Constants.IS_MASTER && !PIDAtTolerance) {
            swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rOut, currentPose.getRotation()));
        } else {
            swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(robotVelocity.getX(), robotVelocity.getY(), robotVelocity.getRotation().getRadians(), currentPose.getRotation()));
        }

        
        // update the PID values from the tunable numbers
        if(Constants.tuningMode) {
            anglePID.setPID(kP_angle.doubleValue(), kI_angle.doubleValue(), kD_angle.doubleValue());
            xPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
            yPID.setPID(kP.doubleValue(), kI.doubleValue(), kD.doubleValue());
        }

        //update telemetry
        targetPosePublisher.accept(robotTargetPose);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
