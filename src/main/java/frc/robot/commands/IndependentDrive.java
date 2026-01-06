// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotConfig;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

/**
 * drives a given robot independently
 */
public class IndependentDrive extends Command {
  public static Pose2d masterOffset = RobotConfig.offsetPositions[0];
  private final Swerve swerve;
  private final Supplier<Pose2d> leftJoystickVelocity, rightJoystickVelocity, masterOffsetSupplier;
  
  private StructSubscriber<Pose2d> masterPoseSubscriber, masterOffsetSubscriber, slaveOffsetSubscriber;
  private StructPublisher<Pose2d> centerPosePublisher, masterOffsetPublisher, slaveOffsetPublisher, slaveTargetPosePublisher, masterCurrentPosePublisher, slaveCurrentPosePublisher;
  // private Pose2d formationCenterPosition = new Pose2d();
  
  public IndependentDrive(Swerve swerve, Supplier<Pose2d> leftJoystickVelocity, Supplier<Pose2d> rightJoystickVelocity, Supplier<Pose2d> masterOffsetSupplier) {
    this.swerve = swerve;
    this.leftJoystickVelocity = leftJoystickVelocity;
    this.rightJoystickVelocity = rightJoystickVelocity;
    this.masterOffsetSupplier = masterOffsetSupplier;
    if (!Constants.IS_MASTER) {
      centerPosePublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("center pose", Pose2d.struct).publish();       
      masterOffsetPublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("master offset", Pose2d.struct).publish();       
      slaveOffsetPublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("slave offset", Pose2d.struct).publish();       
      slaveTargetPosePublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("slave target", Pose2d.struct).publish();       
      masterCurrentPosePublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("master pose", Pose2d.struct).publish();       
      slaveCurrentPosePublisher = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("slave pose", Pose2d.struct).publish();       
      masterPoseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.RobotType.master.toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());
    } else {
      masterOffsetSubscriber = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("master offset", Pose2d.struct).subscribe(new Pose2d(-1, -1, new Rotation2d()));       
      masterOffsetSubscriber = NetworkTableInstance.getDefault().getTable("IndependentDrive").getSubTable(Constants.currentRobot.toString()).getStructTopic("slave offset", Pose2d.struct).subscribe(new Pose2d(-1, -1, new Rotation2d()));       
    }
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    System.out.println("IndependentDrive initialized");

    // formationCenterPosition = 
  }

  @Override
  public void execute() {
    Pose2d velocity = Constants.IS_MASTER ? leftJoystickVelocity.get() : rightJoystickVelocity.get();
    swerve.setRobotSpeeds(new ChassisSpeeds(velocity.getX(), velocity.getY(), velocity.getRotation().getRadians()));
    System.out.println("IndependentDrive executing");
    
    if (!Constants.IS_MASTER) {
      System.out.println("IndependentDrive slave executing");
      Pose2d masterPose = masterPoseSubscriber.get();
      Pose2d currentPose = swerve.getPose();
      Pose2d formationCenterPose = new Pose2d(masterPose.getTranslation().plus(currentPose.getTranslation()).times(0.5), masterPose.getRotation());
      masterOffset = new Pose2d(masterPose.getTranslation().minus(formationCenterPose.getTranslation()).rotateBy(masterPose.getRotation().times(-1)), new Rotation2d());
      Pose2d slaveOffset = new Pose2d(currentPose.getTranslation().minus(formationCenterPose.getTranslation()).rotateBy(masterPose.getRotation().times(-1)), currentPose.getRotation().minus(formationCenterPose.getRotation()));
      RobotConfig.offsetPositions[1] = slaveOffset;
      RobotConfig.offsetPositions[0] = masterOffset;

      centerPosePublisher.accept(formationCenterPose);
      masterOffsetPublisher.accept(masterOffset);
      slaveOffsetPublisher.accept(slaveOffset);

      slaveTargetPosePublisher.accept(masterPose.plus(RobotConfig.offsetPositions[1].minus(RobotConfig.offsetPositions[0])));
      masterCurrentPosePublisher.accept(masterPose);
      slaveCurrentPosePublisher.accept(currentPose);
    } else {
      RobotConfig.offsetPositions[1] = slaveOffsetSubscriber.get();
      RobotConfig.offsetPositions[0] = masterOffsetSubscriber.get();
    }

    // System.out.println(RobotConfig.offsetPositions[0] + " | " + RobotConfig.offsetPositions[1]);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("IndependentDrive ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
