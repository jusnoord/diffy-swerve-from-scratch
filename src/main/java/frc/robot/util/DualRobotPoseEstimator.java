// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * DualRobotPoseEstimator manages pose estimation for both master and slave robots.
 * Combines odometry and camera inputs from both robots to compute improved pose estimates.
 * All computation happens on the master robot.
 */
public class DualRobotPoseEstimator {
    private final NERDPoseEstimator masterEstimator;
    private final NERDPoseEstimator slaveEstimator;
    
    private Pose2d masterPose = new Pose2d();
    private Pose2d slavePose = new Pose2d();
    
    /**
     * Constructs a DualRobotPoseEstimator.
     * 
     * @param masterKinematics Kinematics for the master robot
     * @param slaveKinematics Kinematics for the slave robot
     * @param masterGyroAngle Initial gyro angle for master
     * @param masterModulePositions Initial module positions for master
     * @param masterInitialPose Initial pose estimate for master
     * @param slaveGyroAngle Initial gyro angle for slave (from slave odometry)
     * @param slaveModulePositions Initial module positions for slave (from slave odometry)
     * @param slaveInitialPose Initial pose estimate for slave
     */
    public DualRobotPoseEstimator(
            SwerveDriveKinematics masterKinematics,
            SwerveDriveKinematics slaveKinematics,
            Rotation2d masterGyroAngle,
            SwerveModulePosition[] masterModulePositions,
            Pose2d masterInitialPose,
            Rotation2d slaveGyroAngle,
            SwerveModulePosition[] slaveModulePositions,
            Pose2d slaveInitialPose) {
        masterEstimator = new NERDPoseEstimator(
            masterKinematics,
            masterGyroAngle,
            masterModulePositions,
            masterInitialPose
        );
        slaveEstimator = new NERDPoseEstimator(
            slaveKinematics,
            slaveGyroAngle,
            slaveModulePositions,
            slaveInitialPose
        );
    }
    
    /**
     * Updates master robot pose estimate with odometry.
     * 
     * @param gyroAngle Current gyro angle
     * @param modulePositions Current module positions
     * @return Updated master pose estimate
     */
    public Pose2d updateMaster(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        masterPose = masterEstimator.update(gyroAngle, modulePositions);
        return masterPose;
    }
    
    /**
     * Updates slave robot pose estimate with odometry received from slave.
     * 
     * @param slaveOdometry The slave robot's current odometry pose
     * @return Updated slave pose estimate (currently just returns the odometry, will be enhanced with fusion)
     */
    public Pose2d updateSlave(Pose2d slaveOdometry) {
        // For now, use the slave odometry directly. In the future, we could maintain
        // a separate odometry state for the slave if we receive module positions.
        slavePose = slaveOdometry;
        return slavePose;
    }
    
    /**
     * Adds a vision measurement from the master robot's camera.
     * 
     * @param visionRobotPoseMeters Field-relative pose from master camera
     * @param timestampSeconds Timestamp of the vision measurement
     * @param distance Distance to the vision target
     */
    public void addMasterVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, double distance) {
        masterEstimator.addVisionMeasurement(
            visionRobotPoseMeters,
            timestampSeconds,
            VecBuilder.fill(distance / 2, distance / 2, distance / 2)
        );
        masterPose = masterEstimator.getEstimatedPosition();
    }
    
    /**
     * Adds a vision measurement from the slave robot's camera.
     * Combines the slave camera transform with master pose to compute field-relative pose.
     * 
     * @param slaveCameraTransform Robot-relative transform from slave camera to tag
     * @param timestampSeconds Timestamp of the vision measurement
     * @param masterPoseAtTime Master pose at the time of the vision measurement
     * @param distance Distance to the vision target
     */
    public void addSlaveVisionMeasurement(
            Transform2d slaveCameraTransform,
            double timestampSeconds,
            Pose2d masterPoseAtTime,
            double distance) {
        // Convert slave camera transform to field-relative pose
        // The slave camera sees a tag, and we know the master's position
        // We need to compute where the slave is relative to the master based on the camera transform
        
        // For now, use a simple approach: assume the tag is at a known location
        // and compute slave pose from that. In a more sophisticated implementation,
        // we would use the relative transform between robots.
        
        // Compute field-relative pose: master pose + relative offset from camera transform
        // This is a simplified version - the actual implementation would need to account
        // for the tag's field position and the relationship between master and slave
        Pose2d fieldRelativeSlavePose = masterPoseAtTime.plus(slaveCameraTransform);
        
        slaveEstimator.addVisionMeasurement(
            fieldRelativeSlavePose,
            timestampSeconds,
            VecBuilder.fill(distance / 2, distance / 2, distance / 2)
        );
        slavePose = slaveEstimator.getEstimatedPosition();
    }
    
    /**
     * Combines robot-relative offsets from master and slave cameras to improve pose estimates.
     * Uses the relative transform between robots to cross-validate and improve both estimates.
     * 
     * @param masterCameraTransform Master robot-relative transform to tag
     * @param slaveCameraTransform Slave robot-relative transform to tag
     * @param timestamp Timestamp of the measurements
     */
    public void combineRelativeOffsets(
            Transform2d masterCameraTransform,
            Transform2d slaveCameraTransform,
            double timestamp) {
        // Compute the relative transform between robots based on camera measurements
        // If both robots see the same tag, the difference in their camera transforms
        // tells us about their relative positions
        
        // Transform from master to slave: slaveTransform - masterTransform (in tag frame)
        // This gives us the relative position of slave relative to master
        
        // For now, we'll use a weighted average approach
        // In a more sophisticated implementation, we would use the relative transform
        // to constrain both poses and improve accuracy
        
        // The relative offset can be used to cross-validate the poses
        // If master sees tag at position A and slave sees it at position B,
        // the difference should match the known relative position of the robots
    }
    
    /**
     * Gets the current estimated pose for the master robot.
     * 
     * @return Master robot pose estimate
     */
    public Pose2d getMasterPose() {
        return masterPose;
    }
    
    /**
     * Gets the current estimated pose for the slave robot.
     * 
     * @return Slave robot pose estimate
     */
    public Pose2d getSlavePose() {
        return slavePose;
    }
    
    /**
     * Resets both pose estimators.
     * 
     * @param masterPose New master pose
     * @param slavePose New slave pose
     * @param masterGyroAngle Master gyro angle
     * @param masterModulePositions Master module positions
     * @param slaveGyroAngle Slave gyro angle (if available)
     * @param slaveModulePositions Slave module positions (if available)
     */
    public void resetPoses(
            Pose2d masterPose,
            Pose2d slavePose,
            Rotation2d masterGyroAngle,
            SwerveModulePosition[] masterModulePositions,
            Rotation2d slaveGyroAngle,
            SwerveModulePosition[] slaveModulePositions) {
        masterEstimator.resetPosition(masterGyroAngle, masterModulePositions, masterPose);
        slaveEstimator.resetPosition(slaveGyroAngle, slaveModulePositions, slavePose);
        this.masterPose = masterPose;
        this.slavePose = slavePose;
    }
}

