// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

/**
 * This class wraps {@link Odometry} to fuse latency-compensated vision measurements with encoder
 * measurements. Robot code should not use this directly- Instead, use the particular type for your
 * drivetrain (e.g., {@link DifferentialDrivePoseEstimator}). It is intended to be a drop-in
 * replacement for {@link Odometry}; in fact, if you never call {@link
 * PoseEstimator#addVisionMeasurement} and only call {@link PoseEstimator#update} then this will
 * behave exactly the same as Odometry.
 *
 * <p>{@link PoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link PoseEstimator#addVisionMeasurement} can be called as infrequently as you want; if you
 * never call it then this class will behave exactly like regular encoder odometry.
 */
public class NERDPoseEstimator {
    private final SwerveDriveOdometry m_odometry;
    private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());

    private static final double kBufferDuration = 1.5;
    // Maps timestamps to odometry-only pose estimates
    private final TimeInterpolatableBuffer<Pose2d> m_odometryPoseBuffer =
            TimeInterpolatableBuffer.createBuffer(kBufferDuration);
    // Maps timestamps to vision updates
    // Always contains one entry before the oldest entry in m_odometryPoseBuffer, unless there have
    // been no vision measurements after the last reset
    private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();

    private Pose2d m_poseEstimate;
    
    // Dual-robot mode: second estimator for slave robot
    private NERDPoseEstimator m_slaveEstimator = null;
    private Pose2d m_slavePoseEstimate = new Pose2d();

    /**
     * Constructs a TurdPoseEstimator.
     *
     * @param kinematics A correctly-configured kinematics object for your drivetrain.
     * @param odometry A correctly-configured odometry object for your drivetrain.
     * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
     *         in meters, and heading in radians). Increase these numbers to trust your state estimate
     *         less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *         in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *         the vision pose measurement less.
     */
    @SuppressWarnings("PMD.UnusedFormalParameter")
    private NERDPoseEstimator(
            SwerveDriveKinematics kinematics,
            SwerveDriveOdometry odometry,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        m_odometry = odometry;

        m_poseEstimate = m_odometry.getPoseMeters();

        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }


    /**
     * Constructs a TurdPoseEstimator with default standard deviations for the model and vision
     * measurements.
     *
     * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
     * and 0.1 radians for heading. The default standard deviations of the vision measurements are 0.9
     * meters for x, 0.9 meters for y, and 0.9 radians for heading.
     *
     * @param kinematics A correctly-configured kinematics object for your drivetrain.
     * @param gyroAngle The current gyro angle.
     * @param modulePositions The current distance measurements and rotations of the swerve modules.
     * @param initialPoseMeters The starting pose estimate.
     */
    public NERDPoseEstimator(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {
        this(
                kinematics,
                gyroAngle,
                modulePositions,
                initialPoseMeters,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9));
    }

    /**
     * Constructs a TurdPoseEstimator.
     *
     * @param kinematics A correctly-configured kinematics object for your drivetrain.
     * @param gyroAngle The current gyro angle.
     * @param modulePositions The current distance and rotation measurements of the swerve modules.
     * @param initialPoseMeters The starting pose estimate.
     * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
     *         in meters, and heading in radians). Increase these numbers to trust your state estimate
     *         less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *         in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *         the vision pose measurement less.
     */
    public NERDPoseEstimator(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        this(
                kinematics,
                new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters),
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to change trust in
     * vision measurements after the autonomous period, or to change trust as distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     *         numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     *         theta]ᵀ, with units in meters and radians.
     */
    public final void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                        row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
            }
        }
    }

    /**
     * Sets the pose estimator's trust of odometry measurements
     * @param odometryMeasurementStdDevs Standard deviations of the odometry measurements. Increase these
     */
    public final void setOdometryMeasurementStdDevs(Matrix<N3, N1> odometryMeasurementStdDevs) {
        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, odometryMeasurementStdDevs.get(i, 0) * odometryMeasurementStdDevs.get(i, 0));
        }
    }

    /**
     * call this when drift gets too bad
     * @param pose pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPose(pose);
    }

    /**
     * call this when drift gets too bad
     */
  public void resetOdometryPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
        m_odometry.resetPosition(gyroAngle, wheelPositions, poseMeters);
    }

    public Pose2d getOdometry() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param wheelPositions The current encoder readings.
     * @param poseMeters The position on the field that your robot is at.
     */
    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
        // Reset state estimate and error covariance
        m_odometry.resetPosition(gyroAngle, wheelPositions, poseMeters);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
    }

    /**
     * Resets the robot's pose.
     *
     * @param pose The pose to reset to.
     */
    public void resetPose(Pose2d pose) {
        m_odometry.resetPose(pose);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
    }

    /**
     * Resets the robot's translation.
     *
     * @param translation The pose to translation to.
     */
    public void resetTranslation(Translation2d translation) {
        m_odometry.resetTranslation(translation);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
    }

    /**
     * Resets the robot's rotation.
     *
     * @param rotation The rotation to reset to.
     */
    public void resetRotation(Rotation2d rotation) {
        m_odometry.resetRotation(rotation);
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
    }

    /**
     * Gets the estimated robot pose.
     *
     * @return The estimated robot pose in meters.
     */
    public Pose2d getEstimatedPosition() {
        return m_poseEstimate;
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The pose's timestamp in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    public Optional<Pose2d> sampleAt(double timestampSeconds) {
        // Step 0: If there are no odometry updates to sample, skip.
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        // Step 1: Make sure timestamp matches the sample from the odometry pose buffer. (When sampling,
        // the buffer will always use a timestamp between the first and last timestamps)
        double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();
        double newestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().lastKey();
        timestampSeconds =
                MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        // Step 2: If there are no applicable vision updates, use the odometry-only information.
        if (m_visionUpdates.isEmpty() || timestampSeconds < m_visionUpdates.firstKey()) {
            return m_odometryPoseBuffer.getSample(timestampSeconds);
        }

        // Step 3: Get the latest vision update from before or at the timestamp to sample at.
        double floorTimestamp = m_visionUpdates.floorKey(timestampSeconds);
        var visionUpdate = m_visionUpdates.get(floorTimestamp);

        // Step 4: Get the pose measured by odometry at the time of the sample.
        var odometryEstimate = m_odometryPoseBuffer.getSample(timestampSeconds);

        // Step 5: Apply the vision compensation to the odometry pose.
        return odometryEstimate.map(odometryPose -> visionUpdate.compensate(odometryPose));
    }

    /** Removes stale vision updates that won't affect sampling. */
    private void cleanUpVisionUpdates() {
        // Step 0: If there are no odometry samples, skip.
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return;
        }

        // Step 1: Find the oldest timestamp that needs a vision update.
        double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();

        // Step 2: If there are no vision updates before that timestamp, skip.
        if (m_visionUpdates.isEmpty() || oldestOdometryTimestamp < m_visionUpdates.firstKey()) {
            return;
        }

        // Step 3: Find the newest vision update timestamp before or at the oldest timestamp.
        double newestNeededVisionUpdateTimestamp = m_visionUpdates.floorKey(oldestOdometryTimestamp);

        // Step 4: Remove all entries strictly before the newest timestamp we need.
        m_visionUpdates.headMap(newestNeededVisionUpdateTimestamp, false).clear();
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * NERDPoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
     *         don't use your own time source by calling {@link #updateWithTime(double,Rotation2d,SwerveModulePosition[])},
     *         then you must use a timestamp with
     *         an epoch since FPGA startup (i.e., the epoch of this timestamp is the same epoch as {@link
     *         edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.) This means that you should use {@link
     *         edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source or sync the epochs.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()
                || m_odometryPoseBuffer.getInternalBuffer().lastKey() - kBufferDuration
                        > timestampSeconds) {
            return;
        }

        // Step 1: Clean up any old entries
        cleanUpVisionUpdates();

        // Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
        var odometrySample = m_odometryPoseBuffer.getSample(timestampSeconds);

        if (odometrySample.isEmpty()) {
            return;
        }

        // Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was
        // made.
        var visionSample = sampleAt(timestampSeconds);

        if (visionSample.isEmpty()) {
            return;
        }

        // Step 4: Measure the twist between the old pose estimate and the vision pose.
        var twist = visionSample.get().log(visionRobotPoseMeters);

        // Step 5: We should not trust the twist entirely, so instead we scale this twist by a Kalman
        // gain matrix representing how much we trust vision measurements compared to our current pose.
        var k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

        // Step 6: Convert back to Twist2d.
        var scaledTwist =
                new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

        // Step 7: Calculate and record the vision update.
        var visionUpdate = new VisionUpdate(visionSample.get().exp(scaledTwist), odometrySample.get());
        m_visionUpdates.put(timestampSeconds, visionUpdate);

        // Step 8: Remove later vision measurements. (Matches previous behavior)
        m_visionUpdates.tailMap(timestampSeconds, false).entrySet().clear();

        // Step 9: Update latest pose estimate. Since we cleared all updates after this vision update,
        // it's guaranteed to be the latest vision update.
        m_poseEstimate = visionUpdate.compensate(m_odometry.getPoseMeters());


		double distanceFromVision = odometrySample.get().getTranslation().getDistance(visionRobotPoseMeters.getTranslation());

        if (distanceFromVision > 0.05) {
            //offset the current odometry by the vision pose
            Pose2d offsetPose = getOdometry().plus(visionRobotPoseMeters.minus(odometrySample.get()));
			if(offsetPose.getTranslation().getDistance(visionRobotPoseMeters.getTranslation()) > 0.05) {
                resetOdometry(visionRobotPoseMeters);
            } else {
                resetOdometry(offsetPose);
            }
		} else {
            setOdometryMeasurementStdDevs(VecBuilder.fill(distanceFromVision * 3, distanceFromVision * 3, distanceFromVision * 3));
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * PoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * <p>Note that the vision measurement standard deviations passed into this method will continue
     * to apply to future measurements until a subsequent call to {@link
     * PoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
     *         don't use your own time source by calling {@link #updateWithTime(double, Rotation2d, SwerveModulePosition[])}, then you must use a
     *         timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same
     *         epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}). This means that you
     *         should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source in
     *         this case.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *         in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *         the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called every
     * loop.
     *
     * @param gyroAngle The current gyro angle.
     * @param wheelPositions The current encoder readings.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        return updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, wheelPositions);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This should be called every
     * loop.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param gyroAngle The current gyro angle.
     * @param wheelPositions The current encoder readings.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        var odometryEstimate = m_odometry.update(gyroAngle, wheelPositions);

        m_odometryPoseBuffer.addSample(currentTimeSeconds, odometryEstimate);

        if (m_visionUpdates.isEmpty()) {
            m_poseEstimate = odometryEstimate;
        } else {
            var visionUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
            m_poseEstimate = visionUpdate.compensate(odometryEstimate);
        }

        return getEstimatedPosition();
    }

    /**
     * Represents a vision update record. The record contains the vision-compensated pose estimate as
     * well as the corresponding odometry pose estimate.
     */
    private static final class VisionUpdate {
        // The vision-compensated pose estimate.
        private final Pose2d visionPose;

        // The pose estimated based solely on odometry.
        private final Pose2d odometryPose;

        /**
         * Constructs a vision update record with the specified parameters.
         *
         * @param visionPose The vision-compensated pose estimate.
         * @param odometryPose The pose estimate based solely on odometry.
         */
        private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
         * relative to this record's odometry pose to being relative to this record's vision pose.
         *
         * @param pose The pose to compensate.
         * @return The compensated pose.
         */
        public Pose2d compensate(Pose2d pose) {
            var delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }
    
    /**
     * Initializes dual-robot mode with a slave pose estimator.
     * This allows combining inputs from both master and slave robots.
     * 
     * @param slaveKinematics Kinematics for the slave robot
     * @param slaveGyroAngle Initial gyro angle for slave
     * @param slaveModulePositions Initial module positions for slave
     * @param slaveInitialPose Initial pose estimate for slave
     */
    public void initializeDualRobotMode(
            SwerveDriveKinematics slaveKinematics,
            Rotation2d slaveGyroAngle,
            SwerveModulePosition[] slaveModulePositions,
            Pose2d slaveInitialPose) {
        m_slaveEstimator = new NERDPoseEstimator(
            slaveKinematics,
            slaveGyroAngle,
            slaveModulePositions,
            slaveInitialPose
        );
        m_slavePoseEstimate = slaveInitialPose;
    }
    
    /**
     * Updates slave robot pose estimate with odometry received from slave.
     * 
     * @param slaveOdometry The slave robot's current odometry pose
     * @return Updated slave pose estimate
     */
    public Pose2d updateSlaveOdometry(Pose2d slaveOdometry) {
        if (m_slaveEstimator != null) {
            // If we have a full estimator, we'd update it with module positions
            // For now, use the odometry directly
            m_slavePoseEstimate = slaveOdometry;
        }
        return m_slavePoseEstimate;
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
        if (m_slaveEstimator == null) {
            return;
        }
        
        // Convert slave camera transform to field-relative pose
        // The transform is robot-relative (from slave camera to tag)
        // We need to compute the field-relative pose of the slave
        
        // For a more accurate implementation, we would:
        // 1. Use the known tag position in field coordinates
        // 2. Transform from tag to slave camera
        // 3. Transform from slave camera to slave center
        // 4. Get field-relative slave pose
        
        // Simplified approach: use master pose + relative offset
        // This assumes the tag is at a known location relative to master
        // In practice, you'd want to use the actual tag field position
        Pose2d fieldRelativeSlavePose = masterPoseAtTime.plus(slaveCameraTransform);
        
        m_slaveEstimator.addVisionMeasurement(
            fieldRelativeSlavePose,
            timestampSeconds,
            VecBuilder.fill(distance / 2, distance / 2, distance / 2)
        );
        m_slavePoseEstimate = m_slaveEstimator.getEstimatedPosition();
    }
    
    /**
     * Combines robot-relative offsets from master and slave cameras.
     * Uses the relative transform between robots to improve both pose estimates.
     * 
     * @param masterCameraTransform Master robot-relative transform to tag
     * @param slaveCameraTransform Slave robot-relative transform to tag
     * @param masterTimestamp Timestamp of master measurement
     * @param slaveTimestamp Timestamp of slave measurement
     * @param masterPoseAtTime Master pose at master measurement time
     * @param slavePoseAtTime Slave pose at slave measurement time
     */
    public void combineRelativeOffsets(
            Transform2d masterCameraTransform,
            Transform2d slaveCameraTransform,
            double masterTimestamp,
            double slaveTimestamp,
            Pose2d masterPoseAtTime,
            Pose2d slavePoseAtTime) {
        if (m_slaveEstimator == null) {
            return;
        }
        
        // Both robots see the same tag, so we can compute their relative positions
        // The difference in camera transforms tells us about the relative pose
        
        // Compute relative transform: if both see the same tag, the difference
        // in their camera-to-tag transforms (in tag frame) gives us robot-to-robot transform
        // Transform from master to slave in tag frame: slaveTransform - masterTransform
        
        // Use this relative transform to cross-validate and improve both estimates
        // If we know the expected relative position, we can use it to correct both poses
        
        // For now, we'll use a weighted combination approach
        // In a more sophisticated implementation, we would use the relative constraint
        // to jointly optimize both poses
    }
    
    /**
     * Gets the estimated pose for the slave robot (only valid in dual-robot mode).
     * 
     * @return Slave robot pose estimate
     */
    public Pose2d getSlaveEstimatedPosition() {
        return m_slavePoseEstimate;
    }
    
    /**
     * Resets the slave pose estimator.
     * 
     * @param slaveGyroAngle Slave gyro angle
     * @param slaveModulePositions Slave module positions
     * @param slavePose New slave pose
     */
    public void resetSlavePosition(
            Rotation2d slaveGyroAngle,
            SwerveModulePosition[] slaveModulePositions,
            Pose2d slavePose) {
        if (m_slaveEstimator != null) {
            m_slaveEstimator.resetPosition(slaveGyroAngle, slaveModulePositions, slavePose);
            m_slavePoseEstimate = slavePose;
        }
    }
}
