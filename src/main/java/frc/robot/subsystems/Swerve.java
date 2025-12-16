// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConfig;
import frc.robot.Constants.RobotConfig.SingleRobotConfig;
import frc.robot.util.NERDPoseEstimator;
import frc.robot.Constants;
import frc.robot.Robot;

/**a subsystem for a NERDSwerve robot */
public class Swerve extends SubsystemBase {
	private final Pigeon2 gyro;

	private Pigeon2SimState gyroSimState;

	private final ArrayList<DrivePod> pods = new ArrayList<DrivePod>();

	private final NERDPoseEstimator poseEstimator;
	private final SwerveDriveKinematics drivetrainKinematics;

	public double targetAngle = 0;

	private final Field2d field2d = new Field2d();

	StructArrayPublisher<SwerveModuleState> SMSPublisher;
	StructPublisher<Pose2d> PosePublisher;
	StructPublisher<ChassisSpeeds> ChassisSpeedsPublisher;
	DoubleEntry azimuthkPSub;
	DoubleEntry azimuthkISub;
	DoubleEntry azimuthkDSub;

	double simGyroPosition = 0;

	public Swerve(int robot) {
		drivetrainKinematics = RobotConfig.robotConfigs[robot].drivetrainKinematics;
		gyro = new Pigeon2(RobotConfig.pigeonID);

		for (int i = 0; i < RobotConfig.robotConfigs[robot].PodConfigs.length; i++) {
			pods.add(
					// new DrivePod(i, RobotMap.PodConfigs[i].encoderID, RobotMap.PodConfigs[i].leftMotorID,
					// RobotMap.PodConfigs[i].rightMotorID, PodConfig.leftMotorInvert, PodConfig.rightMotorInvert,
					// RobotMap.PodConfigs[i].encoderOffset, PodConfig.encoderInvert, PodConfig.ampLimit,
					// PodConfig.motorsBrake, PodConfig.rampRate, PodConfig.kP, PodConfig.kI, PodConfig.kD, PodConfig.motorGearing));
					new DrivePod(RobotConfig.robotConfigs[robot].PodConfigs[i].encoderID, RobotConfig.robotConfigs[robot].PodConfigs[i].azimuthID, RobotConfig.robotConfigs[robot].PodConfigs[i].driveID, RobotConfig.robotConfigs[robot].PodConfigs[i].encoderOffset, RobotConfig.azimuthInvert, 
					RobotConfig.azimuthAmpLimit, RobotConfig.azimuthRadiansPerMotorRotation, RobotConfig.azimuthBrake, RobotConfig.azimuthMotorRampRate, RobotConfig.azimuthkP, 
					RobotConfig.azimuthkI, RobotConfig.azimuthkD, RobotConfig.azimuthkS, RobotConfig.azimuthMaxOutput, RobotConfig.azimuthDriveSpeedMultiplier, RobotConfig.driveInvert, 
					RobotConfig.driveAmpLimit, RobotConfig.driveBrake, RobotConfig.driveMotorRampRate));
		}


		// initialize pod positions
		SwerveModulePosition positions[] = pods.stream()
				.map(DrivePod::getPodPosition)
				.toArray(SwerveModulePosition[]::new);

		// initialize odometry based on the pod positions
		poseEstimator = new NERDPoseEstimator(
				drivetrainKinematics,
				getGyro(),
				getModulePositions(),
				new Pose2d());


		// gyro.configAllSettings(new Pigeon2Configuration());

		// initialize telemetry publishers
		String tab = Constants.currentRobot.toString();

		SMSPublisher = NetworkTableInstance.getDefault().getTable(tab).getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
				.publish();
		PosePublisher = NetworkTableInstance.getDefault().getTable(tab).getStructTopic("RobotPose", Pose2d.struct).publish();
		ChassisSpeedsPublisher = NetworkTableInstance.getDefault().getTable(tab).getStructTopic("ChassisSpeeds", ChassisSpeeds.struct)
				.publish();

		// initialize PID subscribers
		if(Constants.tuningMode) {
			azimuthkPSub = NetworkTableInstance.getDefault().getTable(tab).getSubTable("PIDs").getDoubleTopic("kP").getEntry(RobotConfig.PodConfig.kP);
			azimuthkISub = NetworkTableInstance.getDefault().getTable(tab).getSubTable("PIDs").getDoubleTopic("kI").getEntry(RobotConfig.PodConfig.kI);
			azimuthkDSub = NetworkTableInstance.getDefault().getTable(tab).getSubTable("PIDs").getDoubleTopic("kD").getEntry(RobotConfig.PodConfig.kD);

			azimuthkPSub.set(RobotConfig.PodConfig.kP);
			azimuthkISub.set(RobotConfig.PodConfig.kI);
			azimuthkDSub.set(RobotConfig.PodConfig.kD);

			azimuthkDSub.getAtomic();
		}

		if (Robot.isSimulation()) {
			gyroSimState = new Pigeon2SimState(gyro);
		}
	}

	@Override
	public void periodic() {
		//update odometry
		poseEstimator.update(getGyro(), getModulePositions());

		//update telemetry
		SmartDashboard.putNumber("pigeon", getGyro().getDegrees());

		field2d.setRobotPose(getPose());

		SMSPublisher.set(getModuleStates());
		PosePublisher.set(getPose());
		ChassisSpeedsPublisher.set(drivetrainKinematics.toChassisSpeeds(getModuleStates()));


		// update the numbers (if tuning mode is enabled)
		if (Constants.tuningMode) {
			pods.forEach(pod -> {
			pod.setPID(
					azimuthkPSub.get(),
					azimuthkISub.get(),
					azimuthkDSub.get());
		});
		}
	}

	/**
	 * gets the current robot chassis speeds
	 * 
	 * @return the current robot chassis speeds
	 */
	private ChassisSpeeds getCurrentRobotChassisSpeeds() {
		// get the current robot chassis speeds
		return drivetrainKinematics.toChassisSpeeds(getModuleStates());
	}

	/**
	 * gets the current robot pose
	 * 
	 * @return the current robot pose
	 */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * resets the odometry to the given pose
	 * 
	 * @param pose the new pose to reset to
	 */
	public void resetOdometry(Pose2d pose) {
		poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), pose);
	}

	/**
	 * gets the current robot chassis speeds
	 * 
	 * @return the current robot chassis speeds
	 */
	public SwerveModulePosition[] getModulePositions() {
		return pods.stream()
				.map(DrivePod::getPodPosition)
				.toArray(SwerveModulePosition[]::new);
	}

	/**
	 * gets the current robot module states
	 * 
	 * @return the current robot module states
	 */
	public SwerveModuleState[] getModuleStates() {
		return pods.stream()
				.map(DrivePod::getState)
				.toArray(SwerveModuleState[]::new);
	}

	/**
	 * resets the pods and odometry zero
	 */
	public void resetPods() {
		resetGyro();
		pods.stream().forEach(DrivePod::resetPod);

		resetOdometry(new Pose2d());
	}

	/**
	 * sets all motor output to 0
	 */
	public void stop() {
		pods.stream().forEach(DrivePod::stop);
	}

	public Rotation2d getGyro() {
		return gyro.getRotation2d();
	}

	public void resetGyro() {
		gyro.setYaw(0);
	}

	/**
	 * sets the robot's chassis speeds based on the given chassis speeds, enables isTurning
	 * @param chassisSpeeds the desired chassis speeds (+x forward, +y left, +omega counter-clockwise)
	 */
	public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
		setRobotSpeeds(chassisSpeeds, true);
	}

	/**
	 * sets the robot's chassis speeds based on the given chassis speeds
	 * @param chassisSpeeds the desired chassis speeds (+x forward, +y left, +omega counter-clockwise)
	 * @param enableIsTurning if true, the robot will not move if any pod is turning and will not start turning if any pod is still moving
	 */
	public void setRobotSpeeds(ChassisSpeeds chassisSpeeds, boolean enableIsTurning) {
		SwerveModuleState[] states = drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, RobotConfig.robotMaxSpeed);

		for (int i = 0; i < pods.size(); i++) {
			pods.get(i).setPodState(states[i], enableIsTurning);
		}
	}

	public ArrayList<DrivePod> getPods() {
		return pods;
	}

	@Override
	public void simulationPeriodic() {
		// update the gyro to functionally be the robot/odo angle
		simGyroPosition += Units.radiansToDegrees(drivetrainKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond) * Robot.kDefaultPeriod;
		gyroSimState.setRawYaw(simGyroPosition);
	}

	/**
	 * accepts a vision measurement for pose estimation
	 * 
	 * @param visionPose estimated pose from the vision system
	 * @param distance   distance from the vision target to the robot, used for
	 *                   uncertainty in the pose estimation
	 */

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, double distance) {
		if(DriverStation.isDisabled()) {
			//fully sync pose estimator to vision on disable
			poseEstimator.resetOdometry(visionPose);
			poseEstimator.addVisionMeasurement(visionPose,
					timestamp,
					VecBuilder.fill(0.01, 0.01, 0.01));
		} else {
			//add some bias to odo on enable
			poseEstimator.addVisionMeasurement(visionPose,
					timestamp,
					// decreases vision confidence with distance
					VecBuilder.fill(distance / 2, distance / 2, distance / 2));
		}
	}

	
	/**
	 * sets pose to this
	 * 
	 * @param pose estimated pose from the vision system
	 */
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPose(pose);
	}
}
