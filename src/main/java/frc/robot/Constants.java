// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RobotMap.PodConfig;

/**
 * Constants class holds all configuration values and constants for the robot.
 * Includes robot type, CAN IDs, kinematics, PID gains, and hardware settings.
 * Organized into nested classes for logical grouping.
 */
public final class Constants {
	public static final boolean tuningMode = false; // if true, the robot will use the dashboard to get values for PIDs

    //grab the master/slave from a file on the robot. This will determine whether it's running the shuffleboard server.
    public static final Path MASTER_PATH = Paths.get("/home/lvuser/master");
    public static final boolean IS_MASTER = MASTER_PATH.toFile().exists();
	//TODO: phase out if statements and replace them with switches w/ this enum
	public static enum RobotType {
		master, slave
	} 

	public static RobotType currentRobot = IS_MASTER ? RobotType.master : RobotType.slave;

	/**
	 * RobotMap contains hardware mapping and configuration for robot components.
	 * Includes pod configurations, camera names, and kinematics.
	 */
	public final class RobotMap {
		private static final double wheelBase = 1.0;
		private static final double trackWidth = 1.0;
		public static final PodConfig[] PodConfigs = {
			new PodConfig(0, 1, 2, 0d, new Translation2d(wheelBase / 2, trackWidth / 2)),
			new PodConfig(3, 4, 5, 0d, new Translation2d(wheelBase / 2, -trackWidth / 2)),
			new PodConfig(6, 7, 8, 0d, new Translation2d(-wheelBase / 2, trackWidth / 2)),
			new PodConfig(9, 10, 11, 0d, new Translation2d(-wheelBase / 2, -trackWidth / 2))
		};
		public static SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(
			java.util.Arrays.stream(PodConfigs)
				.map(pod -> pod.position)
				.toArray(Translation2d[]::new)
		);



		// Camera IDs. this is for individual camera-threads, but there's only one so its fine
		public static enum CameraName {
			master, slave
		}

		/**
		 * PodConfig holds configuration for a single swerve pod.
		 * Includes motor IDs, encoder offsets, and position.
		 */
		public static final class PodConfig {
			public final int leftMotorID;
			public final int rightMotorID;
			public final int encoderID;
			public final double encoderOffset;
			public final Translation2d position;
			public PodConfig(int leftMotorID, int rightMotorID, int encoderID, double encoderOffset, Translation2d position) {
				this.leftMotorID = leftMotorID;
				this.rightMotorID = rightMotorID;
				this.encoderID = encoderID;
				this.encoderOffset = encoderOffset;
				this.position = position;
			}
			public static final boolean encoderInvert = true;
			public static final boolean leftMotorInvert = false; 
			public static final boolean rightMotorInvert = true; 

			public static final double motorGearing = 50;

			public static final boolean motorsBrake = true;
			public static final int ampLimit = 80;
			public static final double maxOutput = 1;
			public static final double rampRate = 0.2;

			public static final double kP = 1;
			public static final double kI = 0.0;
			public static final double kD = 0.2;
		}
	}

	/**
	 * RobotConstants contains robot-wide constants for control and hardware.
	 * Includes speed limits, PID gains, and camera/vision offsets.
	 */
	public final class RobotConstants {
		public static final double robotMaxLinearSpeed = 1; // meters per second
		public static final double robotMaxRotationalSpeed = 2.0; // meters per second
		public static final double motorMaxOutput = 1.0; // max output of the motors

		//PID gains for tandem drive controller
		public static final double tandemkP = 0.3;
		public static final double tandemkI = 0.0;
		public static final double tandemkD = 0.0;

		public static final double tandemkP_angle = 0.3;
		public static final double tandemkI_angle = 0.0;
		public static final double tandemkD_angle = 0.0;

		public static final double tandemTranslation_deadband = 0.008;
		public static final double tandemAngle_deadband = 0.005;

		//forward, left, height; roll, pitch, yaw
		public static final Translation2d centerOfMasterToTag = new Translation2d(0, 0.2102); // meters, distance from the center of the master robot to the tag
		public static final Transform3d SLAVE_CAMERA_LOCATION = new Transform3d(new Translation3d(0.152, 0.1996, -0.015), new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(105)));
	}

	/**
	 * JoystickConstants contains configuration for joystick input.
	 * Includes port numbers and deadband values.
	 */
	public final class JoystickConstants {
		public static final int driverPort = 0; // port of the driver controller

		public static final double deadband = 0.05; // deadband for the joysticks
		public static final double triggerDeadband = 0.05; // deadband for the triggers
	}

	/**
	 * SimConstants contains simulation-specific constants for robot physics.
	 */
	public final class SimConstants {
		public static final double inertia = 0.0605; // kg*m^2
		public static final double mass = 50; // kg, approximate mass of the robot
		public static final double wheelRadius = 0.3; // meters
		public static final double trackWidth = 0.5; // meters, distance between left and right wheels
		public static final double gearRatio = 1/PodConfig.motorGearing; // gear ratio of the drivetrain

	}

	/**
	 * RobotConfig contains configuration for the robot's drive system and pods.
	 * Includes pod configs, kinematics, and reset logic.
	 */
	public class RobotConfig {
		public static final double driveMetersPerMotorRotation = 1/(Units.inchesToMeters(2) * Math.PI / 1.36); //Wheel Diameter M * PI / Enc Count Per Rev / Gear Ratio

        public static final int pigeonID = 25;
        public static final PIDController gyroPID = new PIDController(0.046, 0d, 0.001);

        /**
		 * PodConfig holds configuration for a single robot pod.
		 */
        public static final class PodConfig {
			public final int azimuthID;
			public final int driveID;
			public final int encoderID;
			public final double encoderOffset;
			public final Translation2d position;
			public PodConfig(int azimuthID, int driveID, int encoderID, double encoderOffset, Translation2d podPosition) {
				this.azimuthID = azimuthID;
				this.driveID = driveID;
				this.encoderID = encoderID;
				this.encoderOffset = encoderOffset;
				this.position = podPosition;
			}

			public static final double motorGearing = 50;

			public static final boolean motorsBrake = true;

			public static final double kP = 1;
			public static final double kI = 0.0;
			public static final double kD = 0.2;
		}


		private static final double wheelBase = Units.inchesToMeters(11.054);
		private static final double trackWidth = Units.inchesToMeters(11.054);
		public static Pose2d[] offsetPositions = {new Pose2d(new Translation2d(0.0, 0.4), new Rotation2d()), new Pose2d(new Translation2d(0.0, -0.4), new Rotation2d())}; // default center of rotation of robot
		public static Command reset() {
			return new InstantCommand(() -> resetOffsetPositions());
		}
		public static void resetOffsetPositions() {
			offsetPositions[0] = new Pose2d(new Translation2d(0.0, 0.4), new Rotation2d());
			offsetPositions[1] = new Pose2d(new Translation2d(0.0, -0.4), new Rotation2d());
		}

		/**
		 * SingleRobotConfig holds configuration for a robot's set of pods and kinematics.
		 */
		public static final class SingleRobotConfig {
			public PodConfig[] PodConfigs;
			public SwerveDriveKinematics drivetrainKinematics;
			public SingleRobotConfig(PodConfig[] podConfigs) {
				this.PodConfigs = podConfigs;
				this.drivetrainKinematics = new SwerveDriveKinematics(
					java.util.Arrays.stream(podConfigs)
						.map(pod -> pod.position)
						.toArray(Translation2d[]::new)
				);
			}
		}
		public static final SingleRobotConfig[] robotConfigs = new SingleRobotConfig[] { 
		new SingleRobotConfig(new PodConfig[] { // prowl AKA master AKA og megatron
			new PodConfig(16, 15, 23, -0.8865d, new Translation2d(wheelBase / 2, trackWidth / 2)),
			new PodConfig(14, 13, 22, 0.3328d, new Translation2d(wheelBase / 2, -trackWidth / 2)),
			new PodConfig(18, 17, 24, 0.8132d, new Translation2d(-wheelBase / 2, trackWidth / 2)),
			new PodConfig(12, 11, 21, -0.4138d + (1d/33d), new Translation2d(-wheelBase / 2, -trackWidth / 2))
		}),
		new SingleRobotConfig(new PodConfig[] { // NemesisPrime AKA slave AKA Kris's NERDSwerve
			new PodConfig(16, 15, 23, 0.9844, new Translation2d(wheelBase / 2, trackWidth / 2)),
			new PodConfig(14, 13, 22, 0.4797, new Translation2d(wheelBase / 2, -trackWidth / 2)),
			new PodConfig(18, 17, 24, -0.6980, new Translation2d(-wheelBase / 2, trackWidth / 2)),
			new PodConfig(12, 11, 21, -0.4006, new Translation2d(-wheelBase / 2, -trackWidth / 2))
		})};


        public static final double robotMaxSpeed = 3.99; // joystick multiplier in meters per second
        public static final double formationMaxRotationalSpeed = 0.6; //maximum rotational speed of the formation in rad/s


        // Azimuth Settings
        public static final boolean azimuthBrake = true;

        public static final int azimuthAmpLimit = 80;
        public static final double azimuthMaxOutput = 1;


        public static final double azimuthkP = 1.2;
        
        public static final double azimuthkI = 0.02;
        public static final double azimuthkD = 0.001;
        public static final double azimuthkS = 0.0;

        public static final double azimuthDriveSpeedMultiplier = 0.5;

        public static final double azimuthMotorRampRate = 0.0;
		public static final boolean encoderInvert = true;
		public static final boolean azimuthInvert = false; 

        // Drive Settings
        public static final double podMaxSpeed = 1;

        public static final boolean driveBrake = false;

        public static final int driveAmpLimit = 80;
        public static final int boostDriveLimit = 90;
        public static final double driveMotorRampRate = 0.2;

        public static final double azimuthRadiansPerMotorRotation = 2.200000047683716;
		public static final boolean driveInvert = true; 
	}
}
