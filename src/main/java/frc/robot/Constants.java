// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;

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
    // public static final Path MASTER_PATH = Paths.get("/home/lvuser/master");
	public static final File MASTER_FILE = new File("/is_master");
    public static final boolean IS_MASTER = MASTER_FILE.exists(); // MASTER_PATH.toFile().exists();
	//TODO: phase out if statements and replace them with switches w/ this enum
	public static enum RobotType {
		master, slave;

		public RobotType getOpposite() {
			return this == master ? slave : master;
		}
	} 

	public static RobotType currentRobot = IS_MASTER ? RobotType.master : RobotType.slave;

	/**
	 * RobotMap contains hardware mapping and configuration for robot components.
	 * Includes pod configurations, camera names, and kinematics.
	 */
	public final class RobotMap {


		// Camera IDs. this is for individual camera-threads, but there's only one so its fine
		public static enum CameraName {
			slaveFront, masterFront
		}

		/**
		 * PodConfig holds configuration for a single swerve pod.
		 * Includes motor IDs, encoder offsets, and position.
		 */
		/**
		 * PodConfig holds configuration for a single robot pod.
		 */
        public static final class PodConfig {
			public final int azimuthID;
			public final int driveID;
			public final int encoderID;
			public final double encoderOffset;
			public final Translation2d position;

			public static final boolean motorsBrake = true;

			public final int ampLimit;
			public final double maxOutput;
			public final double rampRate;

			public final double kP;
			public final double kI;
			public final double kD;
			public final double kS;
			public final double kV;
			public final double kA;

			public PodConfig(int azimuthID, int driveID, int encoderID, double encoderOffset, Translation2d podPosition) {
				this.azimuthID = azimuthID;
				this.driveID = driveID;
				this.encoderID = encoderID;
				this.encoderOffset = encoderOffset;
				this.position = podPosition;
				kP = 1;
				kI = 0.0;
				kD = 0.2;
				kS = 0;
				kV = 0;
				kA = 0;
				ampLimit = 80;
				maxOutput = 1;
				rampRate = 0.2;
			}


			public PodConfig(PodConfig podConfig, double kP, double kI, double kD, double kS, double kV, double kA) {
				this.azimuthID = podConfig.azimuthID;
				this.driveID = podConfig.driveID;
				this.encoderID = podConfig.encoderID;
				this.encoderOffset = podConfig.encoderOffset;
				this.position = podConfig.position;
				this.kP = kP;
				this.kI = kI;
				this.kD = kD;
				this.kS = kS; 
				this.kV = kV;
				this.kA = kA;
				ampLimit = podConfig.ampLimit;
				maxOutput = podConfig.maxOutput;
				rampRate = podConfig.rampRate;
			}

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
		public static final Translation2d centerOfMasterToTag = new Translation2d(0.16, 0.0); // meters, distance from the center of the master robot to the tag
		public static final Transform3d SLAVE_CAMERA_LOCATION = new Transform3d(new Translation3d(0.15, 0.0, 0.0), new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));
	}


	public final class VisionConstants {
		//TODO: move vision stuff here

		//TODO: fix these
		public static final ArrayList<Integer> RobotTagIDs = new ArrayList<>(Arrays.asList(17, 18, 19, 20));
		public static final ArrayList<Integer> StationTagIDs = new ArrayList<>(Arrays.asList(9, 10, 11, 12));
		public static final ArrayList<Integer> WingTagIDs = new ArrayList<>(Arrays.asList(13, 14, 15, 16));
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
		public static final double gearRatio = 0.02; // gear ratio of the drivetrain

	}

	/**
	 * RobotConfig contains configuration for the robot's drive system and pods.
	 * Includes pod configs, kinematics, and reset logic.
	 */
	public class RobotConfig {
		public static final double driveMetersPerMotorRotation = 8.143/(Units.inchesToMeters(4)*Math.PI);//1/(Units.inchesToMeters(2) * Math.PI / 1.36); //Wheel Diameter M * PI / Enc Count Per Rev / Gear Ratio ((inverted))
        public static final double azimuthDriveSpeedMultiplier = 1.0/3.571;
        public static final double azimuthRadiansPerMotorRotation = 1/21.4286;

        public static final int pigeonID = 25;
        public static final PIDController gyroPID = new PIDController(0.046, 0d, 0.001);



		private static final double wheelBase = Units.inchesToMeters(21);
		private static final double trackWidth = Units.inchesToMeters(21);
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
			public SingleRobotConfig(PodConfig[] podConfigs, double kP, double kI, double kD, double kS, double kV, double kA) {
				this.PodConfigs = podConfigs;
				for (int i = 0; i < podConfigs.length; i++) {
					podConfigs[i] = new PodConfig(podConfigs[i], kP, kI, kD, kS, kV, kA);
				}
				this.drivetrainKinematics = new SwerveDriveKinematics(
					java.util.Arrays.stream(podConfigs)
						.map(pod -> pod.position)
						.toArray(Translation2d[]::new)
				);
			}
		}
		public static final SingleRobotConfig[] robotConfigs = new SingleRobotConfig[] { 
			new SingleRobotConfig(new PodConfig[] { // first (red) robot
				new PodConfig(8, 4, 24, 0.3184, new Translation2d(wheelBase / 2, -trackWidth / 2)), // BL
				new PodConfig(5, 9, 22, 0.7168, new Translation2d(-wheelBase / 2, trackWidth / 2)), // FR
				new PodConfig(10, 6, 21, 0.5891, new Translation2d(wheelBase / 2, trackWidth / 2)), // BR
				new PodConfig(7, 11, 23, 0.2756, new Translation2d(-wheelBase / 2, -trackWidth / 2)) // FL - +
			}, 100.0, 0.0, 0.5, 0.1, 2.66, 0.0),
			new SingleRobotConfig(new PodConfig[] { // second (green) robot
				new PodConfig(8, 4, 24, 0.0591, new Translation2d(wheelBase / 2, -trackWidth / 2)), // BL
				new PodConfig(5, 9, 22, 0.7627, new Translation2d(-wheelBase / 2, trackWidth / 2)), // FR
				new PodConfig(10, 6, 21, -0.6096, new Translation2d(wheelBase / 2, trackWidth / 2)), // BR
				new PodConfig(7, 11, 23, 0.3977, new Translation2d(-wheelBase / 2, -trackWidth / 2)) // FL - +
			}, 100.0, 0.0, 0.5, 0.1, 2.66, 0.0)
		};


        public static final double robotMaxSpeed = 12; // joystick multiplier in meters per second
        public static final double formationMaxRotationalSpeed = 0.6; //maximum rotational speed of the formation in rad/s


        // Azimuth Settings
        public static final boolean azimuthBrake = true;

        public static final int azimuthAmpLimit = 80;
        public static final double azimuthMaxOutput = 1;



        public static final double azimuthMotorRampRate = 0.0;
		public static final boolean encoderInvert = true;
		public static final boolean azimuthInvert = true; 

        // Drive Settings
        public static final double podMaxSpeed = 1;

        public static final boolean driveBrake = false;

        public static final int driveAmpLimit = 80;
        public static final int boostDriveLimit = 90;
        public static final double driveMotorRampRate = 0.0;

		public static final boolean driveInvert = false; 
	}
}
