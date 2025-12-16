// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.RobotConfig;
import frc.robot.commands.IndependentDrive;
import frc.robot.util.InputInterface;
import frc.robot.util.Tuple;

/**subsystem that runs on the master to send inputs to NT, and on slave to send localization data */
public class InputSender extends SubsystemBase {
	/** Creates a new SendInputs. */
	private XboxController controller;
	private SlewRateLimiter xLimiter = new SlewRateLimiter(1.0);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(1.0);
	private SlewRateLimiter rotationalLimiter = new SlewRateLimiter(3.0);
	
	// References to subsystems for slave localization data (only used on slave)
	private Swerve swerve;
	private PhotonVision photonVision;

	public InputSender() {
		controller = new XboxController(JoystickConstants.driverPort);		
	}
	
	/**
	 * Sets references to Swerve and PhotonVision subsystems for sending localization data.
	 * Should be called from slave robot.
	 * 
	 * @param swerve The Swerve subsystem
	 * @param photonVision The PhotonVision subsystem
	 */
	public void setLocalizationSubsystems(Swerve swerve, PhotonVision photonVision) {
		this.swerve = swerve;
		this.photonVision = photonVision;
	}

	@Override
	public void periodic() {
		if (Constants.IS_MASTER) {
			InputInterface.updateInputs(controller, DriverStation.isEnabled(), Timer.getFPGATimestamp(), grabJoystickVelocity());
		} else {
			InputInterface.updateInputs(IndependentDrive.masterOffset);
			
			// Send slave localization data to master
			if (swerve != null && photonVision != null) {
				Pose2d slaveOdometry = swerve.getPose();
				Tuple<Transform2d, Double> cameraUpdates = photonVision.getCameraUpdates();
				if (cameraUpdates != null) {
					InputInterface.updateSlaveLocalizationData(
						slaveOdometry,
						cameraUpdates.k,
						cameraUpdates.v
					);
				} else {
					// Send odometry even if no camera data
					InputInterface.updateSlaveLocalizationData(
						slaveOdometry,
						new Transform2d(),
						0.0
					);
				}
			}
		}
	}

	private Pose2d grabJoystickVelocity() {
        return new Pose2d(new Translation2d((MathUtil.applyDeadband(xLimiter.calculate(controller.getRightY()), JoystickConstants.deadband)), (MathUtil.applyDeadband(yLimiter.calculate(controller.getRightX()), JoystickConstants.deadband))).times(-0.05), new Rotation2d((MathUtil.applyDeadband(rotationalLimiter.calculate(controller.getLeftX()), JoystickConstants.deadband))).times(-0.2));
	}
}
