// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
	private final double maximumSpeed = 5.74;
	private SwerveDrive swerveDrive;
	private Field2d m_field;
	private boolean m_fieldRelative, m_poseLocked;

	public DriveSubsystem() {
		// uses yagsl config files in /deploy/swerve to create a swerve drive
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
		File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
		try {
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
		swerveDrive.setHeadingCorrection(false);
		swerveDrive.setCosineCompensator(false); // to make movement smoother when strafing

		// displays the robot pose within field on dashboard
		m_field = new Field2d();
		SmartDashboard.putData(m_field);

		m_fieldRelative = true;
		m_poseLocked = false;
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (IOException | ParseException e) {
			throw new RuntimeException(e);
		}

		AutoBuilder.configure(
				this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				(speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
																		// RELATIVE ChassisSpeeds. Also optionally
																		// outputs individual module feedforwards
				new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
												// holonomic drive trains
						new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
				),
				config, // The robot configuration
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this // Reference to this subsystem to set requirements
		);
	}

	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void resetPose(Pose2d startingPose) {
		swerveDrive.resetOdometry(startingPose);
	}

	/**
	 * Resets the NavX gyro and the rotation of the robot pose to 0.
	 */
	public void zeroGyro() {
		System.out.println("gyro reset");
		swerveDrive.zeroGyro();
	}

	/**
	 * Toggles the robot between being locked and unlocked.
	 * In a locked state, all wheels point toward the center in an X shape, to
	 * prevent pushing from other bots.
	 * Robot behaves normally in unlocked state.
	 */
	public void togglePoseLocked() {
		m_poseLocked = !m_poseLocked;
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return swerveDrive.getRobotVelocity();
	}

	public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
		System.out.println(robotRelativeSpeeds.omegaRadiansPerSecond);
		swerveDrive.drive(robotRelativeSpeeds);
	}

	public void driveRobotRelative(double x, double y, double rot) {
		swerveDrive.drive(new Translation2d(x, y), rot, false, false);
	}

	/**
	 * Returns a command to drive the robot based on inputs ranging from -1 to 1
	 * (e.g. joystick axes)
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier angularRotation) {
		return run(() -> {
			if (m_poseLocked)
				swerveDrive.lockPose();
			else {
				// System.out.printf("angular velocity: %.2f\n",
				// MathUtil.applyDeadband(angularRotation.getAsDouble() *
				// swerveDrive.getMaximumAngularVelocity(), Constants.kTurningDeadband));
				swerveDrive.drive(
						new Translation2d(
								MathUtil.applyDeadband(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
										Constants.DriveConstants.CONTROLLER_DEADBAND),
								MathUtil.applyDeadband(translationY.getAsDouble() * swerveDrive.getMaximumVelocity(),
								Constants.DriveConstants.CONTROLLER_DEADBAND)),
						MathUtil.applyDeadband(angularRotation.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
						Constants.DriveConstants.CONTROLLER_DEADBAND),
						m_fieldRelative,
						false);
						
			}
		}).withName("Drive Command");
	}

	/**
	 * Returns the limelight's estimated bot pose, based on visible AprilTags.
	 * If no tags are visible, returns an empty Pose2d (0 x, 0 y, 0 radians).
	 */
	private Pose2d getVisionPose() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
		double[] values = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		return new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
	}

	/**
	 * Switches the robot's orientation between field and robot relative.
	 */
	public void toggleFieldRelative() {
		m_fieldRelative = !m_fieldRelative;
		SmartDashboard.putBoolean("Field Relative", m_fieldRelative);
	}

	/**
	 * Resets the robot's odometry to the limelight's estimated vision pose.
	 * Generally, should only be called at the start of teleop.
	 */
	public void findStartingVisionPose() {
		Pose2d visionPose = getVisionPose();
		if (visionPose.getX() != 0.0 && visionPose.getY() != 0.0)
			swerveDrive.resetOdometry(visionPose);
	}

	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}

	public Command getAutonomousCommand(String pathName) {
		// Create a path following command using AutoBuilder. This will also trigger
		// event markers.
		return new PathPlannerAuto(pathName);
	}

	@Override
	public void periodic() {
		// update bot pose based on ll vision estimate
		// Pose2d visionPose = getVisionPose();
		// if (visionPose.getX() != 0.0 && visionPose.getY() != 0.0)
		// 	swerveDrive.addVisionMeasurement(getVisionPose(), Timer.getFPGATimestamp());

		// logging
		SmartDashboard.putBoolean("Field Relative", m_fieldRelative);
		SmartDashboard.putBoolean("Pose Locked", m_poseLocked);
		m_field.setRobotPose(getPose());

	}
}