// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class DriveSubsystem extends SubsystemBase {
	private final SwerveDrive swerveDrive;
	private final Field2d m_field = new Field2d();

	private boolean fieldRelative, poseLocked;

	// Network Table Publishers
	private final StructPublisher<Pose3d> posePublisher;
	private final StructArrayPublisher<SwerveModuleState> moduleStatesPublisher;
	private final DoublePublisher yawPublisher;
	private final DoublePublisher pitchPublisher;
	private final DoublePublisher rollPublisher;
	private final DoubleArrayPublisher chassisSpeedsPublisher;
	private final DoublePublisher leftXPub;
	private final DoublePublisher leftYPub;
	private final DoublePublisher rightXPub;

	private Command currentPathCommand;

	private RobotConfig config;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
		try {
			swerveDrive = new SwerveParser(swerveJsonDirectory)
					.createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
		} catch (IOException i) {
			throw new RuntimeException(i);
		}

		swerveDrive.setHeadingCorrection(false);
		swerveDrive.setCosineCompensator(true);

		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
		SmartDashboard.putData("Field", m_field);

		// set initial pose
		swerveDrive.resetOdometry(new Pose2d(2, 2, new Rotation2d(0)));

		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
		}

		// Configure AutoBuilder
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

		// Initialize publishers
		posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("Drive/Pose", Pose3d.struct).publish();
		moduleStatesPublisher = NetworkTableInstance.getDefault()
				.getStructArrayTopic("Drive/ModuleStates", SwerveModuleState.struct).publish();
		yawPublisher = NetworkTableInstance.getDefault()
				.getDoubleTopic("Drive/Gyro/Yaw").publish();
		pitchPublisher = NetworkTableInstance.getDefault()
				.getDoubleTopic("Drive/Gyro/Pitch").publish();
		rollPublisher = NetworkTableInstance.getDefault()
				.getDoubleTopic("Drive/Gyro/Roll").publish();
		chassisSpeedsPublisher = NetworkTableInstance.getDefault()
				.getDoubleArrayTopic("Drive/ChassisSpeeds").publish();
		leftXPub = NetworkTableInstance.getDefault()
				.getDoubleTopic("Debug/Controller/LeftX").publish();
		leftYPub = NetworkTableInstance.getDefault()
				.getDoubleTopic("Debug/Controller/LeftY").publish();
		rightXPub = NetworkTableInstance.getDefault()
				.getDoubleTopic("Debug/Controller/RightX").publish();
	}

	@Override
	public void periodic() {
		// Update pose
		Pose2d pose2d = swerveDrive.getPose();
		posePublisher.set(new Pose3d(pose2d.getX(), pose2d.getY(), 0.0,
				new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians())));

		// Update module states
		moduleStatesPublisher.set(swerveDrive.getStates());

		// Update gyro data
		yawPublisher.set(swerveDrive.getYaw().getDegrees());
		pitchPublisher.set(swerveDrive.getPitch().getDegrees());
		rollPublisher.set(swerveDrive.getRoll().getDegrees());

		if (getVisionPose().getX() != 0.0 && getVisionPose().getY() != 0.0)
			swerveDrive.resetOdometry(getVisionPose());

		// Update chassis speeds
		ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
		chassisSpeedsPublisher.set(new double[] {
				speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond,
				speeds.omegaRadiansPerSecond
		});

		// Update field visualization
		m_field.setRobotPose(pose2d);
	}

	// Resets the robot pose (simply delegate to your swerveDrive)
	public void resetPose(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	// Returns the robot’s current chassis speeds (robot-relative)
	public ChassisSpeeds getRobotRelativeSpeeds() {
		return swerveDrive.getRobotVelocity();
	}

	// Drives the robot using robot-relative speeds and feedforwards
	// If you do not use the feedforwards, you can simply ignore them or log them.
	public void driveRobotRelative(ChassisSpeeds speeds) {
		// For example, simply command the drive:
		// (Adjust this call to match your swerveDrive.drive() API)
		swerveDrive.drive(speeds, true, new Translation2d(0, 0));
	}

	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void addVisionMeasurement(Pose2d visionPose) {
		swerveDrive.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
	}

	/**
	 * Checks if a path command is currently running.
	 *
	 * @return True if a path command is active, false otherwise.
	 */
	public boolean isPathCommandRunning() {
		return currentPathCommand != null && currentPathCommand.isScheduled();
	}

	public Pose2d getVisionPose() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
		double[] values = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		return new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
	}

	public void toggleFieldRelative() {
		fieldRelative = !fieldRelative;
	}

	public void findStartingVisionPose() {
		Pose2d visionPose = getVisionPose();
		if (visionPose.getX() != 0.0 && visionPose.getY() != 0.0)
			swerveDrive.resetOdometry(visionPose);
	}

	public Command createPathToPoint(double targetX, double targetY, double targetRotationDegrees) {
		// Define the target pose with the desired coordinates and holonomic rotation
		Pose2d targetPose = new Pose2d(
				targetX,
				targetY,
				Rotation2d.fromDegrees(targetRotationDegrees));

		// Define the constraints for the path
		PathConstraints constraints = new PathConstraints(
				5.74, // Max velocity in m/s
				1.0, // Max acceleration in m/s^2
				Units.degreesToRadians(540), // Max angular velocity in rad/s
				Units.degreesToRadians(720) // Max angular acceleration in rad/s^2
		);

		// Use AutoBuilder to create the pathfinding command
		Command pathCommand = AutoBuilder.pathfindToPose(
				targetPose,
				constraints,
				0.0 // Goal end velocity in meters/sec
		);

		if (pathCommand != null) {
			// Ensure the command requires the DriveSubsystem
			pathCommand = pathCommand.andThen(() -> {
				System.out.println("[DriveSubsystem] Path command completed.");
				currentPathCommand = null; // Clear the reference upon completion
			});

			// Handle interruptions
			pathCommand = pathCommand.andThen(new InstantCommand(() -> {
				System.out.println("[DriveSubsystem] Path command was interrupted.");
				currentPathCommand = null; // Clear the reference upon interruption
			}));
		}

		return pathCommand;
	}

	public Command createPathToPose2D(Pose2d targetPose) {
		// Define the constraints for the path
		PathConstraints constraints = new PathConstraints(
				5.74, // Max velocity in m/s
				1.0, // Max acceleration in m/s^2
				Units.degreesToRadians(540), // Max angular velocity in rad/s
				Units.degreesToRadians(720) // Max angular acceleration in rad/s^2
		);

		// Use AutoBuilder to create the pathfinding command
		Command pathCommand = AutoBuilder.pathfindToPose(
				targetPose,
				constraints,
				0.0 // Goal end velocity in meters/sec
		);

		if (pathCommand != null) {
			// Ensure the command requires the DriveSubsystem
			pathCommand = pathCommand.andThen(() -> {
				System.out.println("[DriveSubsystem] Path command completed.");
				currentPathCommand = null; // Clear the reference upon completion
			});

			// Handle interruptions
			pathCommand = pathCommand.andThen(new InstantCommand(() -> {
				System.out.println("[DriveSubsystem] Path command was interrupted.");
				currentPathCommand = null; // Clear the reference upon interruption
			}));
		}

		return pathCommand;
	}

	/**
	 * Creates a path-following command to follow a predefined path.
	 *
	 * @param trajectory The PathPlannerTrajectory representing the path to follow.
	 * @return A Command that, when scheduled, will follow the specified path.
	 */
	public Command createPathFollowingCommand(PathPlannerPath trajectory) {
		// Define the constraints for the path
		PathConstraints constraints = new PathConstraints(
				5.74, // Max velocity in m/s
				1.0, // Max acceleration in m/s^2
				Units.degreesToRadians(540), // Max angular velocity in rad/s
				Units.degreesToRadians(720) // Max angular acceleration in rad/s^2
		);
		// Use AutoBuilder to create the path-following command
		Command pathCommand = AutoBuilder.pathfindThenFollowPath(trajectory, constraints);

		if (pathCommand != null) {
			// Ensure the command requires the DriveSubsystem
			pathCommand = pathCommand.andThen(() -> {
				System.out.println("[DriveSubsystem] Path following command completed.");
				currentPathCommand = null; // Clear the reference upon completion
			});

			// Handle interruptions
			pathCommand = pathCommand.andThen(new InstantCommand(() -> {
				System.out.println("[DriveSubsystem] Path following command was interrupted.");
				currentPathCommand = null; // Clear the reference upon interruption
			}));
		}

		return pathCommand;
	}

	/**
	 * Command to drive the robot using translative values and heading as angular
	 * velocity.
	 *
	 * @param translationX     Translation in the X direction.
	 * @param translationY     Translation in the Y direction.
	 * @param angularRotationX Rotation of the robot to set
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier angularRotationX) {
		return new RunCommand(() -> {
			double xSpeed = translationX.getAsDouble();
			double ySpeed = translationY.getAsDouble();
			double rot = angularRotationX.getAsDouble();
			leftXPub.set(xSpeed);
			leftYPub.set(ySpeed);
			rightXPub.set(rot);
			// System.out.printf("Drive inputs - X: %.2f, Y: %.2f, Rot: %.2f%n", xSpeed,
			// ySpeed, rot);
			swerveDrive.drive(
					new Translation2d(xSpeed * swerveDrive.getMaximumChassisVelocity(),
							ySpeed * swerveDrive.getMaximumChassisVelocity()),
					rot * swerveDrive.getMaximumChassisAngularVelocity(),
					true,
					false,
					new Translation2d(-Units.inchesToMeters(Constants.DriveConstants.kGyroOffsetX), 0.0));
		}, this).withName("TeleopCommand");
	}

}