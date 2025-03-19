// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.DriveConstants;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

	/**
	 * Swerve drive object.
	 */
	private final SwerveDrive swerveDrive;

	/**
	 * Field visualization.
	 */
	private final Field2d m_field = new Field2d();

	/**
	 * Whether the drive is field-relative or robot-relative.
	 */
	private boolean fieldRelative = true;

	/**
	 * Whether the pose is locked for defense.
	 */
	private boolean poseLocked = false;

	/**
	 * Currently running path command.
	 */
	private Command currentPathCommand;

	/**
	 * PathPlanner configuration.
	 */
	private RobotConfig config;

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

	/**
	 * Initialize {@link SwerveDrive} with the directory provided.
	 */
	public DriveSubsystem() {
		// Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
		// objects being created.
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
		File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
		try {
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.MAX_SPEED_MS);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

		swerveDrive.setHeadingCorrection(true);
		swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
		swerveDrive.setAngularVelocityCompensation(true, true, 0.2);
		swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

		SmartDashboard.putData("Field", m_field);

		// Set initial pose
		swerveDrive.resetOdometry(new Pose2d(2, 2, new Rotation2d(0)));

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

		setupPathPlanner();
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

		// Process vision measurements if available
		Pose2d visionPose = getVisionPose();
		if (visionPose.getX() != 0.0 && visionPose.getY() != 0.0) {
			swerveDrive.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
		}

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

	@Override
	public void simulationPeriodic() {
		// Simulation updates if needed
	}

	/**
	 * Setup AutoBuilder for PathPlanner.
	 */
	public void setupPathPlanner() {
		try {
			config = RobotConfig.fromGUISettings();

			final boolean enableFeedforward = true;
			// Configure AutoBuilder last
			AutoBuilder.configure(
					this::getPose,
					// Robot pose supplier
					this::resetOdometry,
					// Method to reset odometry (will be called if your auto has a starting pose)
					this::getRobotVelocity,
					// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
					(speedsRobotRelative, moduleFeedForwards) -> {
						if (enableFeedforward) {
							swerveDrive.drive(
									speedsRobotRelative,
									swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
									moduleFeedForwards.linearForces());
						} else {
							swerveDrive.setChassisSpeeds(speedsRobotRelative);
						}
					},
					// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
					new PPHolonomicDriveController(
							// PPHolonomicController for path following
							new PIDConstants(5.0, 0.0, 0.0),
							// Translation PID constants
							new PIDConstants(5.0, 0.0, 0.0)
					// Rotation PID constants
					),
					config,
					// The robot configuration
					() -> {
						// Boolean supplier that controls when the path will be mirrored for the red
						// alliance
						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					},
					this
			// Reference to this subsystem to set requirements
			);
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}
	}

	/**
	 * Get the autonomous command for a predefined path.
	 *
	 * @param pathName PathPlanner path name.
	 * @return {@link PathPlannerAuto} path command.
	 */
	public Command getAutonomousCommand(String pathName) {
		return new PathPlannerAuto(pathName);
	}

	/** Adds a new timestamped vision measurement. */
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
		swerveDrive.addVisionMeasurement(visionRobotPoseMeters, Timer.getFPGATimestamp());
		// System.out.println("Vision measurement added.");
	}

	/**
	 * Use PathPlanner Path finding to go to a point on the field.
	 *
	 * @param pose Target {@link Pose2d} to go to.
	 * @return PathFinding command
	 */
	public Command driveToPose(Pose2d pose) {
		// Create the constraints to use while pathfinding
		PathConstraints constraints = new PathConstraints(
				swerveDrive.getMaximumChassisVelocity(), 4.0,
				swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

		// Since AutoBuilder is configured, we can use it to build pathfinding commands
		Command pathCommand = AutoBuilder.pathfindToPose(
				pose,
				constraints,
				0.0 // Goal end velocity in meters/sec
		);

		if (pathCommand != null) {
			currentPathCommand = pathCommand.andThen(() -> {
				System.out.println("[DriveSubsystem] Path command completed.");
				currentPathCommand = null;
			});

			currentPathCommand = currentPathCommand.andThen(new InstantCommand(() -> {
				System.out.println("[DriveSubsystem] Path command was interrupted.");
				currentPathCommand = null;
			}));
		}

		return currentPathCommand;
	}

	/**
	 * Drives the robot to a specified pose using PathPlanner pathfinding.
	 *
	 * @param pose The target pose to drive to.
	 * @return A command that drives the robot to the specified pose.
	 */
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
		} else {
			System.out.println("[DriveSubsystem] Warning: Path command creation failed.");
			return new InstantCommand(); // Return a no-op command instead of crashing
		}

		return pathCommand;
	}

	/**
	 * Create a path following command to follow a predefined path.
	 *
	 * @param trajectory The PathPlannerPath representing the path to follow.
	 * @return A Command that, when scheduled, will follow the specified path.
	 */
	public Command createPathFollowingCommand(PathPlannerPath trajectory) {
		PathConstraints constraints = new PathConstraints(
				swerveDrive.getMaximumChassisVelocity(),
				4.0,
				swerveDrive.getMaximumChassisAngularVelocity(),
				Units.degreesToRadians(720));

		Command pathCommand = AutoBuilder.pathfindThenFollowPath(trajectory, constraints);

		if (pathCommand != null) {
			pathCommand = pathCommand.andThen(() -> {
				System.out.println("[DriveSubsystem] Path following command completed.");
				currentPathCommand = null;
			});

			pathCommand = pathCommand.andThen(new InstantCommand(() -> {
				System.out.println("[DriveSubsystem] Path following command was interrupted.");
				currentPathCommand = null;
			}));
		}

		return pathCommand;
	}

	/**
	 * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
	 * PathPlanner.
	 *
	 * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
	 *                                  achieve.
	 * @return {@link Command} to run.
	 */
	public Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed) {
		try {
			SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
					RobotConfig.fromGUISettings(),
					swerveDrive.getMaximumChassisAngularVelocity());
			AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
					new SwerveSetpoint(swerveDrive.getRobotVelocity(),
							swerveDrive.getStates(),
							DriveFeedforwards.zeros(swerveDrive.getModules().length)));
			AtomicReference<Double> previousTime = new AtomicReference<>();

			return Commands.startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
					() -> {
						double newTime = Timer.getFPGATimestamp();
						SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
								robotRelativeChassisSpeed.get(),
								newTime - previousTime.get());
						swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
								newSetpoint.moduleStates(),
								newSetpoint.feedforwards().linearForces());
						prevSetpoint.set(newSetpoint);
						previousTime.set(newTime);
					});
		} catch (Exception e) {
			DriverStation.reportError(e.toString(), true);
			return Commands.none();
		}
	}

	/**
	 * Drive with 254's Setpoint generator; port written by PathPlanner.
	 *
	 * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
	 * @return Command to drive the robot using the setpoint generator.
	 */
	public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
		try {
			return driveWithSetpointGenerator(() -> {
				return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
			});
		} catch (Exception e) {
			DriverStation.reportError(e.toString(), true);
		}
		return Commands.none();
	}

	/**
	 * Command to characterize the robot drive motors using SysId
	 *
	 * @return SysId Drive Command
	 */
	public Command sysIdDriveMotorCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setDriveSysIdRoutine(
						new Config(),
						this, swerveDrive, 12),
				3.0, 5.0, 3.0);
	}

	/**
	 * Command to characterize the robot angle motors using SysId
	 *
	 * @return SysId Angle Command
	 */
	public Command sysIdAngleMotorCommand() {
		return SwerveDriveTest.generateSysIdCommand(
				SwerveDriveTest.setAngleSysIdRoutine(
						new Config(),
						this, swerveDrive),
				3.0, 5.0, 3.0);
	}

	/**
	 * Returns a Command that centers the modules of the SwerveDrive subsystem.
	 *
	 * @return a Command that centers the modules of the SwerveDrive subsystem
	 */
	public Command centerModulesCommand() {
		return run(() -> Arrays.asList(swerveDrive.getModules())
				.forEach(it -> it.setAngle(0.0)));
	}

	/**
	 * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
	 * object.
	 *
	 * @param kS the static gain of the feedforward
	 * @param kV the velocity gain of the feedforward
	 * @param kA the acceleration gain of the feedforward
	 */
	public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
		swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
	}

	/**
	 * Command to drive the robot using translative values and heading as angular
	 * velocity.
	 *
	 * @param translationX     Translation in the X direction. Cubed for smoother
	 *                         controls.
	 * @param translationY     Translation in the Y direction. Cubed for smoother
	 *                         controls.
	 * @param angularRotationX Angular velocity of the robot to set. Cubed for
	 *                         smoother controls.
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier angularRotationX) {
		return run(() -> {
			// Make the robot move
			ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
					new ChassisSpeeds(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
							translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
							Math.pow(angularRotationX.getAsDouble(), 3)
									* swerveDrive.getMaximumChassisAngularVelocity()),
					swerveDrive.getYaw());
			swerveDrive.drive(chassisSpeeds);
		}).withName("DriveCommand");
	}

	/**
	 * Get the swerve drive kinematics object.
	 *
	 * @return {@link SwerveDriveKinematics} of the swerve drive.
	 */
	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not
	 * need to be reset when calling this method.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by
	 * odometry.
	 *
	 * @return The robot's pose
	 */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but
	 * facing toward 0.
	 */
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	/**
	 * Checks if the alliance is red, defaults to false if alliance isn't available.
	 *
	 * @return true if the red alliance, false if blue. Defaults to false if none is
	 *         available.
	 */
	private boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
	}

	/**
	 * This will zero (calibrate) the robot to assume the current position is facing
	 * forward.
	 * If red alliance, rotate the robot 180 after the drivebase zero command.
	 */
	public void zeroGyroWithAlliance() {
		if (isRedAlliance()) {
			zeroGyro();
			// Set the pose 180 degrees
			resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
		} else {
			zeroGyro();
		}
	}

	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake True to set motors to brake mode, false for coast.
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Gets the current yaw angle of the robot, as reported by the swerve pose
	 * estimator in the underlying drivebase.
	 *
	 * @return The yaw angle
	 */
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	/**
	 * Get the chassis speeds based on controller input of 2 joysticks. One for
	 * speeds in which direction. The other for the angle of the robot.
	 *
	 * @param xInput   X joystick input for the robot to move in the X direction.
	 * @param yInput   Y joystick input for the robot to move in the Y direction.
	 * @param headingX X joystick which controls the angle of the robot.
	 * @param headingY Y joystick which controls the angle of the robot.
	 * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
		return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
				scaledInputs.getY(),
				headingX,
				headingY,
				getHeading().getRadians(),
				DriveConstants.MAX_SPEED_MS);
	}

	/**
	 * Get the chassis speeds based on controller input of 1 joystick and one angle.
	 *
	 * @param xInput X joystick input for the robot to move in the X direction.
	 * @param yInput Y joystick input for the robot to move in the Y direction.
	 * @param angle  The angle in as a {@link Rotation2d}.
	 * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
	 */
	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
		Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

		return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
				scaledInputs.getY(),
				angle.getRadians(),
				getHeading().getRadians(),
				DriveConstants.MAX_SPEED_MS);
	}

	/**
	 * Gets the current field-relative velocity (x, y and omega) of the robot
	 *
	 * @return A ChassisSpeeds object of the current field-relative velocity
	 */
	public ChassisSpeeds getFieldVelocity() {
		return swerveDrive.getFieldVelocity();
	}

	/**
	 * Gets the current velocity (x, y and omega) of the robot
	 *
	 * @return A {@link ChassisSpeeds} object of the current velocity
	 */
	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	/**
	 * Get the {@link SwerveController} in the swerve drive.
	 *
	 * @return {@link SwerveController} from the {@link SwerveDrive}.
	 */
	public SwerveController getSwerveController() {
		return swerveDrive.swerveController;
	}

	/**
	 * Get the {@link SwerveDriveConfiguration} object.
	 *
	 * @return The {@link SwerveDriveConfiguration} for the current drive.
	 */
	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return swerveDrive.swerveDriveConfiguration;
	}

	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	public void lock() {
		swerveDrive.lockPose();
	}

	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return The heading as a {@link Rotation2d} angle
	 */
	public Rotation2d getPitch() {
		return swerveDrive.getPitch();
	}

	/**
	 * Add a fake vision reading for testing purposes.
	 */
	public void addFakeVisionReading() {
		swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
	}

	/**
	 * Gets the swerve drive object.
	 *
	 * @return {@link SwerveDrive}
	 */
	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	/**
	 * Toggle between field-relative and robot-relative driving.
	 */
	public void toggleFieldRelative() {
		fieldRelative = !fieldRelative;
		SmartDashboard.putBoolean("Field Relative", fieldRelative);
	}

	/**
	 * Get the current vision pose, if available.
	 * 
	 * @return The vision-based Pose2d
	 */
	public Pose2d getVisionPose() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
		double[] values = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		return new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
	}

	/**
	 * Use vision to find and set the starting pose of the robot.
	 */
	public void findStartingVisionPose() {
		Pose2d visionPose = getVisionPose();
		if (visionPose.getX() != 0.0 && visionPose.getY() != 0.0) {
			swerveDrive.resetOdometry(visionPose);
		}
	}

	/**
	 * Checks if a path command is currently running.
	 *
	 * @return True if a path command is active, false otherwise.
	 */
	public boolean isPathCommandRunning() {
		return currentPathCommand != null && currentPathCommand.isScheduled();
	}
}