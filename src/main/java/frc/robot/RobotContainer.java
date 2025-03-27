// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import java.io.File;
import java.nio.channels.Pipe.SourceChannel;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.pathplanning.NetworkTablesReceiver;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MaxSwerveDriveSubsystem;
import frc.robot.utils.NavGridCounter;
import swervelib.math.SwerveMath;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer {
	// private final CommandPS5Controller m_controller = new CommandPS5Controller(0);
  private final CommandXboxController m_controller = new CommandXboxController(0);
	private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
	private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();

	private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
	private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
	private final MaxSwerveDriveSubsystem m_driveSubsystem = new MaxSwerveDriveSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

	private final NetworkTablesReceiver m_networkTablesReceiver = new NetworkTablesReceiver();
	private final Limelight m_limelight = new Limelight();
	private static final String CAMERA_0_NAME = "limelight-front";
	// private final NavGridCounter u_navGridAnalyzer = new
	// NavGridCounter("src/main/deploy/pathplanner/navgrid.json");

	// private final SendableChooser<Command> autoChooser;

	private Command currentPathCommand;
	private boolean tracking = false;
	private boolean autoPathEnabled = true;

	// Variables to store previous target coordinates
	private double previousTargetX = Double.NaN;
	private double previousTargetY = Double.NaN;

	// Thread for auto path following
	private Thread autoPathThread;

	private boolean keyDebounce = false;
	private String lastProcessedKey = "";
	private long lastKeyPressTime = 0;
	private final long KEY_DEBOUNCE_TIME = 500; // ms
	private Translation2d lastRobotPosition = new Translation2d();
	private final double POSITION_CHANGE_THRESHOLD = 0.3; // meters
	private Vision vision;

	// Simple boolean flag instead of AtomicBoolean
	private boolean isCurrentlyFollowingPath = false;

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {

		// double driveConversionFactor =
		// SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(3), 4.71);
		// System.out.println("[RobotContainer] Drive Conversion Factor: " +
		// driveConversionFactor);
		// Setup the PathPlanner auto chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        // Register named commands before configuring bindings
		NamedCommands.registerCommand("L1",
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorL1Command()));
		NamedCommands.registerCommand("L2",
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorL2Command())
						.andThen(m_pivotSubsystem.pivotDownCommand()));
		NamedCommands.registerCommand("L3",
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorL3Command())
						.andThen(m_pivotSubsystem.pivotDownCommand()));
		NamedCommands.registerCommand("Source",
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorSourceCommand())
						.andThen(m_pivotSubsystem.pivotUpCommand()));
		NamedCommands.registerCommand("Intake Coral", m_coralSubsystem.coralIntakeCommand());
		NamedCommands.registerCommand("Score Coral", m_coralSubsystem.coralScoreCommand());
		NamedCommands.registerCommand("Intake Algae", m_armSubsystem.armDownCommand()
				.andThen(m_algaeSubsystem.algaeIntakeCommand()).andThen(m_armSubsystem.armUpCommand()));
		NamedCommands.registerCommand("Score Algae",
				m_armSubsystem.armUpCommand().andThen(m_algaeSubsystem.algaeScoreCommand()));
        // registerNamedCommands();
		NamedCommands.registerCommand(";alskdjf", currentPathCommand);
        
        // Configure PathPlanner logging to display on Field2d
        
        // ...existing code...
        configureBindings();
        
        // Start periodic update of Limelight values on ShuffleBoard
        startLimelightUpdates();
	}

	/**
	 * Configures {@link Trigger} objects for buttons on the xbox controller to run
	 * commands for the robot.
	 */
	private void configureBindings() {
		// --- Elevator/Pivot Button Binds ---

		// Makes the robot ready to score a coral in L1/L2/L3 or intake from source
		// Pivots the pivot to neutral first, to make sure the coral manipulator doesn't
		// get caught on the elevator carriage
		// Then brings the elevator to the correct setpoint
		Trigger gotoL1 = m_controller.pov(180)
				.onTrue(m_pivotSubsystem.pivotNeutralCommand()
						.andThen(m_elevatorSubsystem.elevatorL1Command()));
		// For L2 and L3, the pivot pivots down at the end to face the manipulator
		// toward the reef branch
		Trigger gotoL2 = m_controller.pov(270)
				.onTrue(m_pivotSubsystem.pivotNeutralCommand()
						.andThen(m_elevatorSubsystem.elevatorL2Command()).andThen(m_pivotSubsystem.pivotDownCommand()));
		Trigger gotoL3 = m_controller.pov(0)
				.onTrue(m_pivotSubsystem.pivotNeutralCommand()
						.andThen(m_elevatorSubsystem.elevatorL3Command()).andThen(m_pivotSubsystem.pivotDownCommand()));
		// For source, the pivot pivots up at the end to face the manipulator toward the
		// human player station
		Trigger gotoSource = m_controller.pov(90)
				.onTrue(m_pivotSubsystem.pivotNeutralCommand()
						.andThen(m_elevatorSubsystem.elevatorSourceCommand())
						.andThen(m_pivotSubsystem.pivotUpCommand()));

		// Will be used once the Time of Flight is mounted. Runs the coral intake after
		// getting the manipulator in position.
		// Trigger gotoSourceAndIntake = m_controller.povRight()
		// .onTrue(m_pivotSubsystem.pivotNeutralCommand()
		// .andThen(m_elevatorSubsystem.elevatorSourceCommand()).andThen(m_pivotSubsystem.pivotUpCommand())
		// .andThen(m_coralSubsystem.coralIntakeCommand()));

		// --- Coral Button Binds ---

		// Runs the coral manipulator to intake or score a coral.
		Trigger intakeCoral = m_controller.b().onTrue(m_coralSubsystem.coralIntakeCommand()); // Command ends when Time
																								// of Flight detects a
																								// coral
		Trigger scoreCoral = m_controller.y().onTrue(m_coralSubsystem.coralScoreCommand()); // Command ends when Time of
																							// Flight no longer detects
																							// a coral
    intakeCoral.or(scoreCoral).onFalse(m_coralSubsystem.coralOffCommand());

		// --- Algae/Arm Button Binds ---

		// When the A button is pressed, the arm will extend, and the algae manipulator
		// will intake until it detects an algae.
		// Retracts the arm at the end to pull the algae into the robot.
		Trigger intakeAlgae = m_controller.a().onTrue(m_algaeSubsystem.algaeIntakeCommand());
		// When the X button is pressed, the arm will retract (just in case, though it
		// should already be retracted) and then outtake the algae to score.
		Trigger scoreAlgae = m_controller.x().onTrue(m_algaeSubsystem.algaeScoreCommand());
    intakeAlgae.or(scoreAlgae).onFalse(m_algaeSubsystem.algaeOffCommand());


    Trigger armUp = m_controller.rightTrigger().onTrue(m_armSubsystem.armUpCommand());
    Trigger armDown = m_controller.leftTrigger().onTrue(m_armSubsystem.armDownCommand());
    // If neither the right or the left trigger is being pressed, set the arm to idle.
    armUp.or(armDown).onFalse(m_armSubsystem.armHoldCommand());

		// --- Climber Button Binds ---

		// Runs the climber up or down when the right or left triggers are pressed,
		// respectively.
		Trigger climberUp = m_controller.rightBumper()
			.onTrue(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.UP)));
		Trigger climberDown = m_controller.leftBumper()
			.onTrue(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.DOWN)));
		// If neither the right or the left trigger is being pressed, disable the
		// climber.
		climberUp.or(climberDown).onFalse(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.OFF)));

		
		// --- Drive Button Binds ---

		// TODO: Add field/robot relative toggle.

		// Zeroes the gyro (sets the new "forward" direction to wherever the robot is
		// facing) in field relative mode.
		// Trigger gyroReset = m_controller.a().onTrue(new InstantCommand(m_driveSubsystem::zeroGyro));
		
		// Reset the pose and gyro based on Limelight data when the options button is pressed
		m_controller.start().onTrue(Commands.runOnce(() -> {
			System.out.println("Resetting pose using Limelight vision data");
			m_driveSubsystem.resetOdometryWithVision();
		}));
		m_controller.back().onTrue(Commands.runOnce(() -> {
			System.out.println("Zero Gyro");
			m_driveSubsystem.zeroGyro();
		}));
	}

	private void registerNamedCommands() {
		NamedCommands.registerCommand("L1",
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorL1Command()));
		NamedCommands.registerCommand("L2",
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorL2Command())
						.andThen(m_pivotSubsystem.pivotDownCommand()));
		NamedCommands.registerCommand("L3",
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorL3Command())
						.andThen(m_pivotSubsystem.pivotDownCommand()));
		NamedCommands.registerCommand("Source",
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorSourceCommand())
						.andThen(m_pivotSubsystem.pivotUpCommand()));
		NamedCommands.registerCommand("Intake Coral", m_coralSubsystem.coralIntakeCommand());
		NamedCommands.registerCommand("Score Coral", m_coralSubsystem.coralScoreCommand());
		NamedCommands.registerCommand("Intake Algae", m_armSubsystem.armDownCommand()
				.andThen(m_algaeSubsystem.algaeIntakeCommand()).andThen(m_armSubsystem.armUpCommand()));
		NamedCommands.registerCommand("Score Algae",
				m_armSubsystem.armUpCommand().andThen(m_algaeSubsystem.algaeScoreCommand()));

		// m_controller.square()
		// .onTrue(new InstantCommand(() -> {
		// autoPathEnabled = !autoPathEnabled;
		// System.out.println("[RobotContainer] Auto Path Enabled: " + autoPathEnabled);

		// if (autoPathEnabled) {
		// startAutoPathThread();
		// } else {
		// stopAutoPathThread();
		// }
		// }));
	}

	public Command getTeleopCommand() {
		return m_driveSubsystem.driveFieldRelativeCommand(
			() -> MathUtil.applyDeadband(-m_controller.getLeftY(), DriveConstants.CONTROLLER_DEADBAND, 1),
			() -> MathUtil.applyDeadband(-m_controller.getLeftX(), DriveConstants.CONTROLLER_DEADBAND, 1),
			() -> MathUtil.applyDeadband(-m_controller.getRightX(), DriveConstants.CONTROLLER_DEADBAND, 1))
			.withName("TeleopCommand")
		// return m_driveSubsystem.driveWithSetpointGeneratorFieldRelative(
		// 	() -> new ChassisSpeeds(
		// 		MathUtil.applyDeadband(-m_controller.getLeftY(), DriveConstants.CONTROLLER_DEADBAND) * Drivetrain.kMaxSpeed,
		// 		MathUtil.applyDeadband(-m_controller.getLeftX(), DriveConstants.CONTROLLER_DEADBAND) * Drivetrain.kMaxSpeed,
		// 		MathUtil.applyDeadband(-m_controller.getRightX(), DriveConstants.CONTROLLER_DEADBAND) * Drivetrain.kMaxAngularSpeed
		// 	)
		// )
			// .alongWith(new RunCommand(() -> {
			// 	// Use R2 for arm up (positive) and L2 for arm down (negative)
			// 	// Map triggers from 0-1 range to -1 to 1 range
			// 	double r2Value = m_controller.getR2Axis() * 2 - 1;
			// 	double l2Value = m_controller.getL2Axis() * 2 - 1;
			// 	System.out.println("R2: " + r2Value);
			// 	System.out.println("L2: " + l2Value);

			// 	double armPower = r2Value - l2Value;
			// 	m_armSubsystem.armManual(armPower * 0.4); // Scale to match arm's max power limit
			// }, m_armSubsystem))
			// .alongWith(
			// 	Commands.runOnce(() -> {
			// 		// Cross button (X) intakes algae
			// 		m_controller.cross().onTrue(m_algaeSubsystem.algaeIntakeCommand());
			// 		// Circle button (O) scores/outtakes algae
			// 		m_controller.circle().onTrue(m_algaeSubsystem.algaeScoreCommand());
			// 	})
			// )
			;
	}

	// public Command getTestCommand() {
	// 	return m_driveSubsystem.sysIdDriveMotorCommand();
	// }

	/**
	 * Starts the auto path following thread.
	 */
	public synchronized void startAutoPathThread() {
		if (autoPathThread == null || !autoPathThread.isAlive()) {
			System.out.println("[RobotContainer] Starting Auto Path Thread...");
			autoPathThread = new Thread(() -> {
				System.out.println("[RobotContainer] Auto Path Thread started.");
				while (autoPathEnabled) {
					// Define a BooleanSupplier to detect joystick movement on axes 0, 1, or 4.
					BooleanSupplier joystickMoved = () -> {
						double axis0 = m_controller.getLeftX();
						double axis1 = m_controller.getLeftY();
						double axis4 = m_controller.getRightX();
						// System.out.printf("[AutoPathThread] Joystick axes: %.2f, %.2f, %.2f%n",
						// axis0, axis1, axis4);
						return Math.abs(axis0) > Constants.ControllerConstants.kIdleDeadzone ||
								Math.abs(axis1) > Constants.ControllerConstants.kIdleDeadzone ||
								Math.abs(axis4) > Constants.ControllerConstants.kIdleDeadzone;
					};

					// If joystick is moved, cancel any auto-path command and schedule teleop
					// immediately.
					if (joystickMoved.getAsBoolean()) {
						// System.out.println("[AutoPathThread] Joystick moved detected!");
						if (currentPathCommand != null && currentPathCommand.isScheduled()) {
							currentPathCommand.cancel();
							System.out.println("[RobotContainer] Auto path command canceled due to joystick movement.");
							// Immediately schedule teleop command.
							Command teleopCmd = getTeleopCommand();
							teleopCmd.schedule();
							System.out.println("Rerunning Teleop cmd");
						}
						try {
							Thread.sleep(10);
						} catch (InterruptedException e) {
							System.out.println("[RobotContainer] Auto Path Thread interrupted during joystick check.");
							break;
						}
						continue;
					}

					// Retrieve the current key press from NetworkTables.
					String currentKey = m_networkTablesReceiver.getLastKeyPressed();
					// System.out.println("currentkey: " + currentKey);
					// System.out.println("[AutoPathThread] Retrieved key from NT: " + currentKey);

					// If the key has changed, schedule a new auto-path command.
					if (!currentKey.equals(lastProcessedKey)) {
						System.out.println("[AutoPathThread] New key detected: " + currentKey);
						lastProcessedKey = currentKey;

						if (currentPathCommand != null && currentPathCommand.isScheduled()) {
							currentPathCommand.cancel();
							System.out.println("[RobotContainer] Canceled existing path command.");
						}

						// Map the key to a target pose.
						Pose2d targetPose = getPoseFromKey(currentKey);
						if (targetPose != null) {
							System.out.println("[AutoPathThread] Creating path to " + targetPose);
							Command pathCommand = m_driveSubsystem.driveToPose(targetPose)
									.andThen(getTeleopCommand());
							currentPathCommand = pathCommand;
							currentPathCommand.schedule();
							System.out.println(
									"[RobotContainer] Scheduled Auto Path Following Command to " + currentKey + ".");
						} else {
							System.out.println("[AutoPathThread] No pose mapped for key: " + currentKey);
						}
					}

					try {
						Thread.sleep(100); // Check every 10ms.
					} catch (InterruptedException e) {
						System.out.println("[RobotContainer] Auto Path Thread interrupted during normal operation.");
						break;
					}
				}
				System.out.println("[RobotContainer] autothreadenabled is " + autoPathEnabled);
				System.out.println("[RobotContainer] Auto Path Thread stopped.");
			});
			autoPathThread.setDaemon(true);
			autoPathThread.setName("Auto Path Thread");
			autoPathThread.start();
			System.out.println("[RobotContainer] Started Auto Path Thread.");
		}
	}

	/**
	 * Stops the auto path following thread.
	 */
	private synchronized void stopAutoPathThread() {
		if (autoPathThread != null && autoPathThread.isAlive()) {
			autoPathThread.interrupt();
			autoPathThread = null;
			System.out.println("[RobotContainer] Auto Path Thread interrupted and stopped.");
		}

		// Cancel any existing path command
		if (currentPathCommand != null && currentPathCommand.isScheduled()) {
			currentPathCommand.cancel();
			System.out.println("[RobotContainer] Canceled existing path command.");
			currentPathCommand = null;
		}

		// Reset previous target coordinates
		previousTargetX = Double.NaN;
		previousTargetY = Double.NaN;
	}

	/**
	 * Toggles the tracking state.
	 */
	public void toggleTracking() {
		tracking = !tracking;
	}

	/**
	 * Retrieves the current tracking state.
	 *
	 * @return True if tracking is enabled, false otherwise.
	 */
	public boolean getTracking() {
		return tracking;
	}

	/**
	 * Finds the starting pose using vision data with error handling.
	 */
	public void findStartingVisionPose() {
		try {
			m_driveSubsystem.resetOdometryWithVision();
		} catch (Exception e) {
			System.err.println("Error finding starting vision pose: " + e.getMessage());
			// Continue with default pose rather than crashing
		}
	}

	/**
	 * Maps a key to its corresponding Pose2d based on the keyBindings.
	 *
	 * @param key The key string (e.g., "p_noteTopPose").
	 * @return The corresponding Pose2d, or null if no mapping exists.
	 */
	private Pose2d getPoseFromKey(String key) {
		HashMap<String, Pose2d> poseMap = new HashMap<>();
		poseMap.put("q", Constants.noteTopPose);
		poseMap.put("w", Constants.noteCenterPose);
		poseMap.put("e", Constants.noteBottomPose);

		return poseMap.get(key);
	}

	/**
	 * Retrieves the selected autonomous command.
	 *
	 * @return The selected autonomous command.
	 */
	public Command getAutonomousCommand() {
		// return autoChooser.getSelected();
		Command selectedAuto = autoChooser.getSelected();
        if (selectedAuto != null) {
            System.out.println("Running auto: " + autoChooser.getSelected().getName());
            return selectedAuto;
        } else {
            System.out.println("No auto selected - defaulting to nothing");
            return Commands.none();
        }
	}

	/**
	 * Retrieves the DriveSubsystem instance.
	 *
	 * @return The DriveSubsystem.
	 */
	public MaxSwerveDriveSubsystem getDriveSubsystem() {
		return m_driveSubsystem;
	}

	/**
	 * Retrieves the NetworkTablesReceiver instance.
	 *
	 * @return The NetworkTablesReceiver.
	 */
	public NetworkTablesReceiver getNetworkTablesReceiver() {
		return m_networkTablesReceiver;
	}

	public Limelight getLimelight() {
		return m_limelight;
	}
	
	/**
	 * Starts a thread to periodically update Limelight values on ShuffleBoard
	 */
	private void startLimelightUpdates() {
		// Create a thread that updates Limelight values on ShuffleBoard
		Thread limelightUpdateThread = new Thread(() -> {
			while (true) {
				try {
					// Get tx value directly from NetworkTables
					NetworkTable limelightTable = NetworkTableInstance.getDefault()
							.getTable("limelight-left");
					
					// Batch the network table access to reduce overhead
					double tv = limelightTable.getEntry("tv").getDouble(0);
					double tx = limelightTable.getEntry("tx").getDouble(0.0);
					
					// Put the tx value on ShuffleBoard
					SmartDashboard.putNumber("Limelight TX", tx);
					
					// Also display whether there's a valid target
					boolean hasTarget = tv >= 1.0;
					SmartDashboard.putBoolean("Limelight Has Target", hasTarget);
					
					// Sleep longer to reduce CPU usage - 250ms is still 4 updates per second
					Thread.sleep(250);
				} catch (Exception e) {
					System.err.println("Error in limelight thread: " + e.getMessage());
					try {
						Thread.sleep(1000); // On error, sleep longer before retrying
					} catch (InterruptedException ex) {
						Thread.currentThread().interrupt();
						break;
					}
				}
			}
		});
		
		limelightUpdateThread.setDaemon(true);
		limelightUpdateThread.setPriority(Thread.MIN_PRIORITY); // Lower priority to avoid interfering with critical robot functions
		limelightUpdateThread.setName("Limelight Update Thread");
		limelightUpdateThread.start();
		
		System.out.println("Started Limelight update thread");
	}
}
