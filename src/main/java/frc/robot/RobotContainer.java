// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.nio.channels.Pipe.SourceChannel;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.sim.vision.Vision;
import frc.robot.sim.vision.VisionConstants;
import frc.robot.sim.vision.VisionIOLimelight;
import frc.robot.sim.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NetworkTablesReceiver;
import frc.robot.utils.NavGridCounter;

public class RobotContainer {
	private final CommandXboxController m_controller = new CommandXboxController(0);
	private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
	private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();

	private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
	private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  	private final NetworkTablesReceiver m_networkTablesReceiver = new NetworkTablesReceiver();
	private final Limelight m_limelight = new Limelight();
	private static final String CAMERA_0_NAME = "limelight-front";
	private static final String CAMERA_1_NAME = "limelight-back";
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
	private Vision vision;

	public RobotContainer() {
		// autoChooser = AutoBuilder.buildAutoChooser();
		// SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();
		vision = new Vision(
			(pose, time, stdDevs) -> m_driveSubsystem.addVisionMeasurement(pose, time),
				new VisionIOPhotonVisionSim(CAMERA_0_NAME, VisionConstants.robotToCamera0, m_driveSubsystem::getPose));

		// System.out.println(u_navGridAnalyzer.analyze());
	}

  /**
   * Configures {@link Trigger} objects for buttons on the xbox controller to run commands for the robot.
   */
  private void configureBindings() {
    // --- Elevator/Pivot Button Binds ---

    // Makes the robot ready to score a coral in L1/L2/L3 or intake from source
    // Pivots the pivot to neutral first, to make sure the coral manipulator doesn't get caught on the elevator carriage
    // Then brings the elevator to the correct setpoint
    Trigger gotoL1 = m_controller.povDown()
        .onTrue(m_pivotSubsystem.pivotNeutralCommand()
            .andThen(m_elevatorSubsystem.elevatorL1Command()));
    // For L2 and L3, the pivot pivots down at the end to face the manipulator toward the reef branch
    Trigger gotoL2 = m_controller.povLeft()
        .onTrue(m_pivotSubsystem.pivotNeutralCommand()
            .andThen(m_elevatorSubsystem.elevatorL2Command()).andThen(m_pivotSubsystem.pivotDownCommand()));
    Trigger gotoL3 = m_controller.povUp()
        .onTrue(m_pivotSubsystem.pivotNeutralCommand()
            .andThen(m_elevatorSubsystem.elevatorL3Command()).andThen(m_pivotSubsystem.pivotDownCommand()));
    // For source, the pivot pivots up at the end to face the manipulator toward the human player station
    Trigger gotoSource = m_controller.povRight()
        .onTrue(m_pivotSubsystem.pivotNeutralCommand()
            .andThen(m_elevatorSubsystem.elevatorSourceCommand()).andThen(m_pivotSubsystem.pivotUpCommand()));

    // Will be used once the Time of Flight is mounted. Runs the coral intake after getting the manipulator in position.
    // Trigger gotoSourceAndIntake = m_controller.povRight()
    // .onTrue(m_pivotSubsystem.pivotNeutralCommand()
    // .andThen(m_elevatorSubsystem.elevatorSourceCommand()).andThen(m_pivotSubsystem.pivotUpCommand())
    // .andThen(m_coralSubsystem.coralIntakeCommand()));

    // --- Coral Button Binds ---

    // Runs the coral manipulator to intake or score a coral.
    Trigger intakeCoral = m_controller.x().onTrue(m_coralSubsystem.coralIntakeCommand()); // Command ends when Time of Flight detects a coral
    Trigger scoreCoral = m_controller.y().onTrue(m_coralSubsystem.coralScoreCommand()); // Command ends when Time of Flight no longer detects a coral

    // --- Algae/Arm Button Binds ---

    // When the A button is pressed, the arm will extend, and the algae manipulator will intake until it detects an algae.
    // Retracts the arm at the end to pull the algae into the robot.
    // Trigger intakeAlgae = m_controller.a().onTrue(m_armSubsystem.armDownCommand()
    //     .andThen(m_algaeSubsystem.algaeIntakeCommand()).andThen(m_armSubsystem.armUpCommand()));
    // When the X button is pressed, the arm will retract (just in case, though it should already be retracted) and then outtake the algae to score.
    // Trigger scoreAlgae = m_controller.x()
    //     .onTrue(m_armSubsystem.armUpCommand().andThen(m_algaeSubsystem.algaeScoreCommand()));

    // --- Climber Button Binds ---

    // Runs the climber up or down when the right or left triggers are pressed, respectively.
    Trigger climberUp = m_controller.rightTrigger().onTrue(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.UP)));
    Trigger climberDown = m_controller.leftTrigger().onTrue(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.DOWN)));
    // If neither the right or the left trigger is being pressed, disable the climber.
    climberUp.or(climberDown).onFalse(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.OFF)));

    // --- Drive Button Binds ---

    // TODO: Add field/robot relative toggle.

    // Zeroes the gyro (sets the new "forward" direction to wherever the robot is facing) in field relative mode.
    Trigger gyroReset = m_controller.a().onTrue(new InstantCommand(m_driveSubsystem::zeroGyro));
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
    return m_driveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-m_controller.getLeftY(), DriveConstants.CONTROLLER_DEADBAND, 1),
        () -> MathUtil.applyDeadband(-m_controller.getLeftX(), DriveConstants.CONTROLLER_DEADBAND, 1),
        () -> MathUtil.applyDeadband(-m_controller.getRightX(), DriveConstants.CONTROLLER_DEADBAND, 1))
		.withName("TeleopCommand");
  }

  public Command getTestCommand() {
    return m_driveSubsystem.sysIdDriveMotorCommand();
  }

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
							Command pathCommand = m_driveSubsystem.createPathToPose2D(targetPose)
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
	 * Finds the starting pose using vision data.
	 */
	public void findStartingVisionPose() {
		m_driveSubsystem.findStartingVisionPose();
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
		return new Command() {
		};
	}

	/**
	 * Retrieves the DriveSubsystem instance.
	 *
	 * @return The DriveSubsystem.
	 */
	public DriveSubsystem getDriveSubsystem() {
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
}
