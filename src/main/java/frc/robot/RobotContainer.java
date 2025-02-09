// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NetworkTablesReceiver;
import frc.robot.utils.NavGridCounter;


public class RobotContainer {
	private final CommandXboxController m_controller;
	private final DriveSubsystem m_driveSubsystem;
	private final NetworkTablesReceiver m_networkTablesReceiver = new NetworkTablesReceiver();
	private final Limelight m_limelight = new Limelight();
	// private final NavGridCounter u_navGridAnalyzer = new NavGridCounter("src/main/deploy/pathplanner/navgrid.json");

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

	public RobotContainer() {
		m_driveSubsystem = new DriveSubsystem();
		m_controller = new CommandXboxController(0);

		// autoChooser = AutoBuilder.buildAutoChooser();
		// SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();
		registerNamedCommands();

		// System.out.println(u_navGridAnalyzer.analyze());
	}

	private void configureBindings() {
		Trigger gyroReset = m_controller.button(1);
		gyroReset.onTrue(new InstantCommand(m_driveSubsystem::zeroGyro));

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

	/**
	 * Registers named commands with the AutoBuilder.
	 */
	private void registerNamedCommands() {
		// NamedCommands.registerCommand("shootNote", null);
		// NamedCommands.registerCommand("intakeNote", null);
	}

	public Command getTeleopCommand() {
		return m_driveSubsystem.driveCommand(
			() -> Math.abs(m_controller.getLeftY()) > 0.15 ? -m_controller.getLeftY() : 0,
			() -> Math.abs(m_controller.getLeftX()) > 0.15 ? -m_controller.getLeftX() : 0,
			() -> Math.abs(m_controller.getRightX()) > 0.15 ? -m_controller.getRightX() : 0)
			.withName("TeleopCommand");
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
                        // System.out.printf("[AutoPathThread] Joystick axes: %.2f, %.2f, %.2f%n", axis0, axis1, axis4);
                        return Math.abs(axis0) > Constants.ControllerConstants.kIdleDeadzone ||
                               Math.abs(axis1) > Constants.ControllerConstants.kIdleDeadzone ||
                               Math.abs(axis4) > Constants.ControllerConstants.kIdleDeadzone;
                    };

                    // If joystick is moved, cancel any auto-path command and schedule teleop immediately.
                    if (joystickMoved.getAsBoolean()) {
                        System.out.println("[AutoPathThread] Joystick moved detected!");
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
                    System.out.println("[AutoPathThread] Retrieved key from NT: " + currentKey);

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
                            Command pathCommand = m_driveSubsystem.createPathToPose2D(targetPose).andThen(getTeleopCommand());
                            currentPathCommand = pathCommand;
                            currentPathCommand.schedule();
                            System.out.println("[RobotContainer] Scheduled Auto Path Following Command to " + currentKey + ".");
                        } else {
                            System.out.println("[AutoPathThread] No pose mapped for key: " + currentKey);
                        }
                    }

                    try {
                        Thread.sleep(10); // Check every 10ms.
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