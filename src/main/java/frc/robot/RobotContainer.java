// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PivotSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final SendableChooser<Command> autoChooser;
	private final SendableChooser<Integer> timedAutoChooser;
	private final CommandXboxController m_controller = new CommandXboxController(0);
	private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
	private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();

	private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
	private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

	private boolean usePathPlanner = false;
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		registerNamedCommands();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Mode", autoChooser);
		timedAutoChooser = new SendableChooser<Integer>();
		timedAutoChooser.setDefaultOption("No Auto", 0);
		timedAutoChooser.addOption("Auto 1", 1);
		timedAutoChooser.addOption("Auto 2", 2);
		timedAutoChooser.addOption("Auto 3", 3);
		SmartDashboard.putData("Timed Auto Mode", timedAutoChooser);
		SmartDashboard.putBoolean("Use PathPlanner", usePathPlanner);

		configureBindings();
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
	}

	private void configureBindings() {
				// --- Elevator/Pivot Button Binds ---
		// Makes the robot ready to score a coral in L1/L2/L3 or intake from source
		// Pivots the pivot to neutral first, to make sure the coral manipulator doesn't
		// get caught on the elevator carriage
		// Then brings the elevator to the correct setpoint
		Trigger gotoL1 = m_controller.pov(180)
				.onTrue(m_pivotSubsystem.pivotNeutralCommand()
						.andThen(m_elevatorSubsystem.elevatorL1Command())
						.andThen(m_pivotSubsystem.pivotNeutralCommand()));
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
		// If neither the right or the left trigger is being pressed, set the arm to
		// idle.
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
		// Trigger gyroReset = m_controller.a().onTrue(new
		// InstantCommand(m_driveSubsystem::zeroGyro));

		// Swap robot/field relative
		m_controller.start().onTrue(Commands.runOnce(() -> {
			System.out.println("Swapping field relative/robot relative");
			m_driveSubsystem.toggleFieldRelative();
		}));
		m_controller.back().onTrue(Commands.runOnce(() -> {
			System.out.println("Zero Gyro");
			m_driveSubsystem.zeroGyro();
		}));
	}

	public void findStartingVisionPose() {
		m_driveSubsystem.findStartingVisionPose();
	}

	public Command getAutonomousCommand() {
		usePathPlanner = SmartDashboard.getBoolean("Use PathPlanner", false);
		return usePathPlanner ? m_driveSubsystem.getAutonomousCommand(autoChooser.getSelected().getName()) : getTimedAutoCommand();
	}

	public Command getTimedAutoCommand() {
		// Define speeds (adjust as necessary)
		double mediumForwardSpeed = 0.5; // meters per second
		double mediumSideSpeed = 0.5; // meters per second

		// Auto 1: Drive forward for 3 seconds at medium speed.
		Command auto1 = new SequentialCommandGroup(
				new RunCommand(
						() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(2, 0.0, 0.0)),
						m_driveSubsystem).withTimeout(10.0),
				new InstantCommand(() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)),
						m_driveSubsystem));

		// Auto 2: Drive forward for 1 second, then move arm to L1 position and shoot.
		Command auto2 = new SequentialCommandGroup(
				new RunCommand(
						() -> m_driveSubsystem.driveRobotRelative(
								new ChassisSpeeds(mediumForwardSpeed, 0.0, 0.0)),
						m_driveSubsystem).withTimeout(7.0),
				new InstantCommand(() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)),
						m_driveSubsystem),
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorL1Command()),
				m_coralSubsystem.coralScoreCommand(),
				new InstantCommand(() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)),
						m_driveSubsystem),
				new WaitCommand(1),
				m_coralSubsystem.coralOffCommand(),
				new RunCommand(
						() -> m_driveSubsystem.driveRobotRelative(
								new ChassisSpeeds(-mediumForwardSpeed, 0.0, 0.0)),
						m_driveSubsystem).withTimeout(2.0)); // Ensures the robot stops before executing the next
																// command

		// Auto 3: Wait 5 seconds, strafe right for 1 second, then perform Auto 2.
		Command auto3 = new SequentialCommandGroup(
				new WaitCommand(5.0),
				new RunCommand(() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, mediumSideSpeed, 0.0)),
						m_driveSubsystem).withTimeout(1.0),
				new InstantCommand(() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)),
						m_driveSubsystem),
				new RunCommand(
						() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(mediumForwardSpeed, 0.0, 0.0)),
						m_driveSubsystem).withTimeout(3.0),
				new InstantCommand(() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)),
						m_driveSubsystem),
				m_pivotSubsystem.pivotNeutralCommand().andThen(m_elevatorSubsystem.elevatorL1Command()),
				m_coralSubsystem.coralScoreCommand(),
				new InstantCommand(() -> m_driveSubsystem.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0)),
						m_driveSubsystem),
				new WaitCommand(1),
				m_coralSubsystem.coralOffCommand(),
				new RunCommand(
						() -> m_driveSubsystem.driveRobotRelative(
								new ChassisSpeeds(-mediumForwardSpeed, 0.0, 0.0)),
						m_driveSubsystem).withTimeout(2.0));
		Integer auto = timedAutoChooser.getSelected();

		if (auto == null) {
			auto = 0;
		}


		switch (auto) {

			case 1:
				System.out.println("Running Auto 1");
				return auto1;
			case 2:
				System.out.println("Running Auto 2");
				return auto2;
			case 3:
				System.out.println("Running Auto 3");
				return auto3;
			default:
				System.out.println("Invalid AUTO_MODE, defaulting to no auto.");
				return Commands.none();
		}
	}

	public Command getTeleopCommand() {
		return m_driveSubsystem.driveCommand(
				() -> -m_controller.getLeftY(),
				() -> -m_controller.getLeftX(),
				() -> -m_controller.getRightX())
				.withName("Drive Command");
	}
}
