// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.AlgaeState;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();

  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  // private CommandXboxController m_controller = new CommandXboxController(0);
  // private CommandJoystick m_controller = new CommandJoystick(0);
  private CommandStadiaController m_controller = new CommandStadiaController(0);

  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
    // registerNamedCommands();
  }

  /**
   * Configures {@link Trigger} objects for buttons on the xbox controller to run commands for the robot.
   */
  private void configureBindings() {
    // --- Elevator/Pivot Button Binds ---

    // // Makes the robot ready to score a coral in L1/L2/L3 or intake from source
    // // Pivots the pivot to neutral first, to make sure the coral manipulator doesn't get caught on the elevator carriage
    // // Then brings the elevator to the correct setpoint
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

    // Trigger pivotDown = m_controller.povDown().onTrue(m_pivotSubsystem.pivotDownCommand());
    // Trigger pivotNeutral = m_controller.povLeft().onTrue(m_pivotSubsystem.pivotNeutralCommand());
    // Trigger pivotUp = m_controller.povUp().onTrue(m_pivotSubsystem.pivotUpCommand());

    // Will be used once the Time of Flight is mounted. Runs the coral intake after getting the manipulator in position.
    // Trigger gotoSourceAndIntake = m_controller.povRight()
    // .onTrue(m_pivotSubsystem.pivotNeutralCommand()
    // .andThen(m_elevatorSubsystem.elevatorSourceCommand()).andThen(m_pivotSubsystem.pivotUpCommand())
    // .andThen(m_coralSubsystem.coralIntakeCommand()));

    // --- Coral Button Binds ---

    // Runs the coral manipulator to intake or score a coral.
    Trigger intakeCoral = m_controller.b().onTrue(m_coralSubsystem.coralIntakeCommand()); // Command ends when Time of Flight detects a coral
    Trigger scoreCoral = m_controller.y().onTrue(m_coralSubsystem.coralScoreCommand()); // Command ends when Time of Flight no longer detects a coral
    intakeCoral.or(scoreCoral).onFalse(m_coralSubsystem.coralOffCommand());

    // --- Algae/Arm Button Binds ---

    // Runs the algae rollers on intake/score when the a or x buttons are pressed, respectively.
    Trigger intakeAlgae = m_controller.a().onTrue(m_algaeSubsystem.algaeIntakeCommand());
    Trigger scoreAlgae = m_controller.x().onTrue(m_algaeSubsystem.algaeScoreCommand());
    // If neither button is being pressed, disable the algae roller.
    intakeAlgae.or(scoreAlgae).onFalse(m_algaeSubsystem.algaeOffCommand());

    // Runs the arm up or down when the right or left triggers are pressed, respectively.
    Trigger armUp = m_controller.rightTrigger().onTrue(m_armSubsystem.armUpCommand());
    Trigger armDown = m_controller.leftTrigger().onTrue(m_armSubsystem.armDownCommand());
    // If neither the right or the left trigger is being pressed, set the arm to idle.
    armUp.or(armDown).onFalse(m_armSubsystem.armHoldCommand());

    // --- Climber Button Binds ---

    // Runs the climber up or down when the right or left bumpers are pressed, respectively.
    Trigger climberUp = m_controller.rightBumper().onTrue(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.UP)));
    Trigger climberDown = m_controller.leftBumper().onTrue(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.DOWN)));
    // If neither the right or the left bumper is being pressed, disable the climber.
    climberUp.or(climberDown).onFalse(new InstantCommand(() -> m_climberSubsystem.setState(ClimberState.OFF)));

    // --- Drive Button Binds ---

    // TODO: Add field/robot relative toggle.

    // Zeroes the gyro (sets the new "forward" direction to wherever the robot is facing) in field relative mode.
    Trigger gyroReset = m_controller.frame().onTrue(new InstantCommand(m_driveSubsystem::zeroGyro));
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
  }

  public Command getTeleopCommand() {
    return m_driveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(-m_controller.getLeftY(), DriveConstants.CONTROLLER_DEADBAND, 1),
        () -> MathUtil.applyDeadband(-m_controller.getLeftX(), DriveConstants.CONTROLLER_DEADBAND, 1),
        () -> MathUtil.applyDeadband(-m_controller.getRightX(), DriveConstants.CONTROLLER_DEADBAND, 1));
  }

  public Command getTestCommand() {
    return m_driveSubsystem.sysIdDriveMotorCommand();
  }
}
