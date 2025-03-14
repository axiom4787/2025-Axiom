// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class RobotContainer {
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();

  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private CommandXboxController m_controller = new CommandXboxController(0);
  // private CommandJoystick m_controller = new CommandJoystick(0);

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
    Trigger gyroReset = m_controller.start().onTrue(new InstantCommand(m_driveSubsystem::zeroGyro));
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

  public Command getTeleopCommand() {
    return m_driveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(-m_controller.getLeftY(), DriveConstants.CONTROLLER_DEADBAND, 1),
        () -> -MathUtil.applyDeadband(m_controller.getLeftX(), DriveConstants.CONTROLLER_DEADBAND, 1),
        () -> -MathUtil.applyDeadband(-m_controller.getRightX(), DriveConstants.CONTROLLER_DEADBAND, 1));
  }

  public Command getTestCommand() {
    return m_driveSubsystem.sysIdDriveMotorCommand();
  }
}
