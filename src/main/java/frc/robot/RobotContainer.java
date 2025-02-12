// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.AlgaeState;

public class RobotContainer {
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private CommandXboxController m_controller = new CommandXboxController(0);

  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
  }

  private void configureBindings() {
    Trigger algaeIntake = m_controller.a().onTrue(new InstantCommand(() -> {
      m_algaeSubsystem.setState(AlgaeState.INTAKE);
    }));

    Trigger algaeOuttake = m_controller.a().onTrue(new InstantCommand(() -> {
      m_algaeSubsystem.setState(AlgaeState.INTAKE);
    }));

    algaeIntake.or(algaeOuttake).onFalse(new InstantCommand(() -> {
      m_algaeSubsystem.setState(AlgaeState.OFF);
    }));

    // TODO: CORAL/ELEVATOR BINDINGS
  }

  public Command getTeleopCommand() {
    return null;
  }
}
