// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private CommandXboxController m_controller = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Trigger climb = m_controller.rightTrigger().onTrue(new InstantCommand(() -> {
      m_climberSubsystem.setState(ClimberState.CLIMB);
    }));
    
    Trigger unclimb = m_controller.leftTrigger().onTrue(new InstantCommand(() -> {
      m_climberSubsystem.setState(ClimberState.CLIMB);
    }));

    climb.or(unclimb).onFalse(new InstantCommand(() -> {
      m_climberSubsystem.setState(ClimberState.OFF);
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
