// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import subsystems.DriveSubsystem;

public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final DriveSubsystem m_driveSubsystem;

  public RobotContainer() {
    m_driveSubsystem = new DriveSubsystem();
    configureBindings();
  }

  private void configureBindings() {}

  public Command getTeleopCommand() {
    return m_driveSubsystem.driveCommand(() -> -m_controller.getLeftY()*0.2, () -> -m_controller.getLeftX()*0.2, () -> -m_controller.getRightX()*0.2);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
