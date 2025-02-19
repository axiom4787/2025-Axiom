// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final DriveSubsystem m_driveSubsystem;

  public RobotContainer() {
    m_driveSubsystem = new DriveSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    Trigger gyroReset = m_controller.a();
    gyroReset.onTrue(new InstantCommand(m_driveSubsystem::zeroGyro));
  }

  public Command getTeleopCommand() {
    return m_driveSubsystem.driveCommand(
      () -> -MathUtil.applyDeadband(m_controller.getLeftY(), DriveConstants.CONTROLLER_DEADBAND, 1), 
      () -> -MathUtil.applyDeadband(m_controller.getLeftX(), DriveConstants.CONTROLLER_DEADBAND, 1), 
      () -> -MathUtil.applyDeadband(m_controller.getRightX(), DriveConstants.CONTROLLER_DEADBAND, 1));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getTestCommand() {
    return null;
  }
}
