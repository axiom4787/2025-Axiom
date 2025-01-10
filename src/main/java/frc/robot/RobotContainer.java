// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MIntake;

public class RobotContainer {
  private final MIntake mIntake = new MIntake();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Example: Bind a command to a button
  }

  public Command getTeleopCommand() {
    // Example command to run in teleop mode
    return new InstantCommand(() -> mIntake.mState = Constants.Machine.CoralOuttake, mIntake);
  }
}
