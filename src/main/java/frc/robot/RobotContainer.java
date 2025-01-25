// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
  private final LEDCommand ledRSLSyncCommand;
  private final LEDCommand ledTeamColorCommand;
  private final LEDCommand ledRainbowCommand;
  private final LEDCommand ledOffCommand;

  private LEDSubsystem ledSubsystem;

  public RobotContainer() {
    configureBindings();

    ledSubsystem = new LEDSubsystem(0);
    
    ledRSLSyncCommand =
    new LEDCommand(ledSubsystem,LEDCommand.LEDPatterns.RSL_SYNC);

    ledTeamColorCommand =
    new LEDCommand(ledSubsystem, LEDCommand.LEDPatterns.TEAM_COLOR);

    ledRainbowCommand =
    new LEDCommand(ledSubsystem, LEDCommand.LEDPatterns.RAINBOW);

    ledOffCommand =
    new LEDCommand(ledSubsystem, LEDCommand.LEDPatterns.OFF);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    //TODO: implement teleop command for lights
    return Commands.print("No autonomous command configured");
  }

  public Command getTeleopCommand() {
    //TODO: implement teleop command for lights

    System.out.println("teleop command called");

    return ledRainbowCommand;

  }

}
