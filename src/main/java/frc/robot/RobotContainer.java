// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
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

  public Command getDisabledCommand() {

    System.out.println("disabled command called");

    return ledOffCommand;

  }

  public Command getAutonomousCommand() {

    System.out.println("autonomous command called");

    return ledTeamColorCommand;
    
  }

  public Command getTeleopCommand() {

    System.out.println("teleop command called");

    return ledRainbowCommand;

  }

  public Command getTestCommand() {

    System.out.println("teleop command called");

    return ledRSLSyncCommand;

  }

}
