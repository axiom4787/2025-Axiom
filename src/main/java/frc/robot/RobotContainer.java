// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {

  private LEDSubsystem ledSubsystem;

  public RobotContainer() {
    configureBindings();

    ledSubsystem = new LEDSubsystem(0);
  }

  private void configureBindings() {}

  public Command getDisabledCommand() {

    System.out.println("disabled command called");

    return new LEDCommand(ledSubsystem, RobotStates.States.LEDS_OFF);

  }

  public Command getAutonomousCommand() {

    System.out.println("autonomous command called");

    return new PrintCommand("autonomous command");
  }

  public Command getTeleopCommand() {

    System.out.println("teleop command called");

    return new LEDCommand(ledSubsystem, RobotStates.States.LEDS_TEAM_COLOR);

  }

  public Command getTestCommand() {

    System.out.println("test command called");

    return new LEDCommand(ledSubsystem, RobotStates.States.LEDS_RAINBOW);

  }

}
