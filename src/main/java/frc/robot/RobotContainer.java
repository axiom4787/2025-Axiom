// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.LEDPresets;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {

    private final LEDSubsystem m_ledSubsystem;
    private final Command ledCommand;

    public RobotContainer() {
        configureBindings();

        m_ledSubsystem = new LEDSubsystem(Constants.LEDConstants.LED_PORT);

        ledCommand = m_ledSubsystem.LEDCommand();
    }

    private void configureBindings() {
    }

    public Command getDisabledCommand() {

        System.out.println("disabled command called");

        m_ledSubsystem.usePattern(LEDPresets.LEDS_OFF);

        return new PrintCommand("DISABLED COMMAND") ;

    }

    public Command getAutonomousCommand() {

        System.out.println("autonomous command called");

        m_ledSubsystem.usePattern(LEDPresets.LEDS_TEAM_COLOR);

        return new PrintCommand("AUTO COMMAND");

    }

    public Command getTeleopCommand() {

        System.out.println("teleop command called");

        m_ledSubsystem.usePattern(LEDPresets.LEDS_RAINBOW);

        return new PrintCommand("TELEOP COMMAND");

    }

    public Command getTestCommand() {

        System.out.println("test command called");

        return new PrintCommand("TEST COMMAND");

    }

    public Command getLEDCommand() {
        return ledCommand;
    }

}
