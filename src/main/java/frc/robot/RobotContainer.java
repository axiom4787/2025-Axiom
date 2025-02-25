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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.AlgaeState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import swervelib.parser.json.modules.DriveConversionFactorsJson;

public class RobotContainer {
  // private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  // private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private CommandXboxController m_controller = new CommandXboxController(0);

  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
  }

  private void configureBindings() {
    // Trigger algaeIntake = m_controller.a().onTrue(new InstantCommand(() -> {
    //   m_algaeSubsystem.intake();
    // }));

    // Trigger algaeOuttake = m_controller.x().onTrue(new InstantCommand(() -> {
    //   m_algaeSubsystem.outtake();
    // }));


    Trigger armToL1 = m_controller.povDown().onTrue(new InstantCommand(() -> {
      m_elevatorSubsystem.setElevatorState(ElevatorState.L1);
    }));
    Trigger armToL2 = m_controller.povLeft().onTrue(new InstantCommand(() -> {
      m_elevatorSubsystem.setElevatorState(ElevatorState.L2);
    }));
    Trigger armToL3 = m_controller.povUp().onTrue(new InstantCommand(() -> {
      m_elevatorSubsystem.setElevatorState(ElevatorState.L3);
    }));
    // Trigger armToBase = m_controller.povUp().onTrue(new InstantCommand(() -> {
    //   m_elevatorSubsystem.setElevatorState(ElevatorState.BASE);
    // }));
    Trigger armToSource = m_controller.povRight().onTrue(new InstantCommand(() -> {
      m_elevatorSubsystem.setElevatorState(ElevatorState.SOURCE);
    }));

    // Trigger coralIntake = m_controller.y().onTrue(new InstantCommand(() -> {
    //   m_coralSubsystem.intake();
    // }));
    // Trigger coralOuttake = m_controller.y().onTrue(new InstantCommand(() -> {
    //   m_coralSubsystem.outtake();
    // }));
    // coralIntake.or(coralOuttake).negate().onTrue(new InstantCommand(() -> {
    //   m_coralSubsystem.stop();
    // }));

    Trigger gyroReset = m_controller.a();
    gyroReset.onTrue(new InstantCommand(m_driveSubsystem::zeroGyro));
  }

  public Command getTeleopCommand() {
    return m_driveSubsystem.driveCommand(
      () -> -MathUtil.applyDeadband(m_controller.getLeftY(), DriveConstants.CONTROLLER_DEADBAND, 1), 
      () -> -MathUtil.applyDeadband(m_controller.getLeftX(), DriveConstants.CONTROLLER_DEADBAND, 1), 
      () -> -MathUtil.applyDeadband(m_controller.getRightX(), DriveConstants.CONTROLLER_DEADBAND, 1));
  }
}
