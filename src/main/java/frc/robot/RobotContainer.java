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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.AlgaeState;
import frc.robot.subsystems.CoralSubsystem.CoralState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import swervelib.parser.json.modules.DriveConversionFactorsJson;

public class RobotContainer {
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private CommandXboxController m_controller = new CommandXboxController(0);

  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
  }

  private void configureBindings() {
    Trigger algaeIntake = m_controller.a().onTrue(new InstantCommand(() -> {
      m_algaeSubsystem.setAlgaeState(AlgaeState.INTAKE);
    }));

    Trigger algaeUp = m_controller.b().onTrue(new InstantCommand(() -> {
      m_algaeSubsystem.setAlgaeState(AlgaeState.UP);
    }));

    Trigger algaeOuttake = m_controller.x().onTrue(new InstantCommand(() -> {
      m_algaeSubsystem.setAlgaeState(AlgaeState.OUTTAKE);
    }));


    Trigger armToL1 = m_controller.povDown().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L1);
      }), new InstantCommand(() -> {
        m_coralSubsystem.setCoralState(CoralState.OFF);
      })));
    Trigger armToL2 = m_controller.povLeft().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L2);
      }), new InstantCommand(() -> {
        m_coralSubsystem.setCoralState(CoralState.OFF);
      })));
    Trigger armToL3 = m_controller.povUp().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L3);
      }), new InstantCommand(() -> {
        m_coralSubsystem.setCoralState(CoralState.OFF);
      })));
    Trigger armToSource = m_controller.povRight().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.SOURCE);
      }), new InstantCommand(() -> {
        m_coralSubsystem.setCoralState(CoralState.OFF);
      })));

    Trigger coralIntake = m_controller.y().onTrue(new InstantCommand(() -> {
      m_coralSubsystem.setCoralState(CoralState.SCORE);
    }));
    Trigger coralOuttake = m_controller.povRight().onTrue(new InstantCommand(() -> {
      m_coralSubsystem.setCoralState(CoralState.INTAKE);
    }));

    Trigger gyroReset = m_controller.a().onTrue(new InstantCommand(m_driveSubsystem::zeroGyro));
  }

  public Command getTeleopCommand() {
    return m_driveSubsystem.driveCommand(
      () -> -MathUtil.applyDeadband(m_controller.getLeftY(), DriveConstants.CONTROLLER_DEADBAND, 1), 
      () -> -MathUtil.applyDeadband(m_controller.getLeftX(), DriveConstants.CONTROLLER_DEADBAND, 1), 
      () -> -MathUtil.applyDeadband(m_controller.getRightX(), DriveConstants.CONTROLLER_DEADBAND, 1));
  }

  public Command getTestCommand()
  {
    return m_driveSubsystem.sysIdDriveMotorCommand();
  }
}
