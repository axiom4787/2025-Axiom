// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AlgaeConstants;

// This subsystem controls the roller on the algae manipulator.
public class AlgaeSubsystem extends SubsystemBase {
  private AlgaeState m_state = AlgaeState.OFF;
  // private SparkMax m_rollerMotor = new SparkMax(AlgaeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    // m_rollerMotor.setCANTimeout(250);

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.voltageCompensation(10);
    rollerMotorConfig.smartCurrentLimit(40);
    rollerMotorConfig.idleMode(IdleMode.kBrake);
    // m_rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Algae State", m_state.name());

    // switch (m_state) {
    //   case INTAKE:
    //     m_rollerMotor.set(AlgaeConstants.ALGAE_INTAKE_DUTYCYCLE);
    //     break;
    //   case SCORE:
    //     m_rollerMotor.set(AlgaeConstants.ALGAE_SCORE_DUTYCYCLE);
    //     break;
    //   case OFF:
    //     m_rollerMotor.set(0);
    // }
  }

  /**
   * Command to intake algae.
   * @return A command that sets the algae state to INTAKE until the algae is no longer detected and then disables. (currently runs for 3 seconds since our sensor is not attached yet)
   */
  public Command algaeIntakeCommand() {
    Command algaeIntake = new InstantCommand(() -> m_state = AlgaeState.INTAKE).andThen(new WaitCommand(3))
        .andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));
    
    // command for when sensor is attached:
    // Command algaeIntake = new InstantCommand(() -> m_state = AlgaeState.INTAKE)
    //     .andThen(new WaitUntilCommand(() -> hasAlgae())).andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));
    algaeIntake.addRequirements(this);
    return algaeIntake;
  }

  /**
   * Command to score algae.
   * @return A command that sets the algae state to SCORE until the algae is no longer detected and then disables. (currently runs for 3 seconds since our sensor is not attached yet)
   */
  public Command algaeScoreCommand() {
    Command algaeScore = new InstantCommand(() -> m_state = AlgaeState.SCORE).andThen(new WaitCommand(3))
        .andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));

    // command for when sensor is attached:
    // Command algaeScore = new InstantCommand(() -> m_state = AlgaeState.SCORE)
    //     .andThen(new WaitUntilCommand(() -> !hasAlgae())).andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));
    algaeScore.addRequirements(this);
    return algaeScore;
  }

  /**
   * Method to determine if algae is present with Time of Flight sensor.
   * @return True if algae is present, false otherwise.
   */
  // public boolean hasAlgae() {
  // // TODO: use time of flight to determine if algae is present
  // return false;
  // }

  public enum AlgaeState {
    INTAKE,
    SCORE,
    OFF,
  }
}
