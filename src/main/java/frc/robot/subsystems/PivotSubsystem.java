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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.PivotConstants;

// This subsystem controls the motor that angles the coral manipulator up or down.
public class PivotSubsystem extends SubsystemBase {
  private PivotState m_state = PivotState.NEUTRAL;

  private SparkMax m_pivotMotor = new SparkMax(PivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
  private PIDController m_pivotPID = new PIDController(PivotConstants.PIVOT_KP, PivotConstants.PIVOT_KI, PivotConstants.PIVOT_KD);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

    pivotMotorConfig.inverted(false);
    pivotMotorConfig.smartCurrentLimit(20);
    pivotMotorConfig.idleMode(IdleMode.kBrake);
    pivotMotorConfig.encoder.positionConversionFactor(360);

    m_pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pivotPID.setTolerance(1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Pivot State", m_state.name());

    switch (m_state) {
      case UP:
        m_pivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), PivotConstants.PIVOT_UP_ANGLE);
        break;
      case DOWN:
        m_pivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), PivotConstants.PIVOT_DOWN_ANGLE);
        break;
      case NEUTRAL:
        m_pivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), PivotConstants.PIVOT_NEUTRAL_ANGLE);
        break;
      default:
        m_pivotMotor.set(0);
        break;
    }

    m_pivotMotor.set(m_pivotPID.calculate(m_pivotMotor.getEncoder().getPosition()));
  }

  /**
   * Command to pivot the pivot up.
   * @return A command that pivots the pivot up until it reaches the setpoint.
   */
  public Command pivotUpCommand() {
    Command pivotUp = new InstantCommand(() -> m_state = PivotState.UP)
        .andThen(new WaitUntilCommand(() -> m_pivotPID.atSetpoint()));
    pivotUp.addRequirements(this);
    return pivotUp;
  }

  /**
   * Command to pivot the pivot down.
   * @return A command that pivots the pivot down until it reaches the setpoint.
   */
  public Command pivotDownCommand() {
    Command pivotDown = new InstantCommand(() -> m_state = PivotState.DOWN)
        .andThen(new WaitUntilCommand(() -> m_pivotPID.atSetpoint()));
    pivotDown.addRequirements(this);
    return pivotDown;
  }

  /**
   * Command to pivot the pivot to neutral.
   * @return A command that pivots the pivot to the neutral setpoint.
   */
  public Command pivotNeutralCommand() {
    Command pivotNeutral = new InstantCommand(() -> m_state = PivotState.NEUTRAL)
        .andThen(new WaitUntilCommand(() -> m_pivotPID.atSetpoint()));
    pivotNeutral.addRequirements(this);
    return pivotNeutral;
  }

  public enum PivotState {
    UP,
    DOWN,
    NEUTRAL
  }
}
