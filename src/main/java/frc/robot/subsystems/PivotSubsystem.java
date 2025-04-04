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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
    pivotMotorConfig.absoluteEncoder.positionConversionFactor(360);

    m_pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

<<<<<<< Updated upstream
    m_pivotPID.setTolerance(1);
    m_pivotPID.enableContinuousInput(0, 360);
=======
    m_pivotPID.setTolerance(3);
    m_pivotPID.enableContinuousInput(0, 360);

    SmartDashboard.putNumberArray("Set Pivot PID", new double[] {
      PivotConstants.PIVOT_KP,
      PivotConstants.PIVOT_KI,
      PivotConstants.PIVOT_KD
    });
    SmartDashboard.putNumberArray("Current Pivot PID", new double[] {
      m_pivotPID.getP(),
      m_pivotPID.getI(),
      m_pivotPID.getD(),
    });
>>>>>>> Stashed changes
  }

  @Override
  public void periodic() {
<<<<<<< Updated upstream
    SmartDashboard.putString("Pivot State", m_state.name());
    SmartDashboard.putNumber("Pivot Encoder", m_pivotMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("PID Error", m_pivotPID.getError());
=======
    setPID();
    SmartDashboard.putString("Pivot State (Name)", m_state.name());
    SmartDashboard.putNumber("Pivot Encoder", m_pivotMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("Pivot PID Setpoint", m_pivotPID.getSetpoint());
    SmartDashboard.putNumber("Pivot Derivative Error", m_pivotPID.getErrorDerivative());
    SmartDashboard.putNumber("Pivot PID Error", m_pivotPID.getError());
>>>>>>> Stashed changes

    switch (m_state) {
      case UP:
        m_pivotPID.calculate(m_pivotMotor.getAbsoluteEncoder().getPosition(), PivotConstants.PIVOT_UP_ANGLE);
        break;
      case DOWN:
        m_pivotPID.calculate(m_pivotMotor.getAbsoluteEncoder().getPosition(), PivotConstants.PIVOT_DOWN_ANGLE);
        break;
      case NEUTRAL:
        m_pivotPID.calculate(m_pivotMotor.getAbsoluteEncoder().getPosition(), PivotConstants.PIVOT_NEUTRAL_ANGLE);
        break;
      default:
        m_pivotMotor.set(0);
        break;
    }

    m_pivotMotor.set(MathUtil.clamp(m_pivotPID.calculate(m_pivotMotor.getAbsoluteEncoder().getPosition()), -0.1, 0.1));
  }

  /**
   * Command to pivot the pivot up.
   * @return A command that pivots the pivot up until it reaches the setpoint.
   */
  public Command pivotUpCommand() {
    Command pivotUp = new InstantCommand(() -> m_state = PivotState.UP)
        .andThen(new WaitUntilCommand(() -> m_pivotPID.atSetpoint())).andThen(new PrintCommand("Pivot Up Finished"));
    pivotUp.addRequirements(this);
    return pivotUp;
  }

<<<<<<< Updated upstream
=======
  public void setPID() {
    double[] SDPivotPID = SmartDashboard.getNumberArray("Set Pivot PID", new double[3]);

    m_pivotPID.setP(SDPivotPID[0]);
    m_pivotPID.setI(SDPivotPID[1]);
    m_pivotPID.setD(SDPivotPID[2]);

    SmartDashboard.putNumberArray("Current Pivot PID", new double[] {
      m_pivotPID.getP(),
      m_pivotPID.getI(),
      m_pivotPID.getD(),
    });
  }

>>>>>>> Stashed changes
  /**
   * Command to pivot the pivot down.
   * @return A command that pivots the pivot down until it reaches the setpoint.
   */
  public Command pivotDownCommand() {
    Command pivotDown = new InstantCommand(() -> m_state = PivotState.DOWN)
        .andThen(new WaitUntilCommand(() -> m_pivotPID.atSetpoint())).andThen(new PrintCommand("Pivot Down Finished"));
    pivotDown.addRequirements(this);
    return pivotDown;
  }

  /**
   * Command to pivot the pivot to neutral.
   * @return A command that pivots the pivot to the neutral setpoint.
   */
  public Command pivotNeutralCommand() {
    Command pivotNeutral = new InstantCommand(() -> m_state = PivotState.NEUTRAL)
        .andThen(new WaitUntilCommand(() -> m_pivotPID.atSetpoint())).andThen(new PrintCommand("Pivot Neutral Finished"));
    pivotNeutral.addRequirements(this);
    return pivotNeutral;
  }

  public enum PivotState {
<<<<<<< Updated upstream
    UP,
    DOWN,
    NEUTRAL
=======
    DOWN,
    NEUTRAL,
    UP,
>>>>>>> Stashed changes
  }
}
