// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;

// This subsystem controls the arm that extends and retracts the algae manipulator.
public class ArmSubsystem extends SubsystemBase {
  private ArmState m_state = ArmState.HOLD_UP;
  private SparkMax m_armMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
  // private PIDController m_armPID = new PIDController(ArmConstants.ARM_KP,
  // ArmConstants.ARM_KI, ArmConstants.ARM_KD);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor.setCANTimeout(250);

    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.voltageCompensation(10);
    armMotorConfig.smartCurrentLimit(40);
    armMotorConfig.idleMode(IdleMode.kBrake);
    armMotorConfig.encoder.positionConversionFactor(1);

    m_armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_armMotor.getEncoder().setPosition(0);

    // m_armPID.setTolerance(1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Arm State", m_state.name());
//    SmartDashboard.putNumber("Arm Encoder", m_armMotor.getEncoder().getPosition());

    switch (m_state) {
      case UP:
        m_armMotor.set(ArmConstants.ARM_UP_DUTYCYCLE);
        // m_armPID.calculate(m_armMotor.getEncoder().getPosition(),
        // ArmConstants.ARM_UP_ANGLE);
        break;
      case DOWN:
        m_armMotor.set(ArmConstants.ARM_DOWN_DUTYCYCLE);
        // m_armPID.calculate(m_armMotor.getEncoder().getPosition(),
        // ArmConstants.ARM_DOWN_ANGLE);
        break;
      case HOLD_UP:
        m_armMotor.set(ArmConstants.ARM_HOLD_UP_DUTYCYCLE);
        break;
      case HOLD_DOWN:
        m_armMotor.set(ArmConstants.ARM_HOLD_DOWN_DUTYCYCLE);
        break;
    }

    // 32 m_armMotor.set(m_armPID.calculate(m_armMotor.getEncoder().getPosition()));
  }

  /**
   * Command to raise the arm.
   * 
   * @return A command that sets the arm state to UP.
   */
  public Command armUpCommand() {
    Command armUp = new InstantCommand(() -> m_state = ArmState.UP);
    armUp.addRequirements(this);
    return armUp;
  }

  /**
   * Command to lower the arm.
   * 
   * @return A command that sets the arm state to DOWN.
   */
  public Command armDownCommand() {
    Command armDown = new InstantCommand(() -> m_state = ArmState.DOWN);
    armDown.addRequirements(this);
    return armDown;
  }

  /**
   * Command to hold the arm at its limit (up/down) while idle (no buttons pressed)
   * 
   * @return A command that sets the arm state to HOLD_DOWN if it was moving down, or HOLD_UP if it was moving up.
   */
  public Command armHoldCommand() {
    Command armHold = new InstantCommand(() -> m_state = switch (m_state) {
      case DOWN, HOLD_DOWN -> ArmState.HOLD_DOWN;
      case UP, HOLD_UP -> ArmState.HOLD_UP;
    });
    armHold.addRequirements(this);
    return armHold;
  }

  public enum ArmState {
    UP,
    DOWN,
    HOLD_UP,
    HOLD_DOWN
  }
}