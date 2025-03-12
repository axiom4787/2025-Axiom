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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;

// This subsystem controls the arm that extends and retracts the algae manipulator.
public class ArmSubsystem extends SubsystemBase {
  private ArmState m_state = ArmState.UP;
  // private SparkMax m_armMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // m_armMotor.setCANTimeout(250);

    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.voltageCompensation(10);
    armMotorConfig.smartCurrentLimit(40);
    armMotorConfig.idleMode(IdleMode.kBrake);
    // m_armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Arm State", m_state.name());

    // switch (m_state) {
    //   case UP:
    //     m_armMotor.set(ArmConstants.ARM_UP_DUTYCYCLE);
    //     // TODO: logic for encoder
    //     break;
    //   case DOWN:
    //     m_armMotor.set(ArmConstants.ARM_DOWN_DUTYCYCLE);
    //     // TODO: logic for encoder
    //     break;
    // }
  }

  /**
   * Command to raise the arm.
   * @return A command that raises the arm until it reaches the retracted setpoint (currently runs for 3 seconds since our encoder is not attached yet)
   */
  public Command armUpCommand() {
    Command armUp = new InstantCommand(() -> m_state = ArmState.UP).andThen(new WaitCommand(3));
    // Command armUp = new InstantCommand(() -> m_state = ArmState.UP).andThen(new WaitUntilCommand(/* encoder */)); // code for when encoder is attached
    armUp.addRequirements(this);
    return armUp;
  }

  /**
   * Command to lower the arm.
   * @return A command that lowers the arm until it reaches the extended setpoint (currently runs for 3 seconds since our encoder is not attached yet)
   */
  public Command armDownCommand() {
    Command armDown = new InstantCommand(() -> m_state = ArmState.DOWN).andThen(new WaitCommand(3));
    // Command armDown = new InstantCommand(() -> m_state = ArmState.DOWN).andThen(new WaitUntilCommand(/* encoder */)); // code for when encoder is attached
    armDown.addRequirements(this);
    return armDown;
  }

  public enum ArmState {
    UP,
    DOWN,
  }
}
