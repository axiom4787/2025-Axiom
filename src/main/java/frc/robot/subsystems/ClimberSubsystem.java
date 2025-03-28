package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

// This subsystem controls the climber arm.
public class ClimberSubsystem extends SubsystemBase {

  private ClimberState m_state = ClimberState.OFF;

  private final SparkMax climbMotor;

  public ClimberSubsystem() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    climbConfig
        .voltageCompensation(12)
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);

    climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    climbMotor.setCANTimeout(250);
    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Climber State", m_state.name());
    switch (m_state) {
      case OFF:
        climbMotor.set(0);
        break;
      case UP:
        climbMotor.set(ClimberConstants.CLIMBER_UP_DUTYCYCLE);
        break;
      case DOWN:
        climbMotor.set(ClimberConstants.CLIMBER_DOWN_DUTYCYCLE);
        break;
    }
  }

  /**
   * Sets the state of the climber arm.
   * 
   * @param desiredState The desired state of the climber arm. OFF, UP, or DOWN.
   */
  public void setState(ClimberState desiredState) {
    m_state = desiredState;
  }

  public enum ClimberState {
    OFF,
    UP,
    DOWN
  }
}