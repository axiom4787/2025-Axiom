package frc.robot.subsystems;

// Motor Controllers
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.PID;

// controls the coral manipulator motors
public class CoralSubsystem extends SubsystemBase {
  private CoralState m_state = CoralState.OFF;

  // Motor Controllers for Coral
  public static final SparkMax m_topCoralMotor = new SparkMax(CoralConstants.TOP_CORAL_MOTOR_ID, MotorType.kBrushless);
  public static final SparkMax m_bottomCoralMotor = new SparkMax(CoralConstants.BOTTOM_CORAL_MOTOR_ID,
      MotorType.kBrushless);

  public CoralSubsystem() {
    SparkMaxConfig coralMotorConfig = new SparkMaxConfig();

    coralMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    coralMotorConfig.encoder
        .positionConversionFactor(PID.CORAL_MOTOR_pCONV)
        .velocityConversionFactor(PID.CORAL_MOTOR_vCONV);
    coralMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(PID.CORAL_MOTOR_KP, PID.CORAL_MOTOR_KI, PID.CORAL_MOTOR_KD)
        .velocityFF(PID.CORAL_MOTOR_KFF, ClosedLoopSlot.kSlot1)
        .outputRange(PID.CORAL_MOTOR_MIN_OUTPUT, PID.CORAL_MOTOR_MAX_OUTPUT);

    m_topCoralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_bottomCoralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    m_topCoralMotor.set(0.0);
    m_bottomCoralMotor.set(0.0);

    switch (m_state) {
      case INTAKE:
        m_topCoralMotor.set(CoralConstants.CORAL_INTAKE_DUTYCYCLE);
        m_bottomCoralMotor.set(CoralConstants.CORAL_INTAKE_DUTYCYCLE);
        break;
      case SCORE:
        m_topCoralMotor.set(CoralConstants.CORAL_SCORE_DUTYCYCLE);
        m_bottomCoralMotor.set(CoralConstants.CORAL_SCORE_DUTYCYCLE);
        break;
      case OFF:
        m_topCoralMotor.set(0.0);
        m_bottomCoralMotor.set(0.0);
        break;
    }
  }

  public void setState(CoralState state) {
    m_state = state;
  }

  public enum CoralState {
    INTAKE,
    SCORE,
    OFF
  }
}
