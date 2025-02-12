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

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PID;

// controls the elevator and arm rotation motors
public class ElevatorSubsystem extends SubsystemBase {

  public ElevatorState m_state = ElevatorState.L1;

  // Motor Controllers for Coral
  public static final SparkMax m_coralArmMotor = new SparkMax(ElevatorConstants.CORAL_ARM_MOTOR_ID,
      MotorType.kBrushless);
  public static final SparkMax m_elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID,
      MotorType.kBrushless);

  public ElevatorSubsystem() {
    SparkMaxConfig coralArmMotorConfig = new SparkMaxConfig();

    coralArmMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    coralArmMotorConfig.encoder
        .positionConversionFactor(PID.ROT_CORAL_pCONV)
        .velocityConversionFactor(PID.ROT_CORAL_vCONV);
    coralArmMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(PID.ROT_CORAL_KP, PID.ROT_CORAL_KI, PID.ROT_CORAL_KD)
        .velocityFF(PID.ROT_CORAL_KFF, ClosedLoopSlot.kSlot1)
        .outputRange(PID.ROT_CORAL_KMIN_OUTPUT, PID.ROT_CORAL_KMAX_OUTPUT);

    m_coralArmMotor.configure(coralArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

    elevatorMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    elevatorMotorConfig.encoder
        .positionConversionFactor(PID.ELEVATOR_pCONV)
        .velocityConversionFactor(PID.ELEVATOR_vCONV);
    elevatorMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(PID.ELEVATOR_KP, PID.ELEVATOR_KI, PID.ELEVATOR_KD)
        .velocityFF(PID.ELEVATOR_KFF, ClosedLoopSlot.kSlot1)
        .outputRange(PID.ELEVATOR_KMIN_OUTPUT, PID.ELEVATOR_KMAX_OUTPUT);

    m_elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    switch (m_state) {
      case SOURCE:
        // TODO: elevator to source position, coral arm angled up
        break;
      case L1:
        // TODO: elevator to L1 position, coral arm angled down
        break;
      case L2:
        // TODO: elevator to L2 position, coral arm angled down
        break;
      case L3:
        // TODO: elevator to L3 position, coral arm angled down
        break;
    }
  }

  public void setState(ElevatorState state) {
    m_state = state;
  }

  public enum ElevatorState {
    SOURCE,
    L1,
    L2,
    L3
  }
}
