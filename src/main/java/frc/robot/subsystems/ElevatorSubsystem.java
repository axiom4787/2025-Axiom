package frc.robot.subsystems;

// Motor Controllers
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PID;

// controls the elevator and arm rotation motors
public class ElevatorSubsystem extends SubsystemBase {

  public ElevatorState m_state = ElevatorState.L1;

  // Motor Controllers for Coral
  private final SparkMax m_coralArmMotor = new SparkMax(ElevatorConstants.CORAL_ARM_MOTOR_ID,
      MotorType.kBrushless);
  private final SparkMax m_elevatorMotorLeft = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_L,
      MotorType.kBrushless);
  private final SparkMax m_elevatorMotorRight = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_R,
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

    SparkMaxConfig elevatorMotorLeftConfig = new SparkMaxConfig();

    elevatorMotorLeftConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    elevatorMotorLeftConfig.encoder
        .positionConversionFactor(PID.ELEVATOR_pCONV)
        .velocityConversionFactor(PID.ELEVATOR_vCONV);
    elevatorMotorLeftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(PID.ELEVATOR_KP, PID.ELEVATOR_KI, PID.ELEVATOR_KD)
        .velocityFF(PID.ELEVATOR_KFF, ClosedLoopSlot.kSlot1)
        .outputRange(PID.ELEVATOR_KMIN_OUTPUT, PID.ELEVATOR_KMAX_OUTPUT);

    m_elevatorMotorLeft.configure(elevatorMotorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig elevatorMotorRightConfig = new SparkMaxConfig();

    elevatorMotorRightConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .follow(m_elevatorMotorLeft);
    elevatorMotorRightConfig.encoder
        .positionConversionFactor(PID.ELEVATOR_pCONV)
        .velocityConversionFactor(PID.ELEVATOR_vCONV);
    elevatorMotorRightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(PID.ELEVATOR_KP, PID.ELEVATOR_KI, PID.ELEVATOR_KD)
        .velocityFF(PID.ELEVATOR_KFF, ClosedLoopSlot.kSlot1)
        .outputRange(PID.ELEVATOR_KMIN_OUTPUT, PID.ELEVATOR_KMAX_OUTPUT);

    m_elevatorMotorRight.configure(elevatorMotorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    SmartDashboard.putString("Elevator State", m_state.name());
    SmartDashboard.putNumber("L Elevator Motor Pos Encoder", m_elevatorMotorLeft.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("R Elevator Motor Pos Encoder", m_elevatorMotorRight.getAbsoluteEncoder().getPosition());

    switch (m_state) {
      case SOURCE:
        m_elevatorMotorLeft.getClosedLoopController().setReference(ElevatorConstants.ELEVATOR_SOURCE_POSITION, ControlType.kPosition);
        m_coralArmMotor.getClosedLoopController().setReference(ElevatorConstants.CORAL_ARM_NEUTRAL_ANGLE, ControlType.kPosition); // Need to test
        System.out.println("ElevatorSubsystem: SOURCE");
        break;
      case L1:
        m_elevatorMotorLeft.getClosedLoopController().setReference(ElevatorConstants.ELEVATOR_L1_POSITION, ControlType.kPosition);
        m_coralArmMotor.getClosedLoopController().setReference(ElevatorConstants.CORAL_ARM_DOWN_ANGLE, ControlType.kPosition); // Need to test
        System.out.println("ElevatorSubsystem: L1");
        break;
      case L2:
        m_elevatorMotorLeft.getClosedLoopController().setReference(ElevatorConstants.ELEVATOR_L2_POSITION, ControlType.kPosition);
        m_coralArmMotor.getClosedLoopController().setReference(ElevatorConstants.CORAL_ARM_DOWN_ANGLE, ControlType.kPosition); // Need to test
        System.out.println("ElevatorSubsystem: L2");
        break;
      case L3:
        m_elevatorMotorLeft.getClosedLoopController().setReference(ElevatorConstants.ELEVATOR_L3_POSITION, ControlType.kPosition);
        m_coralArmMotor.getClosedLoopController().setReference(ElevatorConstants.CORAL_ARM_UP_ANGLE, ControlType.kPosition); // Need to test
        System.out.println("ElevatorSubsystem: L3");
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
