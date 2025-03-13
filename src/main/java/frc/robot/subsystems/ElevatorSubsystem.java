package frc.robot.subsystems;

// Motor Controllers
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PID;

// controls the elevator and arm rotation motors
public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorState m_ElevatorState = ElevatorState.L1;

    private final SparkMax m_elevatorMotorR = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_R, MotorType.kBrushless);
    private final SparkMax m_elevatorMotorL = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_L, MotorType.kBrushless);
    private final PIDController m_elevatorPID = new PIDController(PID.ELEVATOR_KP, PID.ELEVATOR_KI, PID.ELEVATOR_KD);

    private SparkMaxConfig elevatorMotorR_Config = new SparkMaxConfig();
    private SparkMaxConfig elevatorMotorL_Config = new SparkMaxConfig();

    public ElevatorSubsystem() {
        m_elevatorPID.setTolerance(0.05);
  
        elevatorMotorL_Config.inverted(true);
        elevatorMotorL_Config.idleMode(IdleMode.kCoast);
        elevatorMotorL_Config.smartCurrentLimit(40);

        // do NOT trust the right motors encoder it needs to go to mental hospital
        elevatorMotorR_Config.idleMode(IdleMode.kCoast);
        elevatorMotorR_Config.smartCurrentLimit(40);
        elevatorMotorR_Config.follow(m_elevatorMotorL, true);

        m_elevatorMotorR.configure(elevatorMotorR_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevatorMotorL.configure(elevatorMotorL_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_elevatorMotorR.getEncoder().setPosition(0);
        m_elevatorMotorL.getEncoder().setPosition(0);

    }

  @Override
  public void periodic() {
    SmartDashboard.putString("Elevator State", m_ElevatorState.name());
    SmartDashboard.putNumber("Left Elevator Encoder", m_elevatorMotorL.getEncoder().getPosition());
    
    // dont bother looking at this tbh it wont be useful
    SmartDashboard.putNumber("Right Elevator Encoder", m_elevatorMotorR.getEncoder().getPosition());

    switch (m_ElevatorState) {
        case L1:
            m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition(), ElevatorConstants.ELEVATOR_L1_POSITION);
            break;
        case L2:
            m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition(), ElevatorConstants.ELEVATOR_L2_POSITION);
            break;
        case L3:
            m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition(), ElevatorConstants.ELEVATOR_L3_POSITION);
            break;
        case SOURCE:
            m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition(), ElevatorConstants.ELEVATOR_SOURCE_POSITION);
            break;
        default:
            m_elevatorMotorL.set(0);
            break;
    }

    m_elevatorMotorL.set(m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition()));

    // This method will be called once per scheduler run
  }
  public void setElevatorState(ElevatorState someState) {
    m_ElevatorState = someState;
  }
  public enum ElevatorState {
    L1,
    L2,
    L3,
    SOURCE
  }
}
