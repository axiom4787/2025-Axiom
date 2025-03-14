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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.PID;

// controls the coral manipulator motors
public class CoralSubsystem extends SubsystemBase {
  private CoralState m_state = CoralState.OFF; // hasCoral() ? CoralState.FULL : CoralState.EMPTY;

  // Motor Controllers for Coral
  private final SparkMax m_topCoralMotor = new SparkMax(CoralConstants.TOP_CORAL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_bottomCoralMotor = new SparkMax(CoralConstants.BOTTOM_CORAL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_pivotMotor = new SparkMax(CoralConstants.CORAL_ARM_MOTOR_ID, MotorType.kBrushless);

  private final PIDController m_PivotPID = new PIDController(PID.PIVOT_CORAL_KP, PID.PIVOT_CORAL_KI, PID.PIVOT_CORAL_KD);

  private SparkMaxConfig coralMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

  public CoralSubsystem() {

    coralMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);

    m_topCoralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomCoralMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);
    
    m_pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pivotMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putString("Coral State", m_state.name());

    switch (m_state) {
      case INTAKE:
        // m_topCoralMotor.set(CoralConstants.CORAL_INTAKE_DUTYCYCLE);
        m_bottomCoralMotor.set(CoralConstants.CORAL_INTAKE_DUTYCYCLE);
        m_PivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), CoralConstants.CORAL_ARM_UP_ANGLE);
        //if (hasCoral()) {
        //  m_state = CoralState.FULL;
        //}
        System.out.println("Coral Intake");
        break;
      case SCORE:
        // m_topCoralMotor.set(CoralConstants.CORAL_SCORE_DUTYCYCLE);
        m_bottomCoralMotor.set(CoralConstants.CORAL_SCORE_DUTYCYCLE);
        m_PivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), CoralConstants.CORAL_ARM_DOWN_ANGLE);
        //if (!hasCoral()) {
        //  m_state = CoralState.EMPTY;
        //}
        System.out.println("Coral Score");
        break;
      case OFF:
        // m_topCoralMotor.set(0.0);
        m_bottomCoralMotor.set(0.0);
        m_PivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), CoralConstants.CORAL_ARM_NEUTRAL_ANGLE);
        break;
    }
    m_pivotMotor.set(m_PivotPID.calculate(m_pivotMotor.getEncoder().getPosition()));
  }

  public void setCoralState(CoralState state) {
    m_state = state;
  }

  // public void intake() {
  //   //if (!hasCoral()) {
  //     m_state = CoralState.INTAKE;
  //   //}
  // }

  // public void outtake() {
  //   //if (hasCoral()) {
  //     m_state = CoralState.SCORE;
  //   //}
  // }

  // public void neutral() {
  //   m_state = CoralState.OFF;
  // }

  /*/
  public boolean hasCoral() {
    // TODO: use time of flight to determine if coral is present
    return false;
  }
  /*/

  public enum CoralState {
    INTAKE,
    SCORE,
    OFF
  }
}
