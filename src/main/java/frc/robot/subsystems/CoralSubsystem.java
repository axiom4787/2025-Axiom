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
  private CoralState m_state = CoralState.EMPTY; // hasCoral() ? CoralState.FULL : CoralState.EMPTY;

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

    // Our State Machine for Coral
    switch (m_state) {
      case INTAKE:
        // Spin the top and bottom coral motors to intake a coral
        m_topCoralMotor.set(CoralConstants.CORAL_INTAKE_DUTYCYCLE);
        m_bottomCoralMotor.set(CoralConstants.CORAL_INTAKE_DUTYCYCLE);
        //if (hasCoral()) {
        //  m_state = CoralState.FULL;
        //}
        System.out.println("Coral Intake");
        break;
      case SCORE:
        // Spin the top and bottom coral motors to outtake a coral
        m_topCoralMotor.set(CoralConstants.CORAL_SCORE_DUTYCYCLE);
        m_bottomCoralMotor.set(CoralConstants.CORAL_SCORE_DUTYCYCLE);
        //if (!hasCoral()) {
        //  m_state = CoralState.EMPTY;
        //}
        System.out.println("Coral Score");
        break;
      case EMPTY:
        // Disable our motors if we aren't using them
        m_topCoralMotor.set(0.0);
        m_bottomCoralMotor.set(0.0);
        break;
      case FULL:
        // When we are full, don't spin the motors. We already have the coral so we do not need to intake anything
        m_topCoralMotor.set(0.0);
        m_bottomCoralMotor.set(0.0);
        break;
    }
  }

  // Expose intaking and outtaking modes
  // Equivalent to SetState() in ElevatorSubsystem.java
  public void intake() {
    //if (!hasCoral()) {
      m_state = CoralState.INTAKE;
    //}
  }

  public void outtake() {
    //if (hasCoral()) {
      m_state = CoralState.SCORE;
    //}
  }

  /*/
  public boolean hasCoral() {
    // TODO: use time of flight to determine if coral is present
    return false;
  }
  /*/

  // Define what sates we can be in
  public enum CoralState {
    INTAKE,
    SCORE,
    EMPTY,
    FULL
  }
}
