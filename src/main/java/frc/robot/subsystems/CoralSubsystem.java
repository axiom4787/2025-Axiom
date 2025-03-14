package frc.robot.subsystems;

// Motor Controllers
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralConstants;

// This subsystem controls the two rollers on the coral manipulator.
public class CoralSubsystem extends SubsystemBase {
  private CoralState m_state = CoralState.OFF;

<<<<<<< HEAD
=======
  // Motor Controllers for Coral
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c
  private final SparkMax m_topCoralMotor = new SparkMax(CoralConstants.TOP_CORAL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_bottomCoralMotor = new SparkMax(CoralConstants.BOTTOM_CORAL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_pivotMotor = new SparkMax(CoralConstants.CORAL_ARM_MOTOR_ID, MotorType.kBrushless);

  private final PIDController m_PivotPID = new PIDController(PID.PIVOT_CORAL_KP, PID.PIVOT_CORAL_KI, PID.PIVOT_CORAL_KD);

  private SparkMaxConfig coralMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

  public CoralSubsystem() {
<<<<<<< HEAD
    SparkMaxConfig topCoralMotorConfig = new SparkMaxConfig();

    topCoralMotorConfig.inverted(true);
    topCoralMotorConfig.smartCurrentLimit(20);
    topCoralMotorConfig.idleMode(IdleMode.kCoast);

    SparkMaxConfig bottomCoralMotorConfig = new SparkMaxConfig();

    bottomCoralMotorConfig.idleMode(IdleMode.kCoast);
    bottomCoralMotorConfig.smartCurrentLimit(20);
    bottomCoralMotorConfig.follow(m_topCoralMotor, false);

    m_topCoralMotor.configure(topCoralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomCoralMotor.configure(bottomCoralMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
=======

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
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Coral State", m_state.name());

    switch (m_state) {
      case INTAKE:
        m_topCoralMotor.set(CoralConstants.CORAL_INTAKE_DUTYCYCLE);
        m_bottomCoralMotor.set(CoralConstants.CORAL_INTAKE_DUTYCYCLE);
<<<<<<< HEAD
=======
        m_PivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), CoralConstants.CORAL_ARM_UP_ANGLE);
        //if (hasCoral()) {
        //  m_state = CoralState.FULL;
        //}
        System.out.println("Coral Intake");
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c
        break;
      case SCORE:
        m_topCoralMotor.set(CoralConstants.CORAL_SCORE_DUTYCYCLE);
        m_bottomCoralMotor.set(CoralConstants.CORAL_SCORE_DUTYCYCLE);
<<<<<<< HEAD
=======
        m_PivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), CoralConstants.CORAL_ARM_DOWN_ANGLE);
        //if (!hasCoral()) {
        //  m_state = CoralState.EMPTY;
        //}
        System.out.println("Coral Score");
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c
        break;
      case OFF:
        m_topCoralMotor.set(0.0);
        m_bottomCoralMotor.set(0.0);
        m_PivotPID.calculate(m_pivotMotor.getEncoder().getPosition(), CoralConstants.CORAL_ARM_NEUTRAL_ANGLE);
        break;
    }
    m_pivotMotor.set(m_PivotPID.calculate(m_pivotMotor.getEncoder().getPosition()));
  }

<<<<<<< HEAD
  /**
   * Command to intake coral.
   * @return A command that sets the coral state to INTAKE until the coral is no longer detected and then disables. (currently runs for 1 second since our sensor is not attached yet)
   */
  public Command coralIntakeCommand() {
    Command intake = new InstantCommand(() -> m_state = CoralState.INTAKE)
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_state = CoralState.OFF));

    // command for when sensor is attached:
    // Command intake = new InstantCommand(() -> m_state = CoralState.INTAKE)
    // .andThen(new WaitUntilCommand(() -> hasCoral()))
    // .andThen(new InstantCommand(() -> m_state = CoralState.OFF));
    intake.addRequirements(this);
    return intake;
  }

  /**
   * Command to score coral.
   * @return A command that sets the coral state to SCORE until the coral is no longer detected and then disables. (currently runs for 1 second since our sensor is not attached yet)
   */
  public Command coralScoreCommand() {
    Command score = new InstantCommand(() -> m_state = CoralState.SCORE)
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> m_state = CoralState.OFF));

    // command for when sensor is attached:
    // Command outtake = new InstantCommand(() -> m_state = CoralState.SCORE)
    // .andThen(new WaitUntilCommand(() -> !hasCoral()))
    // .andThen(new InstantCommand(() -> m_state = CoralState.OFF));
    score.addRequirements(this);
    return score;
  }

  /**
   * Method to determine if coral is present with Time of Flight sensor.
   * @return True if coral is present, false otherwise.
   */
  // public boolean hasCoral() {
  //   // TODO: use time of flight to determine if coral is present
  //   return false;
  // }
=======
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
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c

  public enum CoralState {
    INTAKE,
    SCORE,
    OFF
  }
}
