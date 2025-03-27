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

  private final SparkMax m_topCoralMotor = new SparkMax(CoralConstants.TOP_CORAL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_bottomCoralMotor = new SparkMax(CoralConstants.BOTTOM_CORAL_MOTOR_ID, MotorType.kBrushless);

  public CoralSubsystem() {
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
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Coral State", m_state.name());

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

  /**
   * Command to intake coral.
   * @return A command that sets the coral state to INTAKE until the coral is no longer detected and then disables. (currently runs for 1 second since our sensor is not attached yet)
   */
  public Command coralIntakeCommand() {
    Command intake = new InstantCommand(() -> m_state = CoralState.INTAKE);
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
    Command score = new InstantCommand(() -> m_state = CoralState.SCORE);
    // command for when sensor is attached:
    // Command outtake = new InstantCommand(() -> m_state = CoralState.SCORE)
    // .andThen(new WaitUntilCommand(() -> !hasCoral()))
    // .andThen(new InstantCommand(() -> m_state = CoralState.OFF));
    score.addRequirements(this);
    return score;
  }

  public Command coralOffCommand() {
    Command off = new InstantCommand(() -> m_state = CoralState.OFF);
    off.addRequirements(this);
    return off;
  }

  /**
   * Method to determine if coral is present with Time of Flight sensor.
   * @return True if coral is present, false otherwise.
   */
  // public boolean hasCoral() {
  //   // TODO: use time of flight to determine if coral is present
  //   return false;
  // }

  public enum CoralState {
    INTAKE,
    SCORE,
    OFF
  }
}
