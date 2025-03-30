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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoralConstants;

// This subsystem controls the two rollers on the coral manipulator.
public class CoralSubsystem extends SubsystemBase {
  private CoralState m_state = CoralState.OFF;

  private final SparkMax m_topCoralMotor = new SparkMax(CoralConstants.TOP_CORAL_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_bottomCoralMotor = new SparkMax(CoralConstants.BOTTOM_CORAL_MOTOR_ID, MotorType.kBrushless);
  
  // Amperage tracking variables
  private double[] m_amperageReadings = new double[CoralConstants.CORAL_AMPERAGE_SAMPLE_SIZE];
  private int m_readingIndex = 0;

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

    // Update amperage readings
    double currentAmperage = (m_topCoralMotor.getOutputCurrent() + m_bottomCoralMotor.getOutputCurrent()) / 2.0;
    m_amperageReadings[m_readingIndex] = currentAmperage;
    m_readingIndex = (m_readingIndex + 1) % CoralConstants.CORAL_AMPERAGE_SAMPLE_SIZE;
    
    // Display average amperage on dashboard
    SmartDashboard.putNumber("Coral Avg Amperage", calculateAverageAmperage());
    SmartDashboard.putBoolean("Has Coral", hasCoral());

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
   * @return A command that sets the coral state to INTAKE until the coral is no longer detected and then disables.
   */
  public Command coralIntakeCommand() {
    Command intake = new InstantCommand(() -> m_state = CoralState.INTAKE);
        // .andThen(new WaitUntilCommand(() -> hasCoral()))
        // .andThen(new InstantCommand(() -> m_state = CoralState.OFF));

    intake.addRequirements(this);
    return intake;
  }

  public Command coralOffCommand() {
    Command off = new InstantCommand(() -> m_state = CoralState.OFF);
    off.addRequirements(this);
    return off;
  }

  /**
   * Command to score coral.
   * @return A command that sets the coral state to SCORE until the coral is no longer detected and then disables.
   */
  public Command coralScoreCommand() {
    Command score = new InstantCommand(() -> m_state = CoralState.SCORE);
        // .andThen(new WaitUntilCommand(() -> !hasCoral()))
        // .andThen(new InstantCommand(() -> m_state = CoralState.OFF));

    score.addRequirements(this);
    return score;
  }

  /**
   * Method to determine if coral is present by monitoring motor amperage.
   * @return True if coral is present, false otherwise.
   */
  public boolean hasCoral() {
    double averageAmperage = calculateAverageAmperage();
    return averageAmperage > CoralConstants.CORAL_AMPERAGE_THRESHOLD;
  }
  
  /**
   * Calculates the average amperage over the sample window.
   * @return The average amperage reading.
   */
  private double calculateAverageAmperage() {
    double sum = 0;
    for (double reading : m_amperageReadings) {
      sum += reading;
    }
    return sum / CoralConstants.CORAL_AMPERAGE_SAMPLE_SIZE;
  }

  public enum CoralState {
    INTAKE,
    SCORE,
    OFF
  }
}
