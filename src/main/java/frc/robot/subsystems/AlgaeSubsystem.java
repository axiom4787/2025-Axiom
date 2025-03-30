// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AlgaeConstants;

// This subsystem controls the roller on the algae manipulator.
public class AlgaeSubsystem extends SubsystemBase {
  private AlgaeState m_state = AlgaeState.OFF;
  private SparkMax m_rollerMotor = new SparkMax(AlgaeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
  
  // Amperage tracking variables
  private double[] m_amperageReadings = new double[AlgaeConstants.ALGAE_AMPERAGE_SAMPLE_SIZE];
  private int m_readingIndex = 0;

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    m_rollerMotor.setCANTimeout(250);

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.voltageCompensation(10);
    rollerMotorConfig.smartCurrentLimit(40);
    rollerMotorConfig.idleMode(IdleMode.kBrake);
    m_rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Algae State", m_state.name());
    
    // Update amperage readings
    double currentAmperage = m_rollerMotor.getOutputCurrent();
    m_amperageReadings[m_readingIndex] = currentAmperage;
    m_readingIndex = (m_readingIndex + 1) % AlgaeConstants.ALGAE_AMPERAGE_SAMPLE_SIZE;
    
    // Display average amperage on dashboard
    SmartDashboard.putNumber("Algae Avg Amperage", calculateAverageAmperage());
    SmartDashboard.putBoolean("Has Algae", hasAlgae());
    
    switch (m_state) {
      case INTAKE:
        m_rollerMotor.set(AlgaeConstants.ALGAE_INTAKE_DUTYCYCLE);
        break;
      case SCORE:
        m_rollerMotor.set(AlgaeConstants.ALGAE_SCORE_DUTYCYCLE);
        break;
      case OFF:
        m_rollerMotor.set(0);
    }
  }

  /**
   * Command to intake algae.
   * @return A command that sets the algae state to INTAKE until the algae is detected and then disables.
   */
  public Command algaeIntakeCommand() {
    Command algaeIntake = new InstantCommand(() -> m_state = AlgaeState.INTAKE)
        .andThen(new WaitUntilCommand(() -> hasAlgae()))
        .andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));
    
    algaeIntake.addRequirements(this);
    return algaeIntake;
  }

  /**
   * Command to score algae.
   * @return A command that sets the algae state to SCORE until the algae is no longer detected and then disables.
   */
  public Command algaeScoreCommand() {
    Command algaeScore = new InstantCommand(() -> m_state = AlgaeState.SCORE)
        .andThen(new WaitUntilCommand(() -> !hasAlgae()))
        .andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));
        
    algaeScore.addRequirements(this);
    return algaeScore;
  }

  /**
   * Command to disable the algae rollers.
   * @return A command that sets the algae state to OFF to disable the subsystem.
   */
  public Command algaeOffCommand() {
    Command algaeOff = new InstantCommand(() -> m_state = AlgaeState.OFF);

    algaeOff.addRequirements(this);
    return algaeOff;
  }

  /**
   * Method to determine if algae is present by monitoring motor amperage.
   * @return True if algae is present, false otherwise.
   */
  public boolean hasAlgae() {
    double averageAmperage = calculateAverageAmperage();
    return averageAmperage > AlgaeConstants.ALGAE_AMPERAGE_THRESHOLD;
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
    return sum / AlgaeConstants.ALGAE_AMPERAGE_SAMPLE_SIZE;
  }

  public enum AlgaeState {
    INTAKE,
    SCORE,
    OFF
  }
}
