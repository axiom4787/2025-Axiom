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
<<<<<<< HEAD
=======
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PID;
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c

// This subsystem controls the roller on the algae manipulator.
public class AlgaeSubsystem extends SubsystemBase {
<<<<<<< HEAD
  private AlgaeState m_state = AlgaeState.OFF;
  // private SparkMax m_rollerMotor = new SparkMax(AlgaeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
=======
  private AlgaeState m_state = AlgaeState.EMPTY; // hasAlgae() ? AlgaeState.FULL : AlgaeState.EMPTY;

  private final SparkMax m_AlgaeWheel = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_R, MotorType.kBrushless);
  private final SparkMax m_ArmMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_L, MotorType.kBrushless);
  private final PIDController m_AlgaePID = new PIDController(PID.ARM_ALGAE_KP, PID.ARM_ALGAE_KI, PID.ARM_ALGAE_KD);

  private SparkMaxConfig algaeWheelConfig = new SparkMaxConfig();
  private SparkMaxConfig armConfig = new SparkMaxConfig();
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
<<<<<<< HEAD
    // m_rollerMotor.setCANTimeout(250);

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig.voltageCompensation(10);
    rollerMotorConfig.smartCurrentLimit(40);
    rollerMotorConfig.idleMode(IdleMode.kBrake);
    // m_rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
=======
    m_AlgaePID.setTolerance(0.05);

    algaeWheelConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);

    m_AlgaeWheel.configure(algaeWheelConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    armConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60);

    m_ArmMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_AlgaeWheel.getEncoder().setPosition(0);
    m_ArmMotor.getEncoder().setPosition(0);
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Algae State", m_state.name());

<<<<<<< HEAD
    // switch (m_state) {
    //   case INTAKE:
    //     m_rollerMotor.set(AlgaeConstants.ALGAE_INTAKE_DUTYCYCLE);
    //     break;
    //   case SCORE:
    //     m_rollerMotor.set(AlgaeConstants.ALGAE_SCORE_DUTYCYCLE);
    //     break;
    //   case OFF:
    //     m_rollerMotor.set(0);
    // }
  }

  /**
   * Command to intake algae.
   * @return A command that sets the algae state to INTAKE until the algae is no longer detected and then disables. (currently runs for 3 seconds since our sensor is not attached yet)
=======
    switch (m_state) {
      case INTAKE:
        // TODO: extend arm
        m_AlgaeWheel.set(AlgaeConstants.ALGAE_WHEEL_INTAKE_DUTYCYCLE);
        m_AlgaePID.calculate(m_ArmMotor.getEncoder().getPosition(), AlgaeConstants.ALGAE_ARM_OUT_ANGLE);
        // m_leftAlgaeWheel.set(AlgaeConstants.ALGAE_WHEEL_INTAKE_DUTYCYCLE);
        // if (hasAlgae())
        // m_state = AlgaeState.FULL;
        // break;
        System.out.println("AlgaeSubsystem: INTAKE");
        break;
      // in case we don't have a time of flight sensor set up to make sure that the algae is collected
      case UP:
        m_AlgaePID.calculate(m_ArmMotor.getEncoder().getPosition(), AlgaeConstants.ALGAE_ARM_IN_ANGLE);
      case OUTTAKE:
        // TODO: extend arm and only activate wheels if arm is extended
        m_AlgaeWheel.set(AlgaeConstants.ALGAE_WHEEL_OUTTAKE_DUTYCYCLE);
        m_AlgaePID.calculate(m_ArmMotor.getEncoder().getPosition(), AlgaeConstants.ALGAE_ARM_IN_ANGLE);
        // m_leftAlgaeWheel.set(AlgaeConstants.ALGAE_WHEEL_OUTTAKE_DUTYCYCLE);
        // if (!hasAlgae()) {
        // m_state = AlgaeState.EMPTY;
        // }
        System.out.println("AlgaeSubsystem: OUTTAKE");
        break;
      case EMPTY:
        // TODO: make sure arm stays in retracted position
        m_AlgaeWheel.set(0.0);
        m_AlgaePID.calculate(m_ArmMotor.getEncoder().getPosition(), AlgaeConstants.ALGAE_ARM_IN_ANGLE);
        break;
      case FULL:
        m_AlgaeWheel.set(0.0);
        m_AlgaePID.calculate(m_ArmMotor.getEncoder().getPosition(), AlgaeConstants.ALGAE_ARM_IN_ANGLE);
        break;
    }

    m_ArmMotor.set(m_AlgaePID.calculate(m_ArmMotor.getEncoder().getPosition()));
  }

  public void setAlgaeState(AlgaeState state) {
    m_state = state;
  }

  // /* Puts the manipulator in intake mode, if an algae is not present. */
  // public void intake() {
  //   // if (!hasAlgae()) {
  //   m_state = AlgaeState.INTAKE;
  //   // }
  // }

  // public void armUp() {
  //   m_state = AlgaeState.UP;
  // }

  // /* Puts the manipulator in outtake mode. */
  // public void outtake() {
  //   // if (hasAlgae()) {
  //   m_state = AlgaeState.OUTTAKE;
  //   // }
  // }

  /*
   * /
   * private boolean hasAlgae() {
   * // TODO: use time of flight to determine if algae is present
   * return false;
   * }
   * /
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c
   */
  public Command algaeIntakeCommand() {
    Command algaeIntake = new InstantCommand(() -> m_state = AlgaeState.INTAKE).andThen(new WaitCommand(3))
        .andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));
    
    // command for when sensor is attached:
    // Command algaeIntake = new InstantCommand(() -> m_state = AlgaeState.INTAKE)
    //     .andThen(new WaitUntilCommand(() -> hasAlgae())).andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));
    algaeIntake.addRequirements(this);
    return algaeIntake;
  }

  /**
   * Command to score algae.
   * @return A command that sets the algae state to SCORE until the algae is no longer detected and then disables. (currently runs for 3 seconds since our sensor is not attached yet)
   */
  public Command algaeScoreCommand() {
    Command algaeScore = new InstantCommand(() -> m_state = AlgaeState.SCORE).andThen(new WaitCommand(3))
        .andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));

    // command for when sensor is attached:
    // Command algaeScore = new InstantCommand(() -> m_state = AlgaeState.SCORE)
    //     .andThen(new WaitUntilCommand(() -> !hasAlgae())).andThen(new InstantCommand(() -> m_state = AlgaeState.OFF));
    algaeScore.addRequirements(this);
    return algaeScore;
  }

  /**
   * Method to determine if algae is present with Time of Flight sensor.
   * @return True if algae is present, false otherwise.
   */
  // public boolean hasAlgae() {
  // // TODO: use time of flight to determine if algae is present
  // return false;
  // }

  public enum AlgaeState {
    INTAKE,
<<<<<<< HEAD
    SCORE,
    OFF,
=======
    UP,
    OUTTAKE,
    EMPTY,
    FULL
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c
  }
}
