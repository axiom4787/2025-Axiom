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

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PID;

public class AlgaeSubsystem extends SubsystemBase {
  private AlgaeState m_state = AlgaeState.EMPTY; // hasAlgae() ? AlgaeState.FULL : AlgaeState.EMPTY;

  private final SparkMax m_AlgaeWheel = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_R, MotorType.kBrushless);
  private final SparkMax m_ArmMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_L, MotorType.kBrushless);
  private final PIDController m_AlgaePID = new PIDController(PID.ARM_ALGAE_KP, PID.ARM_ALGAE_KI, PID.ARM_ALGAE_KD);

  private SparkMaxConfig algaeWheelConfig = new SparkMaxConfig();
  private SparkMaxConfig armConfig = new SparkMaxConfig();

  public AlgaeSubsystem() {
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
  }

  @Override
  public void periodic() {

    SmartDashboard.putString("Algae State", m_state.name());

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

  /* Puts the manipulator in intake mode, if an algae is not present. */
  public void intake() {
    // if (!hasAlgae()) {
    m_state = AlgaeState.INTAKE;
    // }
  }

  public void armUp() {
    m_state = AlgaeState.UP;
  }

  /* Puts the manipulator in outtake mode. */
  public void outtake() {
    // if (hasAlgae()) {
    m_state = AlgaeState.OUTTAKE;
    // }
  }

  /*
   * /
   * private boolean hasAlgae() {
   * // TODO: use time of flight to determine if algae is present
   * return false;
   * }
   * /
   */

  public enum AlgaeState {
    INTAKE,
    UP,
    OUTTAKE,
    EMPTY,
    FULL
  }
}
