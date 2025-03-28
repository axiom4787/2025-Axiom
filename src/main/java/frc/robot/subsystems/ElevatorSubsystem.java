package frc.robot.subsystems;

// Motor Controllers
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;

// This subsystem controls the elevator that raises and lowers the coral manipulator arm.
public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorState m_state = ElevatorState.L1;

    private final SparkMax m_elevatorMotorR = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_R, MotorType.kBrushless);
    private final SparkMax m_elevatorMotorL = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_L, MotorType.kBrushless);

    private final PIDController m_elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_KP,
            ElevatorConstants.ELEVATOR_KI, ElevatorConstants.ELEVATOR_KD);

    public ElevatorSubsystem() {
        m_elevatorPID.setTolerance(0.05);

        SparkMaxConfig elevatorMotorL_Config = new SparkMaxConfig();
        elevatorMotorL_Config.inverted(true);
        elevatorMotorL_Config.idleMode(IdleMode.kCoast);
        elevatorMotorL_Config.smartCurrentLimit(40);

        // do NOT trust the right motors encoder it needs to go to mental hospital
        SparkMaxConfig elevatorMotorR_Config = new SparkMaxConfig();
        elevatorMotorR_Config.idleMode(IdleMode.kCoast);
        elevatorMotorR_Config.smartCurrentLimit(40);
        elevatorMotorR_Config.follow(m_elevatorMotorL, true);

        m_elevatorMotorR.configure(elevatorMotorR_Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_elevatorMotorL.configure(elevatorMotorL_Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // robot should always start with elevator at lowest position
        m_elevatorMotorR.getEncoder().setPosition(0);
        m_elevatorMotorL.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Elevator State", m_state.name());
        SmartDashboard.putNumber("Left Elevator Encoder", m_elevatorMotorL.getEncoder().getPosition());

        // dont bother looking at this tbh it wont be useful
        SmartDashboard.putNumber("Right Elevator Encoder", m_elevatorMotorR.getEncoder().getPosition());

        switch (m_state) {
            case L1     -> m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition(), ElevatorConstants.ELEVATOR_L1_POSITION);
            case L2     -> m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition(), ElevatorConstants.ELEVATOR_L2_POSITION);
            case L3     -> m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition(), ElevatorConstants.ELEVATOR_L3_POSITION);
            case SOURCE -> m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition(), ElevatorConstants.ELEVATOR_SOURCE_POSITION);
            default     -> m_elevatorMotorL.set(0);
        }

        m_elevatorMotorL.set(m_elevatorPID.calculate(m_elevatorMotorL.getEncoder().getPosition()));
    }

    /**
     * Command to raise the elevator to the first level.
     * 
     * @return A command that raises the elevator until it reaches the first level
     *         setpoint.
     */
    public Command elevatorL1Command() {
        Command elevatorL1 = new InstantCommand(() -> m_state = ElevatorState.L1)
                .andThen(new WaitUntilCommand(() -> m_elevatorPID.atSetpoint()))
                .andThen(new PrintCommand("L1 Finished"));
                
        elevatorL1.addRequirements(this);
        return elevatorL1;
    }

    /**
     * Command to raise the elevator to the second level.
     * 
     * @return A command that raises the elevator until it reaches the second level
     *         setpoint.
     */
    public Command elevatorL2Command() {
        Command elevatorL2 = new InstantCommand(() -> m_state = ElevatorState.L2)
                .andThen(new WaitUntilCommand(() -> m_elevatorPID.atSetpoint()))
                .andThen(new PrintCommand("L2 Finished"));

        elevatorL2.addRequirements(this);
        return elevatorL2;
    }

    /**
     * Command to raise the elevator to the third level.
     * 
     * @return A command that raises the elevator until it reaches the third level
     *         setpoint.
     */
    public Command elevatorL3Command() {
        Command elevatorL3 = new InstantCommand(() -> m_state = ElevatorState.L3)
                .andThen(new WaitUntilCommand(() -> m_elevatorPID.atSetpoint()))
                .andThen(new PrintCommand("L3 Finished"));

        elevatorL3.addRequirements(this);
        return elevatorL3;
    }

    /**
     * Command to raise the elevator to the source position.
     * 
     * @return A command that raises the elevator until it reaches the source
     *         setpoint.
     */
    public Command elevatorSourceCommand() {
        Command elevatorSource = new InstantCommand(() -> m_state = ElevatorState.SOURCE)
                .andThen(new WaitUntilCommand(() -> m_elevatorPID.atSetpoint()))
                .andThen(new PrintCommand("Source Finished"));
        elevatorSource.addRequirements(this);
        return elevatorSource;
    }

    public enum ElevatorState {
        L1,
        L2,
        L3,
        SOURCE
    }
}
