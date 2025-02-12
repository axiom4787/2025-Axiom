package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EverybotClimberConstants;
import frc.robot.Constants;

public class EverybotClimber extends SubsystemBase {

    private EverybotClimberState m_state = EverybotClimberState.OFF;

    private final SparkMax climbMotor;

    public EverybotClimber() {

        climbMotor = new SparkMax(EverybotClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

        climbMotor.setCANTimeout(250);

        SparkMaxConfig climbConfig = new SparkMaxConfig();
        climbConfig.voltageCompensation(EverybotClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
        climbConfig.smartCurrentLimit(EverybotClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
        climbConfig.idleMode(IdleMode.kBrake);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Climber State", m_state.name());
        switch (m_state)
        {
            case OFF:
                climbMotor.set(0);
                break;
            case CLIMB:
                climbMotor.set(EverybotClimberConstants.CLIMBER_CLIMB_SPEED);
                break;
            case UNCLIMB:
                climbMotor.set(EverybotClimberConstants.CLIMBER_UNCLIMB_SPEED);
                break;
        }
    }

    public void setDesiredState(EverybotClimberState desiredState) {
        m_state = desiredState;
    }

    public enum EverybotClimberState
    {
        OFF,
        CLIMB,
        UNCLIMB
    }
}
