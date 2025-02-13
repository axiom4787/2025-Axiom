package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private ClimberState m_state = ClimberState.OFF;

    private final SparkMax climbMotor;

    public ClimberSubsystem() {

        climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

        climbMotor.setCANTimeout(250);

        SparkMaxConfig climbConfig = new SparkMaxConfig();
        climbConfig.voltageCompensation(ClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
        climbConfig.smartCurrentLimit(ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
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
                climbMotor.set(ClimberConstants.CLIMBER_CLIMB_DUTYCYCLE);
                break;
            case UNCLIMB:
                climbMotor.set(ClimberConstants.CLIMBER_UNCLIMB_DUTYCYCLE);
                break;
        }
    }

    public void setState(ClimberState desiredState) {
        m_state = desiredState;
    }

    public enum ClimberState
    {
        OFF,
        CLIMB,
        UNCLIMB
    }
}
