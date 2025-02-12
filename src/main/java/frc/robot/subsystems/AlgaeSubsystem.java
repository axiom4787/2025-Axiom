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

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.PID;

public class AlgaeSubsystem extends SubsystemBase {
        private AlgaeState m_state = hasAlgae() ? AlgaeState.FULL : AlgaeState.EMPTY;

        // Motor Controllers for Algae
        public static final SparkMax m_rightArmMotor = new SparkMax(AlgaeConstants.RIGHT_ALGAE_ARM_MOTOR_ID,
                        MotorType.kBrushless);
        public static final SparkMax m_leftArmMotor = new SparkMax(AlgaeConstants.LEFT_ALGAE_ARM_MOTOR_ID,
                        MotorType.kBrushless);
        public static final SparkMax m_leftAlgaeWheel = new SparkMax(AlgaeConstants.LEFT_ALGAE_WHEEL_MOTOR_ID,
                        MotorType.kBrushless);
        public static final SparkMax m_rightAlgaeWheel = new SparkMax(AlgaeConstants.RIGHT_ALGAE_WHEEL_MOTOR_ID,
                        MotorType.kBrushless);

        public AlgaeSubsystem() {
                SparkMaxConfig algaeWheelConfig = new SparkMaxConfig();

                algaeWheelConfig
                                .inverted(true)
                                .idleMode(IdleMode.kBrake);
                algaeWheelConfig.encoder
                                .positionConversionFactor(PID.ALGAE_pCONV)
                                .velocityConversionFactor(PID.ALGAE_vCONV);
                algaeWheelConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(PID.ALGAE_KP, PID.ALGAE_KI, PID.ALGAE_KD)
                                .velocityFF(PID.ALGAE_KFF, ClosedLoopSlot.kSlot1)
                                .outputRange(PID.ALGAE_KMIN_OUTPUT, PID.ALGAE_KMAX_OUTPUT);

                m_rightAlgaeWheel.configure(algaeWheelConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                m_leftAlgaeWheel.configure(algaeWheelConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                SparkMaxConfig armConfig = new SparkMaxConfig();

                armConfig
                                .inverted(false)
                                .idleMode(IdleMode.kBrake);
                armConfig.encoder
                                .positionConversionFactor(PID.ARM_ALGAE_pCONV)
                                .velocityConversionFactor(PID.ARM_ALGAE_vCONV);
                armConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(PID.ARM_ALGAE_KP, PID.ARM_ALGAE_KI, PID.ARM_ALGAE_KD)
                                .velocityFF(PID.ARM_ALGAE_KFF, ClosedLoopSlot.kSlot1)
                                .outputRange(PID.ARM_ALGAE_KMIN_OUTPUT, PID.ARM_ALGAE_KMAX_OUTPUT);

                m_rightArmMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                m_leftArmMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        }

        @Override
        public void periodic() {
                switch (m_state) {
                        case INTAKE:
                                // TODO: extend arm
                                m_rightAlgaeWheel.set(AlgaeConstants.ALGAE_WHEEL_INTAKE_DUTYCYCLE);
                                m_leftAlgaeWheel.set(AlgaeConstants.ALGAE_WHEEL_INTAKE_DUTYCYCLE);
                                if (hasAlgae())
                                        m_state = AlgaeState.FULL;
                                break;
                        case OUTTAKE:
                                // TODO: extend arm and only activate wheels if arm is extended
                                m_rightAlgaeWheel.set(AlgaeConstants.ALGAE_WHEEL_OUTTAKE_DUTYCYCLE);
                                m_leftAlgaeWheel.set(AlgaeConstants.ALGAE_WHEEL_OUTTAKE_DUTYCYCLE);
                                if (!hasAlgae()) {
                                        m_state = AlgaeState.EMPTY;
                                }
                                break;
                        case EMPTY:
                                // TODO: make sure arm stays in retracted position
                                m_rightAlgaeWheel.set(0.0);
                                m_leftAlgaeWheel.set(0.0);
                                break;
                        case FULL:
                                // TODO: make sure arm stays in extended position
                                m_rightAlgaeWheel.set(0.0);
                                m_leftAlgaeWheel.set(0.0);
                                break;
                }
        }

        /** Puts the manipulator in intake mode, if an algae is not present. */
        public void intake() {
                if (!hasAlgae()) {
                        m_state = AlgaeState.INTAKE;
                }
        }

        /** Puts the manipulator in outtake mode. */
        public void outtake() {
                if (hasAlgae()) {
                        m_state = AlgaeState.OUTTAKE;
                }
        }

        private boolean hasAlgae() {
                // TODO: use time of flight to determine if algae is present
                return false;
        }

        public enum AlgaeState {
                INTAKE,
                OUTTAKE,
                EMPTY,
                FULL
        }
}
