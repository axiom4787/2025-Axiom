package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;
import javax.accessibility.AccessibleRelation;

// Motor Controllers
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; 
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Machine;
import frc.robot.Constants;

import frc.robot.Constants.cIntake;
import frc.robot.Constants.cIntake.m_limits;
import frc.robot.Constants.cIntake.m_Intake_Motors;
import frc.robot.Constants.cIntake.PIDConstants;
import frc.robot.Constants.cIntake.AngleReferences;

public class MIntake extends SubsystemBase{

        //Replace with the current state global via import later
        public Machine mState = Constants.state;

        //set our default speed
        public float m_default_speed = cIntake.m_default_speed;

//        public final SparkClosedLoopController m_left_coral_pid, m_right_coral_pid;
//        public final SparkClosedLoopController m_rot_coral_pid;
//    
//        public final SparkClosedLoopController m_arm_algae_1_pid;
//        public final SparkClosedLoopController m_arm_algae_2_pid;
//    
//        public final SparkClosedLoopController m_left_algae_pid;
//        public final SparkClosedLoopController m_right_algae_pid;
//    
        public static final SparkMax m_left_coral = new SparkMax(m_Intake_Motors.motor_coral_1, MotorType.kBrushless), m_right_coral = new SparkMax(m_Intake_Motors.motor_coral_2, MotorType.kBrushless);
        public static final SparkMax m_rot_coral = new SparkMax(m_Intake_Motors.motor_rot_coral, MotorType.kBrushless);
        public static final SparkMax m_arm_algae_1 = new SparkMax(m_Intake_Motors.motor_arm_algae_1, MotorType.kBrushless), m_arm_algae_2 = new SparkMax(m_Intake_Motors.motor_arm_algae_2, MotorType.kBrushless);
        public static final SparkMax m_left_algae = new SparkMax(m_Intake_Motors.motor_left_algae, MotorType.kBrushless), m_right_algae = new SparkMax(m_Intake_Motors.motor_right_algae, MotorType.kBrushless);


    
        public enum algae_arm {
            Down,
            Up
        }
        public algae_arm aState = algae_arm.Up;
    
        public enum coral_arm {
            Processor,
            L1
        }
        public coral_arm cState = coral_arm.Processor;
    
        public MIntake() {
            SparkMaxConfig leftCoralConfig = new SparkMaxConfig();
    
            leftCoralConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            leftCoralConfig.encoder
                .positionConversionFactor(PIDConstants.LEFT_CORAL_pCONV)
                .velocityConversionFactor(PIDConstants.LEFT_CORAL_vCONV);
            leftCoralConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(PIDConstants.LEFT_CORAL_KP, PIDConstants.LEFT_CORAL_KI, PIDConstants.LEFT_CORAL_KD)
                .velocityFF(PIDConstants.LEFT_CORAL_KFF, ClosedLoopSlot.kSlot1)
                .outputRange(PIDConstants.LEFT_CORAL_KMIN_OUTPUT, PIDConstants.LEFT_CORAL_KMAX_OUTPUT);
    
    
            m_left_coral.configure(leftCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
            SparkMaxConfig rightCoralConfig = new SparkMaxConfig();
    
            rightCoralConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);
            rightCoralConfig.encoder
                .positionConversionFactor(PIDConstants.RIGHT_CORAL_pCONV)
                .velocityConversionFactor(PIDConstants.RIGHT_CORAL_vCONV);
            rightCoralConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(PIDConstants.RIGHT_CORAL_KP, PIDConstants.RIGHT_CORAL_KI, PIDConstants.RIGHT_CORAL_KD)
                .velocityFF(PIDConstants.RIGHT_CORAL_KFF, ClosedLoopSlot.kSlot1)
                .outputRange(PIDConstants.RIGHT_CORAL_KMIN_OUTPUT, PIDConstants.RIGHT_CORAL_KMAX_OUTPUT);
    
            m_right_coral.configure(rightCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            SparkMaxConfig rightAlgaeConfig = new SparkMaxConfig();
    
            rightAlgaeConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);
            rightAlgaeConfig.encoder
                .positionConversionFactor(PIDConstants.ALGAE_pCONV)
                .velocityConversionFactor(PIDConstants.ALGAE_vCONV);
            rightAlgaeConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(PIDConstants.ALGAE_KP, PIDConstants.ALGAE_KI, PIDConstants.ALGAE_KD)
                .velocityFF(PIDConstants.ALGAE_KFF, ClosedLoopSlot.kSlot1)
                .outputRange(PIDConstants.ALGAE_KMIN_OUTPUT, PIDConstants.ALGAE_KMAX_OUTPUT);
    
            m_right_algae.configure(rightAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            SparkMaxConfig rotCoralConfig = new SparkMaxConfig();
    
            rotCoralConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            rotCoralConfig.encoder
                .positionConversionFactor(PIDConstants.ROT_CORAL_pCONV)
                .velocityConversionFactor(PIDConstants.ROT_CORAL_vCONV);
            rotCoralConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(PIDConstants.ROT_CORAL_KP, PIDConstants.ROT_CORAL_KI, PIDConstants.ROT_CORAL_KD)
                .velocityFF(PIDConstants.ROT_CORAL_KFF, ClosedLoopSlot.kSlot1)
                .outputRange(PIDConstants.ROT_CORAL_KMIN_OUTPUT, PIDConstants.ROT_CORAL_KMAX_OUTPUT);
    
            m_rot_coral.configure(rotCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
            SparkMaxConfig leftAlgaeConfig = new SparkMaxConfig();
    
            leftAlgaeConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            leftAlgaeConfig.encoder
                .positionConversionFactor(PIDConstants.ALGAE_pCONV)
                .velocityConversionFactor(PIDConstants.ALGAE_vCONV);
            leftAlgaeConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(PIDConstants.ALGAE_KP, PIDConstants.ALGAE_KI, PIDConstants.ALGAE_KD)
                .velocityFF(PIDConstants.ALGAE_KFF, ClosedLoopSlot.kSlot1)
                .outputRange(PIDConstants.ALGAE_KMIN_OUTPUT, PIDConstants.ALGAE_KMAX_OUTPUT);
    
            m_left_algae.configure(leftAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            SparkMaxConfig armAlgae1Config = new SparkMaxConfig();
    
            armAlgae1Config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            armAlgae1Config.encoder
                .positionConversionFactor(PIDConstants.ARM_ALGAE_pCONV)
                .velocityConversionFactor(PIDConstants.ARM_ALGAE_vCONV);
            armAlgae1Config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(PIDConstants.ARM_ALGAE_KP, PIDConstants.ARM_ALGAE_KI, PIDConstants.ARM_ALGAE_KD)
                .velocityFF(PIDConstants.ARM_ALGAE_KFF, ClosedLoopSlot.kSlot1)
                .outputRange(PIDConstants.ARM_ALGAE_KMIN_OUTPUT, PIDConstants.ARM_ALGAE_KMAX_OUTPUT);
    
            m_arm_algae_1.configure(armAlgae1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
            SparkMaxConfig armAlgae2Config = new SparkMaxConfig();
    
            armAlgae2Config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            armAlgae2Config.encoder
                .positionConversionFactor(PIDConstants.ARM_ALGAE_pCONV)
                .velocityConversionFactor(PIDConstants.ARM_ALGAE_vCONV);
            armAlgae2Config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(PIDConstants.ARM_ALGAE_2_KP, PIDConstants.ARM_ALGAE_KI, PIDConstants.ARM_ALGAE_KD)
                .velocityFF(PIDConstants.ARM_ALGAE_KFF, ClosedLoopSlot.kSlot1)
                .outputRange(PIDConstants.ARM_ALGAE_KMIN_OUTPUT, PIDConstants.ARM_ALGAE_KMAX_OUTPUT);
    
            m_arm_algae_2.configure(armAlgae2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        }

        public static class pid_vars{
            public static final SparkClosedLoopController m_left_coral_pid = m_left_coral.getClosedLoopController();
            public static final SparkClosedLoopController m_right_coral_pid = m_right_coral.getClosedLoopController();
            public static final SparkClosedLoopController m_rot_coral_pid = m_rot_coral.getClosedLoopController();
    
            public static final SparkClosedLoopController m_left_algae_pid = m_left_algae.getClosedLoopController();
            public static final SparkClosedLoopController m_right_algae_pid = m_right_algae.getClosedLoopController();
    
            public static final SparkClosedLoopController m_arm_algae_1_pid = m_arm_algae_1.getClosedLoopController();
            public static final SparkClosedLoopController m_arm_algae_2_pid = m_arm_algae_2.getClosedLoopController();
        }
    
        private class Actions {
            public static void Intake_Algae(SparkMax m_left_algae, SparkMax m_right_algae, float m_default_speed){
                m_left_algae.set(-m_default_speed);
                m_right_algae.set(-m_default_speed);
            }
            public static void Outtake_Algae(SparkMax m_left_algae, SparkMax m_right_algae, float m_default_speed){
                m_left_algae.set(m_default_speed);
                m_right_algae.set(m_default_speed);
            }
            public static void a_Angle_Arm(algae_arm aState, SparkMax m_arm_algae_1, SparkMax m_arm_algae_2) {
                switch(aState){
                    case Down:
                        // use pid to move arm down
                        pid_vars.m_arm_algae_1_pid.setReference(AngleReferences.ALGAE_ANGLE_DOWN, ControlType.kPosition);
                        pid_vars.m_arm_algae_2_pid.setReference(AngleReferences.ALGAE_ANGLE_DOWN, ControlType.kPosition);
                    case Up:
                        // use pid to move arm up
                        pid_vars.m_arm_algae_1_pid.setReference(AngleReferences.ALGAE_ANGLE_UP, ControlType.kPosition);
                        pid_vars.m_arm_algae_2_pid.setReference(AngleReferences.ALGAE_ANGLE_UP, ControlType.kPosition);
            }
        }
        public static void c_Angle_Arm(coral_arm cState, SparkMax m_rot_coral) {
            switch(cState){
                case Processor:
                    // use pid to move arm down
                    pid_vars.m_rot_coral_pid.setReference(AngleReferences.CORAL_ANGLE_PROCESSOR, ControlType.kPosition);
            case L1:
                // use pid to move arm up
                pid_vars.m_rot_coral_pid.setReference(AngleReferences.CORAL_ANGLE_L1, ControlType.kPosition);
        }
        }
        public static void Intake_Coral(SparkMax m_left_coral, SparkMax m_right_coral, float m_default_speed){
            m_left_coral.set(-m_default_speed);
            m_right_coral.set(-m_default_speed);
        }
        public static void Outtake_Coral(SparkMax m_left_coral, SparkMax m_right_coral, float m_default_speed){
                    m_left_coral.set(m_default_speed);
                    m_right_coral.set(m_default_speed);
                }
            }
    
        public void periodic(){
            m_left_algae.set(0.0f);
            m_right_algae.set(0.0f);
            m_left_coral.set(0.0f);
            m_right_coral.set(0.0f);
    
            m_arm_algae_1.set(0.0f);
            m_arm_algae_2.set(0.0f);
    
            m_rot_coral.set(0.0f);
    
            switch(mState){
                // Review reqs, update constants, etc
    
                case CoralOuttake:
                    Actions.Outtake_Coral(m_left_coral, m_right_coral, m_default_speed);
                case L1:
                    cState =  coral_arm.L1;
                    Actions.c_Angle_Arm(cState, m_rot_coral);
                    Actions.Outtake_Coral(m_left_coral, m_right_coral, m_default_speed);
        
        case CoralIntake:
            Actions.Intake_Coral(m_left_algae, m_right_algae, m_default_speed);
        case AlgaeIntake:
            Actions.Intake_Algae(m_left_algae, m_right_algae, m_default_speed);
        
        case AlgaeIntakeGround:
            Actions.Intake_Algae(m_left_algae, m_right_algae, m_default_speed);
            aState = algae_arm.Down;
            Actions.a_Angle_Arm(aState, m_arm_algae_1, m_arm_algae_2);
        
        case Processor:
            Actions.Outtake_Algae(m_left_algae, m_right_algae, m_default_speed);
        
        case Robot_off:
            m_left_algae.disable();
            m_right_algae.disable();
            m_left_coral.disable();
            m_right_coral.disable();
            m_arm_algae_1.disable();
            m_arm_algae_2.disable();
            m_rot_coral.disable();
        }
    }
}