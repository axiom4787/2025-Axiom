package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;
import javax.accessibility.AccessibleRelation;

import com.revrobotics.PIDController;

import com.revrobotics.SparkMax;
import com.revrobotics.SparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxConfig;
import com.revrobotics.FeedbackSensor;
import com.revrobotics.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

// Import set saftey enabled

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Machine;
import frc.robot.Constants;

import frc.robot.Constants.cIntake;
import frc.robot.Constants.cIntake.m_limits;
import frc.robot.Constants.cIntake.m_Intake_Motors;
import frc.robot.Constants.cIntake.PIDConstants;
import frc.robot.Constants.cIntake.AngleReferences;

public class MIntake extends SubsystemBase{

    // Replace with the current state global via import later
    public Machine mState = Constants.state;

    // Set our default speed
    public float m_default_speed = cIntake.m_default_speed;

    private final PIDController m_left_coral_pid, m_right_coral_pid;
    private final PIDController m_rot_coral_pid;
    private final PIDController m_arm_algae_1_pid, m_arm_algae_2_pid, m_left_algae_pid, m_right_algae_pid;

    private final SparkMax m_left_coral, m_right_coral;
    private final SparkMax m_rot_coral;
    private final SparkMax m_arm_algae_1, m_arm_algae_2, m_left_algae, m_right_algae;


    private enum algae_arm {
        Down,
        Up
    }
    private algae_arm aState = algae_arm.Up;

    private enum coral_arm {
        Processor,
        L1
    }
    private coral_arm cState = coral_arm.Processor;

    public MIntake() {
        m_left_coral = new CANSparkMax(m_Intake_Motors.motor_coral_1, MotorType.kBrushless);
        SparkMaxConfig leftCoralConfig = new SparkMaxConfig();

        leftCoralConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        leftCoralConfig.encoder
            .positionConversionFactor(PIDConstants.LEFT_CORAL_pCONV)
            .velocityConversionFactor(PIDConstants.LEFT_CORAL_vCONV);
        leftCoralConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PIDConstants.LEFT_CORAL_KP, PIDConstants.LEFT_CORAL_KI, PIDConstants.LEFT_CORAL_KD);

        m_left_coral.configure(leftCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistPara);


        m_right_coral = new CANSparkMax(m_Intake_Motors.motor_coral_2, MotorType.kBrushless);
        SparkMaxConfig rightCoralConfig = new SparkMaxConfig();

        rightCoralConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        rightCoralConfig.encoder
            .positionConversionFactor(PIDConstants.RIGHT_CORAL_pCONV)
            .velocityConversionFactor(PIDConstants.RIGHT_CORAL_vCONV);
        rightCoralConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PIDConstants.RIGHT_CORAL_KP, PIDConstants.RIGHT_CORAL_KI, PIDConstants.RIGHT_CORAL_KD);

        m_right_coral.configure(rightCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistPara);
        

        m_right_algae = new CANSparkMax(m_Intake_Motors.motor_right_algae, MotorType.kBrushless);
        SparkMaxConfig rightAlgaeConfig = new SparkMaxConfig();

        rightAlgaeConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        rightAlgaeConfig.encoder
            .positionConversionFactor(PIDConstants.ALGAE_pCONV)
            .velocityConversionFactor(PIDConstants.ALGAE_vCONV);
        rightAlgaeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PIDConstants.ALGAE_KP, PIDConstants.ALGAE_KI, PIDConstants.ALGAE_KD);

        m_right_algae.configure(rightAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistPara);
        
        
        m_rot_coral = new CANSparkMax(m_Intake_Motors.motor_rot_coral, MotorType.kBrushless);
        SparkMaxConfig rotCoralConfig = new SparkMaxConfig();

        rotCoralConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        rotCoralConfig.encoder
            .positionConversionFactor(PIDConstants.ROT_CORAL_pCONV)
            .velocityConversionFactor(PIDConstants.ROT_CORAL_vCONV);
        rotCoralConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PIDConstants.ROT_CORAL_KP, PIDConstants.ROT_CORAL_KI, PIDConstants.ROT_CORAL_KD);

        m_rot_coral.configure(rotCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistPara);


        m_left_algae = new CANSparkMax(m_Intake_Motors.motor_left_algae, MotorType.kBrushless);
        SparkMaxConfig leftAlgaeConfig = new SparkMaxConfig();

        leftAlgaeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        leftAlgaeConfig.encoder
            .positionConversionFactor(PIDConstants.ALGAE_pCONV)
            .velocityConversionFactor(PIDConstants.ALGAE_vCONV);
        leftAlgaeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PIDConstants.ALGAE_KP, PIDConstants.ALGAE_KI, PIDConstants.ALGAE_KD);


        m_arm_algae_1 = new CANSparkMax(m_Intake_Motors.motor_arm_algae_1, MotorType.kBrushless);
        SparkMaxConfig armAlgae1Config = new SparkMaxConfig();

        armAlgae1Config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        armAlgae1Config.encoder
            .positionConversionFactor(PIDConstants.ARM_ALGAE_pCONV)
            .velocityConversionFactor(PIDConstants.ARM_ALGAE_vCONV);
        armAlgae1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PIDConstants.ARM_ALGAE_1_KP, PIDConstants.ARM_ALGAE_KI, PIDConstants.ARM_ALGAE_KD);

        m_arm_algae_1.configure(armAlgae1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistPara);


        m_arm_algae_2 = new CANSparkMax(m_Intake_Motors.motor_arm_algae_2, MotorType.kBrushless);
        SparkMaxConfig armAlgae2Config = new SparkMaxConfig();

        armAlgae2Config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        armAlgae2Config.encoder
            .positionConversionFactor(PIDConstants.ARM_ALGAE_pCONV)
            .velocityConversionFactor(PIDConstants.ARM_ALGAE_vCONV);
        armAlgae2Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PIDConstants.ARM_ALGAE_2_KP, PIDConstants.ARM_ALGAE_KI, PIDConstants.ARM_ALGAE_KD);

        m_arm_algae_2.configure(armAlgae2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistPara);
        
        
        // Configure PID coefficients
        configurePIDControllers();
    }

    private void configurePIDControllers() {

        // Configure PID controllers for left coral
        m_left_coral_pid.setFF(PIDConstants.LEFT_CORAL_KFF);
        m_left_coral_pid.setOutputRange(PIDConstants.LEFT_CORAL_KMIN_OUTPUT, PIDConstants.LEFT_CORAL_KMAX_OUTPUT);

        // Configure PID controllers for right coral
        m_right_coral_pid.setFF(PIDConstants.RIGHT_CORAL_KFF);
        m_right_coral_pid.setOutputRange(PIDConstants.RIGHT_CORAL_KMIN_OUTPUT, PIDConstants.RIGHT_CORAL_KMAX_OUTPUT);

        // Configure PID controllers for left algae
        m_left_algae_pid.setFF(PIDConstants.ALGAE_KFF);
        m_left_algae_pid.setOutputRange(PIDConstants.ALGAE_KMIN_OUTPUT, PIDConstants.ALGAE_KMAX_OUTPUT);

        // Configure PID controllers for right algae
        m_right_algae_pid.setFF(PIDConstants.ALGAE_KFF);
        m_right_algae_pid.setOutputRange(PIDConstants.ALGAE_KMIN_OUTPUT, PIDConstants.ALGAE_KMAX_OUTPUT);

        // Configure PID controllers for rot coral
        m_rot_coral_pid.setFF(PIDConstants.ROT_CORAL_KFF);
        m_rot_coral_pid.setOutputRange(PIDConstants.ROT_CORAL_KMIN_OUTPUT, PIDConstants.ROT_CORAL_KMAX_OUTPUT);

        // Configure PID controllers for arm algae 1
        m_arm_algae_1_pid.setFF(PIDConstants.ARM_ALGAE_KFF);
        m_arm_algae_1_pid.setOutputRange(PIDConstants.ARM_ALGAE_KMIN_OUTPUT, PIDConstants.ARM_ALGAE_KMAX_OUTPUT);

        // Configure PID controllers for arm algae 2
        m_arm_algae_2_pid.setFF(PIDConstants.ARM_ALGAE_KFF);
        m_arm_algae_2_pid.setOutputRange(PIDConstants.ARM_ALGAE_KMIN_OUTPUT, PIDConstants.ARM_ALGAE_KMAX_OUTPUT);
    }

    private class Actions {
        public void Intake_Algae(SparkMax m_left_algae, SparkMax m_right_algae, float m_default_speed){
            m_left_algae.set(-m_default_speed);
            m_right_algae.set(-m_default_speed);
        }
        public void Outtake_Algae(SparkMax m_left_algae, SparkMax m_right_algae, float m_default_speed){
            m_left_algae.set(m_default_speed);
            m_right_algae.set(m_default_speed);
        }
        public void a_Angle_Arm(algae_arm aState, SparkMax m_arm_algae_1, SparkMax m_arm_algae_2) {
            switch(aState){
                case Down:
                    // use pid to move arm down
                    m_arm_algae_1.setReference(AngleReferences.ALGAE_ANGLE_DOWN, ControlType.kPosition);
                    m_arm_algae_2.setReference(AngleReferences.ALGAE_ANGLE_DOWN, ControlType.kPosition);
                case Up:
                    // use pid to move arm up
                    m_arm_algae_1.setReference(AngleReferences.ALGAE_ANGLE_UP, ControlType.kPosition);
                    m_arm_algae_2.setReference(AngleReferences.ALGAE_ANGLE_UP, ControlType.kPosition);
            }
        }
        public void c_Angle_Arm(coral_arm cState, SparkMax m_rot_coral) {
            switch(cState){
                case Processor:
                    // use pid to move arm down
                    m_rot_coral.setReference(AngleReferences.CORAL_ANGLE_PROCESSOR, ControlType.kPosition);
                case L1:
                    // use pid to move arm up
                    m_rot_coral.setReference(AngleReferences.CORAL_ANGLE_L1, ControlType.kPosition);
            }
        }
        public void Intake_Coral(SparkMax m_left_coral, SparkMax m_right_coral, float m_default_speed){
            m_left_coral.set(-m_default_speed);
            m_right_coral.set(-m_default_speed);
        }
        public void Outtake_Coral(SparkMax m_left_coral, SparkMax m_right_coral, float m_default_speed){
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