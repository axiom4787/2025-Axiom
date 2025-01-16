package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;
import javax.accessibility.AccessibleRelation;

import com.revrobotics.CANPIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

// Import set saftey enabled

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Machine;
import frc.robot.Constants.m_Intake_Motors;
import frc.robot.Constants;

public class MIntake extends SubsystemBase{

    // Replace with the current state global via import later
    public Machine mState = Machine.Robot_off;

    public float m_default_speed = Constants.m_default_speed;

    private final CANPIDController m_left_coral_pid, m_right_coral_pid;
    private final CANPIDController m_rot_coral_pid;
    private final CANPIDController m_arm_algae_1_pid, m_arm_algae_2_pid, m_left_algae_pid, m_right_algae_pid;

    private final CANSparkMax m_left_coral, m_right_coral;
    private final CANSparkMax m_rot_coral;
    private final CANSparkMax m_arm_algae_1, m_arm_algae_2, m_left_algae, m_right_algae;

    public MIntake() {
        m_left_coral = new CANSparkMax(m_Intake_Motors.motor_coral_1, MotorType.kBrushless);
        m_left_coral.setInverted(false);
        m_left_coral.setSafetyEnabled(true);
        m_left_coral_pid = m_left_coral.getPIDController();

        m_right_coral = new CANSparkMax(m_Intake_Motors.motor_coral_1, MotorType.kBrushless);
        m_right_coral.setInverted(true);
        m_right_coral.setSafetyEnabled(true);
        m_right_coral_pid = m_right_coral.getPIDController();

        m_left_algae = new CANSparkMax(m_Intake_Motors.motor_left_algae, MotorType.kBrushless);
        m_left_algae.setInverted(false);
        m_left_algae.setSafetyEnabled(true);
        m_left_algae_pid = m_left_algae.getPIDController();

        m_right_algae = new CANSparkMax(m_Intake_Motors.motor_right_algae, MotorType.kBrushless);
        m_right_algae.setInverted(true);
        m_right_algae.setSafetyEnabled(true);
        m_right_algae_pid = m_right_algae.getPIDController();

        // Setup rot coral motor
        m_rot_coral = new CANSparkMax(m_Intake_Motors.motor_coral_2, MotorType.kBrushless);
        m_rot_coral.setInverted(true);
        m_rot_coral.setSafetyEnabled(true);
        m_rot_coral_pid = m_rot_coral.getPIDController();

        // instantiate arm angle motors
        m_arm_algae_1 = new CANSparkMax(m_Intake_Motors.motor_arm_algae, MotorType.kBrushless);
        m_arm_algae_1.setInverted(false);
        m_arm_algae_1.setSafetyEnabled(true);
        m_arm_algae_1_pid = m_arm_algae_1.getPIDController();

        m_arm_algae_2 = new CANSparkMax(m_Intake_Motors.motor_arm_algae, MotorType.kBrushless);
        m_arm_algae_2.setInverted(false);
        m_arm_algae_2.setSafetyEnabled(true);
        m_arm_algae_2_pid = m_arm_algae_2.getPIDController();

        // Configure PID coefficients
        configurePIDControllers();
    }

    private void configurePIDControllers() {
        double kP = 0.1;
        double kI = 0.0;
        double kD = 0.0;
        double kFF = 0.0;
        double kMaxOutput = 1.0;
        double kMinOutput = -1.0;

        // Configure PID controllers
        m_left_coral_pid.setP(kP);
        m_left_coral_pid.setI(kI);
        m_left_coral_pid.setD(kD);
        m_left_coral_pid.setFF(kFF);
        m_left_coral_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_right_coral_pid.setP(kP);
        m_right_coral_pid.setI(kI);
        m_right_coral_pid.setD(kD);
        m_right_coral_pid.setFF(kFF);
        m_right_coral_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_left_algae_pid.setP(kP);
        m_left_algae_pid.setI(kI);
        m_left_algae_pid.setD(kD);
        m_left_algae_pid.setFF(kFF);
        m_left_algae_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_right_algae_pid.setP(kP);
        m_right_algae_pid.setI(kI);
        m_right_algae_pid.setD(kD);
        m_right_algae_pid.setFF(kFF);
        m_right_algae_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_rot_coral_pid.setP(kP);
        m_rot_coral_pid.setI(kI);
        m_rot_coral_pid.setD(kD);
        m_rot_coral_pid.setFF(kFF);
        m_rot_coral_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_arm_algae_1_pid.setP(kP);
        m_arm_algae_1_pid.setI(kI);
        m_arm_algae_1_pid.setD(kD);
        m_arm_algae_1_pid.setFF(kFF);
        m_arm_algae_1_pid.setOutputRange(kMinOutput, kMaxOutput);

        m_arm_algae_2_pid.setP(kP);
        m_arm_algae_2_pid.setI(kI);
        m_arm_algae_2_pid.setD(kD);
        m_arm_algae_2_pid.setFF(kFF);
        m_arm_algae_2_pid.setOutputRange(kMinOutput, kMaxOutput);
    }
    
    public class Actions {
        public static void Intake_Algae(CANSparkMax m_left_algae, CANSparkMax m_right_algae, float m_default_speed){
            m_left_algae.set(-m_default_speed);
            m_right_algae.set(-m_default_speed);
        }
        public static void Outtake_Algae(CANSparkMax m_left_algae, CANSparkMax m_right_algae, float m_default_speed){
            m_left_algae.set(m_default_speed);
            m_right_algae.set(m_default_speed);
        }
        public static void a_Angle_Arm(float angle, CANSparkMax m_arm_algae_1, CANSparkMax m_arm_algae_2) {
            // Do this!
        }
        public static void c_Angle_Arm(float angle, CANSparkMax m_rot_coral) {
            // Do this!
        }
        public static void Intake_Coral(CANSparkMax m_left_coral, CANSparkMax m_right_coral, float m_default_speed){
            m_left_coral.set(-m_default_speed);
            m_right_coral.set(-m_default_speed);
        }
        public static void Outtake_Coral(CANSparkMax m_left_coral, CANSparkMax m_right_coral, float m_default_speed){
            m_left_coral.set(m_default_speed);
            m_right_coral.set(m_default_speed);
        }
    }
    public void periodic(){
        // Fix all this, it's super broken
        m_left_algae.set(0.0f);
        m_right_algae.set(0.0f);
        m_left_coral.set(0.0f);
        m_right_coral.set(0.0f);

        m_arm_algae_1.set(0.0f);
        m_arm_algae_2.set(0.0f);

        switch(mState){
            // Review reqs, update constants, etc

            case CoralOuttake:
                Actions.Outtake_Coral(m_left_coral, m_right_coral, m_default_speed);
            case L1:
                Actions.c_Angle_Arm(0.0f, m_rot_coral);
                Actions.Outtake_Coral(m_left_coral, m_right_coral, m_default_speed);
            
            case CoralIntake:
                Actions.Intake_Coral(m_left_algae, m_right_algae, m_default_speed);
            case AlgaeIntake:
                Actions.Intake_Algae(m_left_algae, m_right_algae, m_default_speed);
            
            case AlgaeIntakeGround:
                Actions.Intake_Algae(m_left_algae, m_right_algae, m_default_speed);
                Actions.a_Angle_Arm(0.0f, m_arm_algae_1, m_arm_algae_2);
            
            case Processor:
                Actions.Outtake_Algae(m_left_algae, m_right_algae, m_default_speed);
            
            case Robot_off:
                m_left_algae.disable();
                m_right_algae.disable();
                m_left_coral.disable();
                m_right_coral.disable();
                m_arm_algae_1.disable();
                m_arm_algae_2.disable();
        }
    }
    
}