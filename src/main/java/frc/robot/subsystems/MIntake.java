package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;
import javax.accessibility.AccessibleRelation;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Machine;
import frc.robot.Constants.Arm_Angles;
import frc.robot.Constants.m_Intake_Motors;

public class MIntake extends SubsystemBase{

    // Replace with the current state global via import later
    public Machine mState = Machine.Robot_off;

    private final PWMSparkMax m_left_algae, m_right_algae, m_left_coral, m_right_coral;
    private final PWMSparkMax m_arm_angle;
    private final float m_default_speed = 1.0f;

    public MIntake() {
        // instantiate the 4 motors
        m_left_algae = new PWMSparkMax(m_Intake_Motors.motor_algae_1);
        m_left_algae.setInverted(false);
        m_left_algae.setSafetyEnabled(true);

        m_right_algae = new PWMSparkMax(m_Intake_Motors.motor_algae_2);
        m_right_algae.setInverted(true);
        m_right_algae.setSafetyEnabled(true);

        m_left_coral = new PWMSparkMax(m_Intake_Motors.motor_coral_1);
        m_left_coral.setInverted(false);
        m_left_coral.setSafetyEnabled(true);

        m_right_coral = new PWMSparkMax(m_Intake_Motors.motor_coral_2);
        m_right_coral.setInverted(true);
        m_right_coral.setSafetyEnabled(true);

        // instantiate arm angle motors
        m_arm_angle = new PWMSparkMax(m_Intake_Motors.motor_arm_angle);
        m_arm_angle.setInverted(false);
        m_arm_angle.setSafetyEnabled(true);
    }
    
    public class Actions {
        public static void Intake_Algae(PWMSparkMax m_left_algae, PWMSparkMax m_right_algae, float m_default_speed){
            m_left_algae.set(-m_default_speed);
            m_right_algae.set(-m_default_speed);
        }
        public static void Outtake_Algae(PWMSparkMax m_left_algae, PWMSparkMax m_right_algae, float m_default_speed){
            m_left_algae.set(m_default_speed);
            m_right_algae.set(m_default_speed);
        }
        public static void Angle_Arm(float angle, PWMSparkMax m_arm_angle){
            // Do this!
        }
        public static void Intake_Coral(PWMSparkMax m_left_coral, PWMSparkMax m_right_coral, float m_default_speed){
            m_left_coral.set(-m_default_speed);
            m_right_coral.set(-m_default_speed);
        }
        public static void Outtake_Coral(PWMSparkMax m_left_coral, PWMSparkMax m_right_coral, float m_default_speed){
            m_left_coral.set(m_default_speed);
            m_right_coral.set(m_default_speed);
        }
    }
    public void periodic(){

        m_left_algae.disable();
        m_right_algae.disable();
        m_left_coral.disable();
        m_right_coral.disable();

        switch(mState){

            case L1:
                Actions.Angle_Arm(Arm_Angles.arm_angle_l1);
                Actions.Outtake_Coral(m_left_coral, m_right_coral, m_default_speed);
            case L2:
                Actions.Angle_Arm(Arm_Angles.arm_angle_l2);
                Actions.Outtake_Coral(m_left_coral, m_right_coral, m_default_speed);
                Actions.Intake_Algae(m_left_algae, m_right_algae, m_default_speed);
            case L3:
                Actions.Angle_Arm(Arm_Angles.arm_angle_l3);
                Actions.Outtake_Coral(m_left_coral, m_right_coral, m_default_speed);
                Actions.Intake_Algae(m_left_algae, m_right_algae, m_default_speed);
            case L4:
                Actions.Angle_Arm(Arm_Angles.arm_angle_l4, m_arm_angle);
                Actions.Intake_Algae(m_left_algae, m_right_algae, m_default_speed);
            
            case CoralIntake:
                Actions.Intake_Coral(m_left_algae, m_right_algae, m_default_speed);
            case AlgaeIntake:
                Actions.Intake_Algae(m_left_algae, m_right_algae, m_default_speed);
            
            case Processor:
                Actions.Outtake_Algae(m_left_algae, m_right_algae, m_default_speed);
            
            case Robot_off:
                m_left_algae.disable();
                m_right_algae.disable();
                m_left_coral.disable();
                m_right_coral.disable();
        }
    }
    
}
// TODO:
// Arm angle control
// Arm motors 

// DONE:
// state 1 -- L1
// state 2 -- L2
// state 3 -- L3
// state 4 -- L4
// state 5 -- Coral Intake
// state 6 -- Algae Intake
// state 7 -- processor