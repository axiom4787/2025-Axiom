package frc.robot;

import org.ejml.equation.Variable;

public class Constants {

    public static final float m_default_speed = 1.0f;

    public enum Machine{
        CoralOuttake,
        CoralIntake,
        AlgaeIntake,
        AlgaeIntakeGround,
        Processor,
        L1,
        Climbing_on,
        Climbing_off,
        Robot_off
    }
    
    public static class m_Intake_Motors{
        public static final int motor_algae_arm = 0; // Check
        public static final int motor_coral_1 = 0; // Check  
        public static final int motor_coral_2 = 0; // Check  

        public static final int motor_arm_algae = 0; // Check

        public static final int motor_right_algae = 0; // Check
        public static final int motor_left_algae = 0; // Check
    }
}