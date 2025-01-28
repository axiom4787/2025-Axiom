package frc.robot;

import org.ejml.equation.Variable;

public class Constants {

    // Global machine state
    public static Machine state = Machine.Robot_off;

    // Default motor speed
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

    // For Intake Subsystem
    public static class m_Intake_Motors{
        public static final int motor_algae_arm = 0; // Check
        public static final int motor_coral_1 = 0; // Check  
        public static final int motor_coral_2 = 0; // Check  

        public static final int motor_arm_algae = 0; // Check

        public static final int motor_right_algae = 0; // Check
        public static final int motor_left_algae = 0; // Check
    }
    public static class m_limits{
        public static final int m_left_coral_limit = 40; // Check limit; should be in amps
        public static final int m_right_coral_limit = 40; // Check limit; should be in amps

        public static final int m_left_algae_limit = 40; // Check limit; should be in amps
        public static final int m_right_algae_limit = 40; // Check limit; should be in amps

        public static final int m_arm_algae_limit = 40; // Check limit; should be in amps
        public static final int m_rot_coral_limit = 40; // Check limit; should be in amps

        public static final int m_arm_algae_1_limit = 40; // Check limit; should be in amps
        public static final int m_arm_algae_2_limit = 40; // Check limit; should be in amps
    }
}
