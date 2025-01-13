package frc.robot;

import org.ejml.equation.Variable;

public class Constants {
    public enum Machine{
        CoralOuttake,
        CoralIntake,
        AlgaeIntake,
        Processor,
        Ground,
        Climbing_on,
        Climbing_off,
        Robot_off
    }
    
    public static class m_Intake_Motors{
        public static final int motor_algae_1 = 0; // Check
        public static final int motor_algae_2 = 0; // Check
        public static final int motor_coral_1 = 0; // Check  
        public static final int motor_coral_2 = 0; // Check  

        public static final int motor_algae_angle = 0; // Check
    }
}