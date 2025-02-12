package frc.robot;

import org.ejml.equation.Variable;

public class Constants {   
    public static class ClimberConstants
    {
        public static final int CLIMBER_MOTOR_ID = 1;
        public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
        public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
        public static final double CLIMBER_CLIMB_SPEED = -0.5;
        public static final double CLIMBER_UNCLIMB_SPEED = 0.5;
    } 
    public static class cIntake {
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
    
        // for motor controllers for intake
        public static final class PIDConstants {
            public static final double LEFT_CORAL_KP = 0.1;
            public static final double LEFT_CORAL_KI = 0.0;
            public static final double LEFT_CORAL_KD = 0.0;
            public static final double LEFT_CORAL_KFF = 0.0;
            public static final double LEFT_CORAL_KMAX_OUTPUT = 1.0;
            public static final double LEFT_CORAL_KMIN_OUTPUT = -1.0;
            public static final float LEFT_CORAL_pCONV = 1.0f;
            public static final float LEFT_CORAL_vCONV = 1.0f;
    
            public static final double RIGHT_CORAL_KP = 0.1;
            public static final double RIGHT_CORAL_KI = 0.0;
            public static final double RIGHT_CORAL_KD = 0.0;
            public static final double RIGHT_CORAL_KFF = 0.0;
            public static final double RIGHT_CORAL_KMAX_OUTPUT = 1.0;
            public static final double RIGHT_CORAL_KMIN_OUTPUT = -1.0;
            public static final float RIGHT_CORAL_pCONV = 1.0f;
            public static final float RIGHT_CORAL_vCONV = 1.0f;

    
            public static final double ALGAE_KP = 0.1;
            public static final double ALGAE_KI = 0.0;
            public static final double ALGAE_KD = 0.0;
            public static final double ALGAE_KFF = 0.0;
            public static final double ALGAE_KMAX_OUTPUT = 1.0;
            public static final double ALGAE_KMIN_OUTPUT = -1.0;
            public static final float ALGAE_pCONV = 1.0f;
            public static final float ALGAE_vCONV = 1.0f;
    
            public static final double ROT_CORAL_KP = 0.1;
            public static final double ROT_CORAL_KI = 0.0;
            public static final double ROT_CORAL_KD = 0.0;
            public static final double ROT_CORAL_KFF = 0.0;
            public static final double ROT_CORAL_KMAX_OUTPUT = 1.0;
            public static final double ROT_CORAL_KMIN_OUTPUT = -1.0;
            public static final float ROT_CORAL_pCONV = 1.0f;
            public static final float ROT_CORAL_vCONV = 1.0f;
    
            public static final double ARM_ALGAE_KP = 0.1;
            public static final double ARM_ALGAE_KI = 0.0;
            public static final double ARM_ALGAE_KD = 0.0;
            public static final double ARM_ALGAE_KFF = 0.0;
            public static final double ARM_ALGAE_KMAX_OUTPUT = 1.0;
            public static final double ARM_ALGAE_KMIN_OUTPUT = -1.0;
            public static final float ARM_ALGAE_pCONV = 1.0f;
            public static final float ARM_ALGAE_vCONV = 1.0f;
    
            public static final double ARM_ALGAE_2_KP = 0.1;
            public static final double ARM_ALGAE_2_KI = 0.0;
            public static final double ARM_ALGAE_2_KD = 0.0;
            public static final double ARM_ALGAE_2_KFF = 0.0;
            public static final double ARM_ALGAE_2_KMAX_OUTPUT = 1.0;
            public static final double ARM_ALGAE_2_KMIN_OUTPUT = -1.0;
            public static final float ARM_ALGAE_2_pCONV = 1.0f;
            public static final float ARM_ALGAE_2_vCONV = 1.0f;
        }

        public static final class AngleReferences {
            public static final double CORAL_ANGLE_PROCESSOR = 0.0; // Check. In rotations
            public static final double CORAL_ANGLE_L1 = 0.0; // Check. In rotations

            public static final double ALGAE_ANGLE_DOWN = 0.0; // Check. In rotations
            public static final double ALGAE_ANGLE_UP = 0.0; // Check. In rotations
        }
    }
    
}
