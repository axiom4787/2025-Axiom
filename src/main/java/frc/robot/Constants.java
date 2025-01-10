package frc.robot;

import org.ejml.equation.Variable;

public class Constants {

    public static Machine state = Machine.CoralOuttake;

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
    
    public static class cIntake {

        public static float m_default_speed = 1.0f;

        public static class m_Intake_Motors{
            public static final int motor_coral_1 = 1; // Check  
            public static final int motor_coral_2 = 2; // Check  
            public static final int motor_rot_coral = 3; // Check  
    
            public static final int motor_arm_algae_1 = 4; // Check
            public static final int motor_arm_algae_2 = 5; // Fix & Check
    
            public static final int motor_right_algae = 6; // Check
            public static final int motor_left_algae = 7; // Check
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
            public static final double CORAL_ANGLE_L1 = 0.5; // Check. In rotations

            public static final double ALGAE_ANGLE_DOWN = 0.0; // Check. In rotations
            public static final double ALGAE_ANGLE_UP = 1; // Check. In rotations
        }
        public static class Arm_Angles{
            public static final float arm_angle_l1 = 0; // Check
            public static final float arm_angle_l2 = 1; // Check
            public static final float arm_angle_l3 = 2; // Check
            public static final float arm_angle_l4 = 3; // Check
        }
    }
}