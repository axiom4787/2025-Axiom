package frc.robot;

public class Constants {
    public static final float m_default_speed = 1.0f;

    public static class DriveConstants {
        public static final double MAX_SPEED_MS = 4.92;
        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    public static class CoralConstants {
        public static final int TOP_CORAL_MOTOR_ID = 16;
        public static final int BOTTOM_CORAL_MOTOR_ID = 17; 

        public static final double CORAL_INTAKE_DUTYCYCLE = -1; // Check
        public static final double CORAL_SCORE_DUTYCYCLE = 1; // Check
    }

    public static class AlgaeConstants {
        public static final int RIGHT_ALGAE_ARM_MOTOR_ID = 12; 
        public static final int LEFT_ALGAE_ARM_MOTOR_ID = 13; 

        public static final int RIGHT_ALGAE_WHEEL_MOTOR_ID = 14; 
        public static final int LEFT_ALGAE_WHEEL_MOTOR_ID = 15; 

        public static final double ALGAE_WHEEL_INTAKE_DUTYCYCLE = -1; // Check
        public static final double ALGAE_WHEEL_OUTTAKE_DUTYCYCLE = 1; // Check
        public static final int ALGAE_ARM_EMPTY_ANGLE = 0; // Placeholder
        public static final int ALGAE_ARM_FULL_ANGLE = 1; // Placeholder
    }

    public static class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID_R = 9;
        public static final int ELEVATOR_MOTOR_ID_L = 10;
        public static final int CORAL_ARM_MOTOR_ID = 11;


        // values should be in relative encoder rotations
        public static final double ELEVATOR_L3_POSITION = -52; // 121 cm or 3 ft 11 5/8 in above carpet
        public static final double ELEVATOR_L2_POSITION = -25; // 81 cm or 2 ft 7/8 in above carpet
        public static final double ELEVATOR_L1_POSITION = 0.0; // type shit
       public static final double ELEVATOR_SOURCE_POSITION = -20.75; // Check
        public static final double CORAL_ARM_UP_ANGLE = 55; // Check
        public static final double CORAL_ARM_DOWN_ANGLE = -35; // Check
        public static final double CORAL_ARM_NEUTRAL_ANGLE = 0.0; // Check
    }

    public static class PID {
        public static final double CORAL_MOTOR_KP = 0.1;
        public static final double CORAL_MOTOR_KI = 0.0;
        public static final double CORAL_MOTOR_KD = 0.0;
        public static final double CORAL_MOTOR_KFF = 0.0;
        public static final double CORAL_MOTOR_MAX_OUTPUT = 1.0;
        public static final double CORAL_MOTOR_MIN_OUTPUT = -1.0;
        public static final float CORAL_MOTOR_pCONV = 1.0f;
        public static final float CORAL_MOTOR_vCONV = 1.0f;

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

        public static final double ELEVATOR_KP = 0.15;
        public static final double ELEVATOR_KI = 0.0;
        public static final double ELEVATOR_KD = 0.0;
        public static final double ELEVATOR_KFF = -0.05;
        public static final double ELEVATOR_KMAX_OUTPUT = 1.0;
        public static final double ELEVATOR_KMIN_OUTPUT = -1.0;
        public static final float ELEVATOR_pCONV = 1.0f;
        public static final float ELEVATOR_vCONV = 1.0f;

        public static final double ARM_ALGAE_KP = 0.1;
        public static final double ARM_ALGAE_KI = 0.0;
        public static final double ARM_ALGAE_KD = 0.0;
        public static final double ARM_ALGAE_KFF = 0.0;
        public static final double ARM_ALGAE_KMAX_OUTPUT = 1.0;
        public static final double ARM_ALGAE_KMIN_OUTPUT = -1.0;
        public static final float ARM_ALGAE_pCONV = 1.0f;
        public static final float ARM_ALGAE_vCONV = 1.0f;

    }
}