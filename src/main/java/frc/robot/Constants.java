package frc.robot;

public class Constants {
    // MOTOR_ID are for the CAN ids of the motors used by each subsystem
    // DUTYCYCLE is the percentage of power sent to a motor for an action (between -1 and 1)
    // KP, KI, and KD are constant values for PID controllers
    // POSITION is the relative encoder position for the elevator and ANGLE is the absolute encoder position for the pivot
    // DEADBAND is the minimum value for the controller to register as a movement
    
    public static class DriveConstants {
        public static final double MAX_SPEED_MS = 4.92;
        public static final double CONTROLLER_DEADBAND = 0.5;
    }

    public static final class AlgaeConstants {
        public static final int ROLLER_MOTOR_ID = 12;

        public static final double ALGAE_INTAKE_DUTYCYCLE = -0.8;
        public static final double ALGAE_SCORE_DUTYCYCLE = 0.4;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 13;

        public static final double ARM_DOWN_DUTYCYCLE = 0.4;
        public static final double ARM_UP_DUTYCYCLE = -0.4;

        public static final double ARM_HOLD_DOWN_DUTYCYCLE = 0.1;
        public static final double ARM_HOLD_UP_DUTYCYCLE = -0.15;

        public static final double PIVOT_KP = 0.1;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.0;
    }

    public static class ClimberConstants {
        public static final int CLIMBER_MOTOR_ID = 1;
        public static final double CLIMBER_UP_DUTYCYCLE = -0.5;
        public static final double CLIMBER_DOWN_DUTYCYCLE = 0.5;
    }  

    public static class CoralConstants {
<<<<<<< HEAD
        public static final int TOP_CORAL_MOTOR_ID = 15;
        public static final int BOTTOM_CORAL_MOTOR_ID = 16;

        public static final double CORAL_INTAKE_DUTYCYCLE = -0.25; // Check
        public static final double CORAL_SCORE_DUTYCYCLE = 0.25; // Check
=======
        public static final int TOP_CORAL_MOTOR_ID = 16;
        public static final int BOTTOM_CORAL_MOTOR_ID = 17; 
        public static final int CORAL_ARM_MOTOR_ID = 11;

        public static final double CORAL_INTAKE_DUTYCYCLE = -1; // Check
        public static final double CORAL_SCORE_DUTYCYCLE = 1; // Check

        public static final double CORAL_ARM_UP_ANGLE = 55; // Check
        public static final double CORAL_ARM_DOWN_ANGLE = -35; // Check
        public static final double CORAL_ARM_NEUTRAL_ANGLE = 0.0; // Check
    }

    public static class AlgaeConstants {
        public static final int RIGHT_ALGAE_ARM_MOTOR_ID = 12; 
        public static final int LEFT_ALGAE_ARM_MOTOR_ID = 13; 

        public static final int RIGHT_ALGAE_WHEEL_MOTOR_ID = 14; 
        public static final int LEFT_ALGAE_WHEEL_MOTOR_ID = 15; 

        public static final double ALGAE_WHEEL_INTAKE_DUTYCYCLE = -1; // Check
        public static final double ALGAE_WHEEL_OUTTAKE_DUTYCYCLE = 1; // Check
        // Algae arm inside the robot perimeter
        public static final int ALGAE_ARM_IN_ANGLE = 0; // Placeholder
        // Algae arm outside the robot perimeter
        public static final int ALGAE_ARM_OUT_ANGLE = 1; // Placeholder
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c
    }

    public static class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID_R = 9;
        public static final int ELEVATOR_MOTOR_ID_L = 10;

        // in relative encoder rotations
        public static final double ELEVATOR_L3_POSITION = -52; // 121 cm or 3 ft 11 5/8 in above carpet
        public static final double ELEVATOR_L2_POSITION = -25; // 81 cm or 2 ft 7/8 in above carpet
<<<<<<< HEAD
        public static final double ELEVATOR_L1_POSITION = 0.0;
        public static final double ELEVATOR_SOURCE_POSITION = -20.75;
=======
        public static final double ELEVATOR_L1_POSITION = 0.0; // type shit
        public static final double ELEVATOR_SOURCE_POSITION = -20.75; // Check
    }

    public static class PID {
        // public static final double CORAL_MOTOR_KP = 0.1;
        // public static final double CORAL_MOTOR_KI = 0.0;
        // public static final double CORAL_MOTOR_KD = 0.0;
        // public static final double CORAL_MOTOR_KFF = 0.0;
        // public static final double CORAL_MOTOR_MAX_OUTPUT = 1.0;
        // public static final double CORAL_MOTOR_MIN_OUTPUT = -1.0;
        // public static final float CORAL_MOTOR_pCONV = 1.0f;
        // public static final float CORAL_MOTOR_vCONV = 1.0f;

        public static final double RIGHT_CORAL_KP = 0.1;
        public static final double RIGHT_CORAL_KI = 0.0;
        public static final double RIGHT_CORAL_KD = 0.0;
        public static final double RIGHT_CORAL_KFF = 0.0;
        public static final double RIGHT_CORAL_KMAX_OUTPUT = 1.0;
        public static final double RIGHT_CORAL_KMIN_OUTPUT = -1.0;
        public static final float RIGHT_CORAL_pCONV = 1.0f;
        public static final float RIGHT_CORAL_vCONV = 1.0f;

        // public static final double ALGAE_KP = 0.1;
        // public static final double ALGAE_KI = 0.0;
        // public static final double ALGAE_KD = 0.0;
        // public static final double ALGAE_KFF = 0.0;
        // public static final double ALGAE_KMAX_OUTPUT = 1.0;
        // public static final double ALGAE_KMIN_OUTPUT = -1.0;
        // public static final float ALGAE_pCONV = 1.0f;
        // public static final float ALGAE_vCONV = 1.0f;

        public static final double PIVOT_CORAL_KP = 0.001;
        public static final double PIVOT_CORAL_KI = 0.0;
        public static final double PIVOT_CORAL_KD = 0.0;
        public static final double PIVOT_CORAL_KFF = 0.0;
        public static final double PIVOT_CORAL_KMAX_OUTPUT = 1.0;
        public static final double PIVOT_CORAL_KMIN_OUTPUT = -1.0;
        public static final float PIVOT_CORAL_pCONV = 1.0f;
        public static final float PIVOT_CORAL_vCONV = 1.0f;
>>>>>>> 19ee2b9966ca790c18f9a87a50e58bccb131405c

        public static final double ELEVATOR_KP = 0.15;
        public static final double ELEVATOR_KI = 0.0;
        public static final double ELEVATOR_KD = 0.0;
    }

    public static class PivotConstants {
        public static final int PIVOT_MOTOR_ID = 11;

        // in degrees
        public static final double PIVOT_UP_ANGLE = 55;
        public static final double PIVOT_DOWN_ANGLE = -35;
        public static final double PIVOT_NEUTRAL_ANGLE = 0;

        public static final double PIVOT_KP = 0.1;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.0;
    }
}