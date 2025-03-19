package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Constants {
    // MOTOR_ID are for the CAN ids of the motors used by each subsystem
    // DUTYCYCLE is the percentage of power sent to a motor for an action (between -1 and 1)
    // KP, KI, and KD are constant values for PID controllers
    // POSITION is the relative encoder position for the elevator and ANGLE is the absolute encoder position for the pivot
    // DEADBAND is the minimum value for the controller to register as a movement
    
    public static class DriveConstants {
        public static final double MAX_SPEED_MS = 4.92;
        public static final double CONTROLLER_DEADBAND = 0.5;
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kGyroOffsetX = 5.25;
    }

    public static final class AlgaeConstants {
        public static final int ROLLER_MOTOR_ID = 12;

        public static final double ALGAE_INTAKE_DUTYCYCLE = -0.8;
        public static final double ALGAE_SCORE_DUTYCYCLE = 0.4;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 13;

        public static final double ARM_UP_ANGLE = 90;
        public static final double ARM_DOWN_ANGLE = 45;

        public static final double ARM_KP = 0.1;
        public static final double ARM_KI = 0.0;
        public static final double ARM_KD = 0.0;
    }

    public static class ClimberConstants {
        public static final int CLIMBER_MOTOR_ID = 14;
        public static final double CLIMBER_UP_DUTYCYCLE = -0.5;
        public static final double CLIMBER_DOWN_DUTYCYCLE = 0.5;
    }  

    public static class CoralConstants {
        public static final int TOP_CORAL_MOTOR_ID = 15;
        public static final int BOTTOM_CORAL_MOTOR_ID = 16;

        public static final double CORAL_INTAKE_DUTYCYCLE = -0.25; // Check
        public static final double CORAL_SCORE_DUTYCYCLE = 0.25; // Check
    }

    public static class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID_R = 9;
        public static final int ELEVATOR_MOTOR_ID_L = 10;

        // in relative encoder rotations
        public static final double ELEVATOR_L3_POSITION = -52; // 121 cm or 3 ft 11 5/8 in above carpet
        public static final double ELEVATOR_L2_POSITION = -25; // 81 cm or 2 ft 7/8 in above carpet
        public static final double ELEVATOR_L1_POSITION = 0.0;
        public static final double ELEVATOR_SOURCE_POSITION = -20.75;

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

    
    

    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kIdleDeadzone = 0.15;
    }

    // setpoints for auto paths
    public static final Pose2d noteCenterPose = new Pose2d(1.55, 5.37, Rotation2d.fromDegrees(180.00));
    public static final Pose2d noteTopPose = new Pose2d(0.62, 6.60, Rotation2d.fromDegrees(60.00));
    public static final Pose2d noteBottomPose = new Pose2d(0.62, 4.48, Rotation2d.fromDegrees(-60.00));

    private static final Map<String, Pose2d> poseMap = new HashMap<String, Pose2d>() {
        {
            put("q", Constants.noteTopPose);
            put("w", Constants.noteCenterPose);
            put("e", Constants.noteBottomPose);
        }
    };
}
