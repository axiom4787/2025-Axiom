package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    // MOTOR_ID are for the CAN ids of the motors used by each subsystem
    // DUTYCYCLE is the percentage of power sent to a motor for an action (between
    // -1 and 1)
    // KP, KI, and KD are constant values for PID controllers
    // POSITION is the relative encoder position for the elevator and ANGLE is the
    // absolute encoder position for the pivot
    // DEADBAND is the minimum value for the controller to register as a movement

    public static class DriveConstants {
        public static final double MAX_SPEED_MS = 5.740;
        public static final double CONTROLLER_DEADBAND = 0.3;

        // Simulation-specific constants
        public static final double SIM_VELOCITY_DEADBAND = 0.001; // 1 mm/s
        public static final double SIM_ROTATION_DEADBAND = 0.001; // ~0.06 deg/s
    }

    public static final class AlgaeConstants {
        public static final int ROLLER_MOTOR_ID = 12;

        public static final double ALGAE_INTAKE_DUTYCYCLE = -0.8;
        public static final double ALGAE_SCORE_DUTYCYCLE = 0.4;
    }

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 13;

        public static final double ARM_DOWN_DUTYCYCLE = -0.4;
        public static final double ARM_UP_DUTYCYCLE = 0.4;

        public static final double ARM_HOLD_DOWN_DUTYCYCLE = -0.0;
        public static final double ARM_HOLD_UP_DUTYCYCLE = 0.05;
    }

    public static class ClimberConstants {
        public static final int CLIMBER_MOTOR_ID = 14;
        public static final double CLIMBER_UP_DUTYCYCLE = -0.75;
        public static final double CLIMBER_DOWN_DUTYCYCLE = 0.75;
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
        public static final double ELEVATOR_L3_POSITION = -56; // 121 cm or 3 ft 11 5/8 in above carpet
        public static final double ELEVATOR_L2_POSITION = -29; // 81 cm or 2 ft 7/8 in above carpet
        public static final double ELEVATOR_L1_POSITION = 0.0;
        public static final double ELEVATOR_SOURCE_POSITION = -20.75;

        public static final double ELEVATOR_KP = 0.15;
        public static final double ELEVATOR_KI = 0.0;
        public static final double ELEVATOR_KD = 0.0015;
    }

    public static class PivotConstants {
        public static final int PIVOT_MOTOR_ID = 11;
        //private static final double PIVOT_OFFSET = 20;
        // in degrees
        // public static final double PIVOT_UP_ANGLE = 86-PIVOT_OFFSET;
        // public static final double PIVOT_DOWN_ANGLE = 142-PIVOT_OFFSET;
        // public static final double PIVOT_NEUTRAL_ANGLE = 122-PIVOT_OFFSET;
        public static final double PIVOT_UP_ANGLE = 315;
        public static final double PIVOT_DOWN_ANGLE = 20;
        public static final double PIVOT_NEUTRAL_ANGLE = 0;

        public static final double PIVOT_KP = 0.01;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.0;
    }

    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kIdleDeadzone = 0.15;
    }
}