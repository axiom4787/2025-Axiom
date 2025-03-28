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
        public static final double CONTROLLER_DEADBAND = 0.5;

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
        public static final double PIVOT_DOWN_ANGLE = 15;
        public static final double PIVOT_NEUTRAL_ANGLE = 0;

        public static final double PIVOT_KP = 0.025;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.0025;
    }

    public static final class Drivetrain {
        public static final double ROBOT_WIDTH = 0.85; // meters - adjust to your robot's width
        public static final double ROBOT_LENGTH = 0.85; // meters - adjust to your robot's length

        
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeed = 5;
        public static final double kMaxAcceleration = 5;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
        public static final double kMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(31);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(31);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = Math.PI;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = 0;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 5;
        public static final int kRearLeftDrivingCanId = 3;
        public static final int kFrontRightDrivingCanId = 1;
        public static final int kRearRightDrivingCanId = 7;

        public static final int kFrontLeftTurningCanId = 6;
        public static final int kRearLeftTurningCanId = 4;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 8;

        public static final boolean kGyroReversed = false;
        public static final double kHeadingP = 0;
        public static final double kHeadingI = 0;
        public static final double kHeadingD = 0;
        public static final double kHeadingJoystickThreshold = 0.1;
    }

    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kIdleDeadzone = 0.15;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 12;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    // setpoints for auto paths
    public static final Pose2d noteCenterPose = new Pose2d(7, 2, Rotation2d.fromDegrees(180.00));
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
