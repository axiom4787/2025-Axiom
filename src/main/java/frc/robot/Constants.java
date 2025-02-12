// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Constants {
    public static class DriveConstants {
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kGyroOffsetX = 5.25;
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
