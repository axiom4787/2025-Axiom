// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS4Controller;

/** Add your docs here. */
public class Constants {
    public static class DriveConstants {
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kGyroOffsetX = -5.25;
    }

    public enum ControllerType {
        XBOX,
        PS4,
        PS5
    }

    public enum ControllerAction {
        DRIVE_Y,
        DRIVE_X,
        DRIVE_ROT,
        INTAKE,
        SHOOT,
        CLIMB
    }
}
