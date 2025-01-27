// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.util.Units.*;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
    } catch (IOException i) {
      throw new RuntimeException(i);
    }

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(true);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro Rotation", swerveDrive.getYaw().getDegrees());
    SmartDashboard.putNumber("Heading", swerveDrive.getOdometryHeading().getDegrees());
    m_field.setRobotPose(swerveDrive.getPose());
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false,
                        new Translation2d(0 - Units.inchesToMaters(kGyroOffsetX), 0.0));
    });
  }
}