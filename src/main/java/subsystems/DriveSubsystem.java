// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class DriveSubsystem extends SubsystemBase {
	private final SwerveDrive swerveDrive;
	private final Field2d m_field = new Field2d();

	// Network Table Publishers
    private final StructPublisher<Pose3d> posePublisher;
    private final StructArrayPublisher<SwerveModuleState> moduleStatesPublisher;
    private final DoublePublisher yawPublisher;
    private final DoublePublisher pitchPublisher;
    private final DoublePublisher rollPublisher;
    private final DoubleArrayPublisher chassisSpeedsPublisher;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
		try {
			swerveDrive = new SwerveParser(swerveJsonDirectory)
					.createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
		} catch (IOException i) {
			throw new RuntimeException(i);
		}

		swerveDrive.setHeadingCorrection(false);
		swerveDrive.setCosineCompensator(true);

		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
		SmartDashboard.putData("Field", m_field);

		// set initial pose
		swerveDrive.resetOdometry(new Pose2d(2, 2, new Rotation2d(0)));
		
		// Initialize publishers
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Drive/Pose", Pose3d.struct).publish();
        moduleStatesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Drive/ModuleStates", SwerveModuleState.struct).publish();
        yawPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Drive/Gyro/Yaw").publish();
        pitchPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Drive/Gyro/Pitch").publish();
        rollPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Drive/Gyro/Roll").publish();
        chassisSpeedsPublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("Drive/ChassisSpeeds").publish();
	}

	@Override
	public void periodic() {
		// Update pose
        Pose2d pose2d = swerveDrive.getPose();
        posePublisher.set(new Pose3d(pose2d.getX(), pose2d.getY(), 0.0,
            new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians())));

        // Update module states
        moduleStatesPublisher.set(swerveDrive.getStates());

        // Update gyro data
        yawPublisher.set(swerveDrive.getYaw().getDegrees());
        pitchPublisher.set(swerveDrive.getPitch().getDegrees());
        rollPublisher.set(swerveDrive.getRoll().getDegrees());

        // Update chassis speeds
        ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
        chassisSpeedsPublisher.set(new double[] {
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
        });

        // Update field visualization
        m_field.setRobotPose(pose2d);
	}

	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	/**
	 * Command to drive the robot using translative values and heading as angular
	 * velocity.
	 *
	 * @param translationX     Translation in the X direction.
	 * @param translationY     Translation in the Y direction.
	 * @param angularRotationX Rotation of the robot to set
	 * @return Drive command.
	 */
	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
			DoubleSupplier angularRotationX) {
		return run(() -> {
			// Make the robot move
			swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
					translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
					angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
					true,
					false,
					new Translation2d(0 - Units.inchesToMeters(Constants.DriveConstants.kGyroOffsetX), 0.0));
		});
	}
}