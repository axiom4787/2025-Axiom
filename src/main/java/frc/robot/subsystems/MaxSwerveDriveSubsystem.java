// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward; // Keep import for potential future use, though method removed
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory; // Keep import for postTrajectory placeholder
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine; // Needed for SysId placeholders

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

// PathPlanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants; // Renamed from PathPlanner's PIDConstants to avoid clash
import com.pathplanner.lib.config.RobotConfig; // Placeholder for where RobotConfig comes from
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint; // Needed for Setpoint Generator
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator; // Needed for Setpoint Generator

import frc.robot.Constants; // Assuming constants are defined here
import frc.robot.Constants.Drivetrain; // Assuming Drivetrain constants nested here
import frc.robot.utils.SwerveModuleStateLogger; // If you use this utility

import java.io.File; // Keep import for constructor signature, though unused
import java.io.IOException; // For Setpoint Generator
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MaxSwerveDriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            Drivetrain.kFrontLeftDrivingCanId,
            Drivetrain.kFrontLeftTurningCanId,
            Drivetrain.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            Drivetrain.kFrontRightDrivingCanId,
            Drivetrain.kFrontRightTurningCanId,
            Drivetrain.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            Drivetrain.kRearLeftDrivingCanId,
            Drivetrain.kRearLeftTurningCanId,
            Drivetrain.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            Drivetrain.kRearRightDrivingCanId,
            Drivetrain.kRearRightTurningCanId,
            Drivetrain.kBackRightChassisAngularOffset);

    // Array for easy access
    private final MAXSwerveModule[] m_modules = { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight };

    // The gyro sensor (Studica AHRS NavX)
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    // Odometry class for tracking robot pose
    private final SwerveDriveOdometry m_odometry;

    // PID controller for heading control during teleop drive commands
    private final ProfiledPIDController m_headingController = new ProfiledPIDController(
            Drivetrain.kHeadingP, Drivetrain.kHeadingI, Drivetrain.kHeadingD,
            new TrapezoidProfile.Constraints(
                    Drivetrain.kMaxAngularSpeed, // Max angular velocity
                    Drivetrain.kMaxAngularAcceleration // Max angular acceleration
            ));

    // PathPlanner Command Tracking
    private Command currentPathCommand = null;

    // Vision updates flag (from original code, use as needed)
    private final boolean visionDriveTest = false; // Set to true to potentially modify odometry updates

    /** Creates a new DriveSubsystem. */
    public MaxSwerveDriveSubsystem() {
        // Configure the Studica NavX Gyro
        // m_gyro.calibrate(); // Calibrate if needed, might take time
        m_gyro.reset(); // Reset to 0 degrees yaw

        m_odometry = new SwerveDriveOdometry(
                Drivetrain.kDriveKinematics,
                getGyroRotation2d(),
                getModulePositions());

        // Configure heading PID controller
        m_headingController.enableContinuousInput(-Math.PI, Math.PI); // Radians
        m_headingController.setTolerance(Units.degreesToRadians(1.0)); // Tolerance in radians

        // Configure PathPlanner
        setupPathPlanner();

        // Zero heading based on alliance at autonomous init
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));


        // Usage reporting for MAXSwerve template - Keep this
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    }

    // --- Core Periodic Updates ---

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                getGyroRotation2d(),
                getModulePositions());

        

        // --- Debug Logging (Optional - from original code) ---
        // System.out.println("=== SWERVE DEBUG INFO ===");
        // System.out.println("Current Pose: " + getPose());
        // System.out.println("Heading: " + getHeading().getDegrees() + " degrees");
        // System.out.println("Raw Gyro: " + getGyroRotation2d().getDegrees());
        // System.out.println("Robot Velocity: " + getRobotVelocity());
        // System.out.println("Field Velocity: " + getFieldVelocity());
        // var states = getModuleStates();
        // var desiredStates = getDesiredModuleStates(); // Need to implement getDesiredModuleStates if needed
        // for (int i = 0; i < states.length; i++) {
        //     System.out.println("Module " + i +
        //             " - Actual: [" + states[i].angle.getDegrees() + "°, " +
        //             states[i].speedMetersPerSecond + " m/s]" //+
        //            // " - Desired: [" + desiredStates[i].angle.getDegrees() + "°, " +
        //            // desiredStates[i].speedMetersPerSecond + " m/s]"
        //            );
        // }
        // --- End Debug Logging ---
    }

    @Override
    public void simulationPeriodic() {
        // --- Simulation Debug Logging (Optional - Needs Sim Support) ---
        // Note: MAXSwerveModule and Studica AHRS might need specific simulation setup
        // System.out.println("=== SIMULATION DEBUG ===");
        // System.out.println("Sim Gyro Reading: " + getGyroRotation2d().getDegrees()); // May not work without sim setup
        // System.out.println("Sim Pose: " + getPose());
        // var chassisSpeeds = getRobotVelocity(); // Should work if module sim works
        // System.out.println("Chassis Speeds: [vx=" + chassisSpeeds.vxMetersPerSecond +
        //         ", vy=" + chassisSpeeds.vyMetersPerSecond +
        //         ", omega=" + chassisSpeeds.omegaRadiansPerSecond + "]");
        // --- End Simulation Debug Logging ---

        // Send pose data to Advantage Scope for visualization
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("AdvantageScope");
        Pose2d currentPose = getPose();
        double[] poseData = new double[] {
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().getRadians()
        };
        table.getEntry("robotPoseFromMaxSwerve").setDoubleArray(poseData);
    }

    // --- PathPlanner Setup and Commands ---

    /**
	 * Setup AutoBuilder for PathPlanner.
	 */
	public void setupPathPlanner() {
		// Load the RobotConfig from the GUI settings. You should probably
		// store this in your Constants file
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();

			final boolean enableFeedforward = false;
			// Configure AutoBuilder last
			AutoBuilder.configure(
					this::getPose,
					// Robot pose supplier
					this::resetOdometry,
					// Method to reset odometry (will be called if your auto has a starting pose)
					this::getRobotVelocity,
					// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
					(speedsRobotRelative, moduleFeedForwards) -> {
						setChassisSpeeds(speedsRobotRelative);
					},
					// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
					// optionally outputs individual module feedforwards
					new PPHolonomicDriveController(
							// PPHolonomicController is the built in path following controller for holonomic
							// drive trains
							new PIDConstants(5.0, 0.0, 0.0),
							// Translation PID constants
							new PIDConstants(5.0, 0.0, 0.0)
					// Rotation PID constants
					),
					config,
					// The robot configuration
					() -> {
						// Boolean supplier that controls when the path will be mirrored for the red
						// alliance
						// This will flip the path being followed to the red side of the field.
						// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					},
					this
			// Reference to this subsystem to set requirements
			);

		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}

		// Preload PathPlanner Path finding
		// IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
		PathfindingCommand.warmupCommand().schedule();
	}

    /**
     * Get the path follower command for a given path name.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        Command pathCommand = new PathPlannerAuto(pathName);
        currentPathCommand = pathCommand; // Track the command
        return pathCommand;
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                Drivetrain.kMaxSpeed,
                Drivetrain.kMaxAcceleration, // Make sure this is defined in Constants
                Drivetrain.kMaxAngularSpeed, // Make sure this is defined
                Drivetrain.kMaxAngularAcceleration // Make sure this is defined
                );

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathCommand = AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );
        currentPathCommand = pathCommand; // Track the command
        return pathCommand;
    }


    /**
     * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
     * NOTE: Requires RobotConfig to be defined/loaded. Feedforward part might not apply directly to SPARK MAX internal PIDF.
     *
     * @param robotRelativeChassisSpeed Supplier for Robot relative {@link ChassisSpeeds} to achieve.
     * @return {@link Command} to run.
     * @throws RuntimeException If RobotConfig cannot be loaded.
     */
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed) {
        // TODO: Determine how RobotConfig is sourced. Hardcoded? Constants? File?
        // For now, assuming it's accessible via a static method or instance.
        // This is a placeholder - replace with your actual RobotConfig source.
        RobotConfig robotConfig;
        try {
             robotConfig = RobotConfig.fromGUISettings();
         } catch (Exception e) {
             throw new RuntimeException("Failed to load RobotConfig for Setpoint Generator", e);
         }


        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(robotConfig,
                Drivetrain.kMaxAngularSpeed); // Use max angular speed from constants

        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
                new SwerveSetpoint(getRobotVelocity(), getModuleStates(), DriveFeedforwards.zeros(m_modules.length)));
        AtomicReference<Double> previousTime = new AtomicReference<>();

        return runOnce(() -> previousTime.set(Timer.getFPGATimestamp()))
            .andThen(run(() -> {
                double newTime = Timer.getFPGATimestamp();
                double dt = newTime - previousTime.get();
                if (dt <= 0) dt = 0.02; // Avoid division by zero or negative dt

                SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                        robotRelativeChassisSpeed.get(),
                        dt); // Use calculated dt

                // Set module states based on the generated setpoint
                setModuleStates(newSetpoint.moduleStates());

                prevSetpoint.set(newSetpoint);
                previousTime.set(newTime);
            })).withName("DriveWithSetpointGenerator");
    }


    /**
     * Drive with 254's Setpoint generator (Field Relative); port written by PathPlanner.
     *
     * @param fieldRelativeSpeeds Supplier for Field-Relative {@link ChassisSpeeds}
     * @return Command to drive the robot using the setpoint generator.
     */
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(() ->
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading())
            );
        } catch (Exception e) {
            DriverStation.reportError("Error creating Setpoint Generator Command: " + e.toString(), e.getStackTrace());
            return Commands.none();
        }
    }

    // --- SysId Commands (Placeholders) ---

    /**
     * Command to characterize the robot drive motors using SysId.
     * NOTE: This is a placeholder. Direct SysId with MAXSwerveModule requires
     * using WPILib's SysIdRoutine directly, which is more complex than the YAGSL helper.
     *
     * @return Placeholder SysId Drive Command (returns Commands.none()).
     */
    public Command sysIdDriveMotorCommand() {
        System.out.println("WARNING: sysIdDriveMotorCommand is a placeholder in MaxSwerveDriveSubsystem.");
        // Example structure (needs correct mechanism/motor setup):
        // SysIdRoutine routine = new SysIdRoutine(
        //      new SysIdRoutine.Config(),
        //      new SysIdRoutine.Mechanism(
        //              (Measure<Voltage> voltage) -> /* Set voltage to drive motors */,
        //              log -> { /* Log drive motor state (pos, vel) */ },
        //              this
        //      ));
        // return routine.quasistatic(SysIdRoutine.Direction.kForward); // Or dynamic tests
        return Commands.none().withName("SysIdDrivePlaceholder");
    }

    /**
     * Command to characterize the robot angle motors using SysId.
     * NOTE: This is a placeholder. See note for sysIdDriveMotorCommand.
     *
     * @return Placeholder SysId Angle Command (returns Commands.none()).
     */
    public Command sysIdAngleMotorCommand() {
        System.out.println("WARNING: sysIdAngleMotorCommand is a placeholder in MaxSwerveDriveSubsystem.");
        // Similar structure to drive, but controlling/logging angle motors.
        return Commands.none().withName("SysIdAnglePlaceholder");
    }

    // --- Basic Drive Commands & Helpers ---

    /**
     * Returns a Command that centers the modules (sets angles to 0 degrees).
     *
     * @return A Command that centers the modules.
     */
    public Command centerModulesCommand() {
        return run(() -> {
            for (MAXSwerveModule module : m_modules) {
                // Setting angle requires PID control, setting state directly is better
                 module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
            }
        }).withName("CenterModules");
    }


    /**
     * Returns a Command that drives the swerve drive forward a specific distance at a given speed.
     * Assumes starting at pose (0,0). Resets odometry at the start.
     *
     * @param distanceInMeters       the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per second
     * @return a Command that drives the swerve drive forward a specific distance
     */
    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
        // Ensure speed is positive, distance determines direction effectively
        final double effectiveSpeed = Math.abs(speedInMetersPerSecond) * Math.signum(distanceInMeters);
        final double targetDistance = Math.abs(distanceInMeters);
        final Translation2d startTranslation = new Translation2d(); // Assuming start at 0,0 after reset

        return Commands.runOnce(() -> resetOdometry(new Pose2d(startTranslation, getHeading()))) // Reset odometry at start
                .andThen(run(() -> driveRobotRelative(new ChassisSpeeds(effectiveSpeed, 0, 0))))
                .until(() -> getPose().getTranslation().getDistance(startTranslation) >= targetDistance)
                .finallyDo((interrupted) -> stopModules()); // Stop motors when done or interrupted
    }


    /**
     * Placeholder for replacing feedforward. MAXSwerve uses internal PIDF.
     * Direct replacement isn't typical. Gains can be adjusted via MAXSwerveModule methods.
     */
    // public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    //     System.out.println("WARNING: replaceSwerveModuleFeedforward is not directly supported in MaxSwerveDriveSubsystem. Adjust PIDF gains instead.");
    //     // If you need to apply custom FF, you might need to calculate it manually
    //     // and add it to the output, or use voltage control mode.
    // }


    /**
     * Command to drive the robot using joystick values (robot-relative).
     * Inputs are typically squared/cubed for better control feel.
     *
     * @param translationX Supplier for forward/backward motion (-1 to 1). Positive is forward.
     * @param translationY Supplier for strafe motion (-1 to 1). Positive is left.
     * @param rotation     Supplier for rotational motion (-1 to 1). Positive is CCW.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        // Applies scaling and potentially deadband if needed
        return run(() -> {
            double xSpeed = translationX.getAsDouble();
            double ySpeed = translationY.getAsDouble();
            double rot = rotation.getAsDouble();

            // Apply deadband if needed: xSpeed = MathUtil.applyDeadband(xSpeed, Drivetrain.kDriveDeadband);
            // Apply cubing/squaring for sensitivity: xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);

            // Scale speeds to max speed
            double xSpeedMPS = xSpeed * Drivetrain.kMaxSpeed;
            double ySpeedMPS = ySpeed * Drivetrain.kMaxSpeed;
            double rotRadPerSec = rot * Drivetrain.kMaxAngularSpeed; // Radians per second

            // Drive robot relative
            driveRobotRelative(new ChassisSpeeds(xSpeedMPS, ySpeedMPS, rotRadPerSec));

        }).withName("TeleopDriveRobotRelative");
    }


     /**
     * Command to drive the robot using joystick values (field-relative).
     * Inputs are typically squared/cubed for better control feel.
     *
     * @param translationX Supplier for forward/backward motion (-1 to 1). Positive is field X+.
     * @param translationY Supplier for strafe motion (-1 to 1). Positive is field Y+.
     * @param rotation     Supplier for rotational motion (-1 to 1). Positive is CCW.
     * @return Drive command.
     */
    public Command driveFieldRelativeCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        return run(() -> {
            double xSpeed = translationX.getAsDouble();
            double ySpeed = translationY.getAsDouble();
            double rot = rotation.getAsDouble();

            // Apply deadband/cubing as needed here...

            // Scale speeds
            double xSpeedMPS = xSpeed * Drivetrain.kMaxSpeed;
            double ySpeedMPS = ySpeed * Drivetrain.kMaxSpeed;
            double rotRadPerSec = rot * Drivetrain.kMaxAngularSpeed;

            // Drive field relative
            driveFieldRelative(new ChassisSpeeds(xSpeedMPS, ySpeedMPS, rotRadPerSec));

        }).withName("TeleopDriveFieldRelative");
    }


    /**
     * Command to drive the robot translatively using one joystick and maintain an angle
     * using a PID controller based on the second joystick's direction.
     *
     * @param translationX Supplier for translation X component (-1 to 1).
     * @param translationY Supplier for translation Y component (-1 to 1).
     * @param headingX     Supplier for heading joystick X component (-1 to 1).
     * @param headingY     Supplier for heading joystick Y component (-1 to 1).
     * @return Drive command.
     */
    public Command driveCommandHeadingLocked(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            double xSpeed = translationX.getAsDouble(); // Deadband/cubing here if needed
            double ySpeed = translationY.getAsDouble(); // Deadband/cubing here if needed
            double hX = headingX.getAsDouble();
            double hY = headingY.getAsDouble();

            // Scale translation speeds
            double xSpeedMPS = xSpeed * Drivetrain.kMaxSpeed;
            double ySpeedMPS = ySpeed * Drivetrain.kMaxSpeed;

            // Calculate desired heading angle from joystick
            // Only update target angle if joystick is moved beyond a threshold
            double desiredAngleRad = m_headingController.getGoal().position; // Keep current goal by default
            if (Math.hypot(hX, hY) > Drivetrain.kHeadingJoystickThreshold) { // Define this threshold in Constants
                 desiredAngleRad = Math.atan2(hY, hX); // atan2 gives angle in radians
            }

            // Calculate rotational velocity using PID controller
            double currentAngleRad = getHeading().getRadians();
            double rotRadPerSec = m_headingController.calculate(currentAngleRad, desiredAngleRad);

            // Create field-relative speeds (assuming translation inputs are field-relative)
            ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeedMPS, ySpeedMPS, rotRadPerSec);

            // Drive field relative
            driveFieldRelative(fieldSpeeds);

        }).withName("TeleopDriveHeadingLocked");
    }


     // --- Core Drive Methods ---

    /**
     * Primary drive method using ChassisSpeeds (robot-relative).
     * Converts ChassisSpeeds to module states and sets them.
     *
     * @param robotRelativeSpeeds The desired robot-relative chassis speeds.
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        // Correct for drift if applicable (from YAGSL, might need tuning/implementation)
        // ChassisSpeeds correctedSpeeds = ChassisSpeeds.correctForCoeff(robotRelativeSpeeds, Drivetrain.kChassisDiscretizationGain);

        SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        setModuleStates(moduleStates);
    }

    /**
     * Primary drive method using ChassisSpeeds (field-relative).
     * Converts field-relative speeds to robot-relative, then to module states.
     *
     * @param fieldRelativeSpeeds The desired field-relative chassis speeds.
     */
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                getHeading() // Use odometry heading
        );
        driveRobotRelative(robotRelativeSpeeds);
    }


    /**
     * Method to drive the robot using joystick info (matches original signature).
     * Scales inputs and calls driveRobotRelative or driveFieldRelative.
     *
     * @param xSpeedPercent Forward/backward input (-1 to 1). Positive is forward/field X+.
     * @param ySpeedPercent Strafe input (-1 to 1). Positive is left/field Y+.
     * @param rotPercent    Rotation input (-1 to 1). Positive is CCW.
     * @param fieldRelative Whether the translation inputs are field-relative.
     */
    public void drive(double xSpeedPercent, double ySpeedPercent, double rotPercent, boolean fieldRelative) {
        // Apply deadband/cubing if desired (e.g., inside driveCommand methods)
        double xSpeedDelivered = xSpeedPercent * Drivetrain.kMaxSpeed;
        double ySpeedDelivered = ySpeedPercent * Drivetrain.kMaxSpeed;
        double rotDelivered = rotPercent * Drivetrain.kMaxAngularSpeed; // Radians per sec

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        if (fieldRelative) {
            driveFieldRelative(speeds);
        } else {
            driveRobotRelative(speeds);
        }
    }


    /**
     * Overloaded drive method taking Translation2d and rotation rate.
     *
     * @param translation   {@link Translation2d} linear velocity vector (m/s). Interpretation
     * depends on `fieldRelative`.
     * @param rotation      Robot angular rate (rad/s). CCW positive.
     * @param fieldRelative True for field-relative translation, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
         ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
         if (fieldRelative) {
             driveFieldRelative(speeds);
         } else {
             driveRobotRelative(speeds);
         }
    }


    /**
     * Sets the desired states for each swerve module. Includes desaturation.
     *
     * @param desiredStates The desired state for each module.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Drivetrain.kMaxSpeed);

        if (desiredStates.length == m_modules.length) {
            for (int i = 0; i < m_modules.length; i++) {
                m_modules[i].setDesiredState(desiredStates[i]);
            }
        } else {
            System.err.println("ERROR: Incorrect number of SwerveModuleStates provided to setModuleStates!");
        }
    }

     /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), // Rear Left
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))  // Rear Right
        });
    }

    /** Stops all modules. */
    public void stopModules() {
         for (MAXSwerveModule module : m_modules) {
             module.stop();
         }
    }


    // --- Odometry and Pose Management ---

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose. Also resets the heading PID controller goal.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        // Reset NavX first if you want the gyro angle to match the pose angle
        // m_gyro.setAngleAdjustment(pose.getRotation().getDegrees()); // Or use reset() if starting at 0
        // Note: If gyro drift is significant, resetting odometry without resetting gyro
        // can lead to divergence between gyro angle and odometry angle. Decide on strategy.
        // For PathPlanner AutoBuilder, it expects this to reset based on the current gyro reading.
         m_odometry.resetPosition(
                getGyroRotation2d(),
                getModulePositions(),
                pose);
        // Reset heading controller goal to the new pose's rotation
        m_headingController.reset(pose.getRotation().getRadians());
    }


    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        for (MAXSwerveModule module : m_modules) {
            module.resetEncoders();
        }
        // After resetting encoders, you MUST reset odometry
        resetOdometry(new Pose2d(getPose().getTranslation(), getPose().getRotation()));
    }

    /** Zeroes the heading of the robot by resetting the gyro. Also resets odometry to maintain consistency. */
    public void zeroGyro() {
        m_gyro.reset();
        // Reset odometry to current translation but with 0 rotation
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
         System.out.println("Gyro zeroed. Current Pose: " + getPose());
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false.
     */
    private boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * Zeros the gyro. If on the red alliance, sets the odometry heading to 180 degrees.
     * Called automatically at the start of autonomous mode.
     */
    public void zeroGyroWithAlliance() {
        zeroGyro(); // Reset gyro and odometry to 0 degrees heading first
        if (isRedAlliance()) {
            // Set the odometry pose to 180 degrees while keeping the translation
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
             System.out.println("Zeroed Gyro for Red Alliance. Pose set to: " + getPose());
        } else {
             System.out.println("Zeroed Gyro for Blue Alliance. Pose set to: " + getPose());
        }
    }

     /**
     * Use vision data (if available and valid) to reset the robot's odometry.
     */
    public void resetOdometryWithVision() {
        Pose2d visionPose = getVisionPose(); // Assumes Limelight or similar source
        // Add checks for validity (e.g., non-zero, timestamp, ambiguity)
        if (visionPose != null && visionPose.getX() != 0.0 && visionPose.getY() != 0.0) { // Basic check
            System.out.println("Resetting Odometry with Vision Pose: " + visionPose);
            resetOdometry(visionPose);
            // Optionally add vision measurement to odometry instead of hard reset:
            // addVisionMeasurement(visionPose, Timer.getFPGATimestamp()); // Or timestamp from vision
        } else {
             System.out.println("Vision pose not valid for odometry reset.");
        }
    }

    /**
     * Adds a vision measurement to the SwerveDriveOdometry.
     *
     * @param visionRobotPose The pose of the robot as estimated by vision.
     * @param timestampSeconds The timestamp of the vision measurement in seconds (FPGA time).
     */
    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
        m_odometry.resetPose(visionRobotPose);
    }

    /** Add a fake vision reading for testing purposes. */
    public void addFakeVisionReading() {
        addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
         System.out.println("Added fake vision measurement.");
    }

    // --- Getters ---

    /**
     * Returns the current odometry-estimated heading of the robot.
     *
     * @return The robot's heading.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation(); // Use odometry heading which might be fused/corrected
    }

    /**
     * Returns the raw rotation from the gyro.
     * @return Rotation2d based on gyro angle.
     */
    public Rotation2d getGyroRotation2d() {
        // Negate if gyro is reversed relative to robot forward
         return Rotation2d.fromDegrees(m_gyro.getAngle() * (Drivetrain.kGyroReversed ? -1.0 : 1.0));
    }


    /**
     * Returns the turn rate of the robot from the gyro.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRateDegreesPerSec() {
        return m_gyro.getRate() * (Drivetrain.kGyroReversed ? -1.0 : 1.0);
    }

     /**
     * Returns the turn rate of the robot from the gyro.
     *
     * @return The turn rate of the robot, in radians per second
     */
    public double getTurnRateRadiansPerSec() {
        return Units.degreesToRadians(getTurnRateDegreesPerSec());
    }

    /**
     * Returns the current pitch angle from the gyro.
     * @return Pitch angle as Rotation2d.
     */
    public Rotation2d getPitch() {
        // Ensure the NavX library provides pitch correctly. Axis might need inversion.
        return Rotation2d.fromDegrees(m_gyro.getPitch());
    }


    /**
     * Returns the current state of each swerve module.
     * @return Array of SwerveModuleState objects.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
            states[i] = m_modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the current position of each swerve module.
     * @return Array of SwerveModulePosition objects.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
            positions[i] = m_modules[i].getPosition();
        }
        return positions;
    }

    /**
     * Gets the current robot-relative velocity (x, y, and omega).
     *
     * @return A {@link ChassisSpeeds} object of the current robot velocity.
     */
    public ChassisSpeeds getRobotVelocity() {
        return Drivetrain.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot.
     *
     * @return A ChassisSpeeds object of the current field-relative velocity.
     */
    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getHeading());
    }


    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return Drivetrain.kDriveKinematics;
    }

    /**
     * Get the vision-estimated pose from NetworkTables (e.g., Limelight).
     * Returns null if data is invalid or not available.
     * @return Pose2d or null.
     */
    public Pose2d getVisionPose() {
        // Example using Limelight's "botpose_wpiblue" or "botpose_wpired"
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); // Adjust table name if needed
        String poseEntryName = isRedAlliance() ? "botpose_wpired" : "botpose_wpiblue";
        double[] values = table.getEntry(poseEntryName).getDoubleArray(new double[0]);

        // Check if data is valid ( Limelight provides 7 values: x, y, z, roll, pitch, yaw, timestamp)
        if (values.length >= 6) {
            // WPILib Pose2d uses (X, Y, Rotation)
            // Limelight provides [0]=x, [1]=y, [2]=z, [3]=roll, [4]=pitch, [5]=yaw
             return new Pose2d(values[0], values[1], Rotation2d.fromDegrees(values[5]));
             // Consider using the timestamp (values[6]) for latency compensation if needed
        } else {
            // Return null or a default pose if vision data is invalid
            return null;
        }
    }


    // --- Setters ---

    /**
     * Sets the desired chassis speeds (robot-relative) to be achieved by the drivetrain.
     * This is used by PathPlanner's AutoBuilder.
     *
     * @param chassisSpeeds The desired robot-relative speeds.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        driveRobotRelative(chassisSpeeds);
    }

    // --- Other Methods ---

    /**
     * Post the trajectory to the field (Placeholder).
     * MAXSwerve doesn't have a built-in equivalent to YAGSL's telemetry posting.
     * You could manually send trajectory states to NetworkTables/Shuffleboard if desired.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
         System.out.println("INFO: postTrajectory is a placeholder in MaxSwerveDriveSubsystem.");
         // Example: Send to Shuffleboard
         // Field2d field = SmartDashboard.getField("Field");
         // field.getObject("traj").setTrajectory(trajectory);
    }


    /**
     * Lock the swerve drive in an X-pattern to resist movement.
     */
    public void lock() {
        setX();
    }


    /**
     * Checks if a PathPlanner path command (started by getAutonomousCommand or driveToPose)
     * is currently scheduled and running.
     *
     * @return True if a path command is active, false otherwise.
     */
    public boolean isPathCommandRunning() {
        return currentPathCommand != null && currentPathCommand.isScheduled();
    }


    // --- Methods from YAGSL not directly applicable ---

    /**
     * Get the SwerveController (YAGSL specific object). Not available in MAXSwerve.
     * @return null
     */
    public Object getSwerveController() { // Return type Object to avoid compile error if SwerveController not imported
        System.err.println("WARNING: getSwerveController() is not applicable to MaxSwerveDriveSubsystem. Returning null.");
        return null;
    }

    /**
     * Get the SwerveDriveConfiguration (YAGSL specific object). Not available in MAXSwerve.
     * @return null
     */
    public Object getSwerveDriveConfiguration() { // Return type Object to avoid compile error
        System.err.println("WARNING: getSwerveDriveConfiguration() is not applicable to MaxSwerveDriveSubsystem. Returning null.");
        return null;
    }

    /**
     * Get the SwerveDrive object (YAGSL specific). Not available in MAXSwerve.
     * @return null
     */
     public Object getSwerveDrive() { // Return type Object to avoid compile error
         System.err.println("WARNING: getSwerveDrive() is not applicable to MaxSwerveDriveSubsystem. Returning null.");
         return null;
     }

     // NOTE: driveWithSwerveMath methods from the original code were complex and seemed
     // to replicate kinematic calculations. Standard drive methods using ChassisSpeeds
     // are generally preferred. If that specific algorithm is needed, it could be
     // reimplemented carefully, ensuring constants like chassis dimensions are correct.
}