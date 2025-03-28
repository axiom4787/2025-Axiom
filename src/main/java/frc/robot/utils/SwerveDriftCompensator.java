package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Utility class to compensate for drift in swerve drive systems.
 */
public class SwerveDriftCompensator {
    // PID controllers for position correction
    private final PIDController xController = new PIDController(0.5, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.5, 0.0, 0.0);
    private final PIDController rotController = new PIDController(0.7, 0.0, 0.0);

    // Drift correction factor (applied to velocity)
    private static final double DRIFT_CORRECTION_FACTOR = 0.05;

    // Track previous pose for velocity-based corrections
    private Pose2d previousPose = new Pose2d();
    private double previousTime = Timer.getFPGATimestamp();

    // For telemetry
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("DriftCompensation");

    // Store the original commanded speed for comparison
    private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

    // For static drift compensation
    private double xDriftBias = 0.0;
    private double yDriftBias = 0.0;
    private double rotDriftBias = 0.0;

    // Configurable deadband for when to apply corrections
    private static final double VELOCITY_DEADBAND = 0.02;

    /**
     * Constructor for SwerveDriftCompensator.
     * Initializes PID controllers with continuous input for rotation.
     */
    public SwerveDriftCompensator() {
        // Configure rotation controller for continuous input (wrapping at +/- PI)
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        // Set output range for PID controllers
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        rotController.setTolerance(0.01);
    }

    /**
     * Apply drift compensation to chassis speeds based on expected vs actual
     * motion.
     * 
     * @param requestedSpeeds The speeds requested by the driver or autonomous
     *                        system
     * @param currentPose     The current measured pose of the robot
     * @return Compensated ChassisSpeeds to reduce drift
     */
    public ChassisSpeeds compensateForDrift(ChassisSpeeds requestedSpeeds, Pose2d currentPose) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - previousTime;

        // Skip processing if time delta is too small
        if (dt < 0.005) {
            return requestedSpeeds;
        }

        // Store the requested speeds for telemetry
        lastCommandedSpeeds = requestedSpeeds;

        // Calculate expected pose change based on previous commanded speeds
        Translation2d expectedDelta = new Translation2d(
                lastCommandedSpeeds.vxMetersPerSecond * dt,
                lastCommandedSpeeds.vyMetersPerSecond * dt);

        // Calculate actual pose change
        Translation2d actualDelta = currentPose.getTranslation().minus(previousPose.getTranslation());

        // Only apply drift compensation if we're actually trying to move
        boolean isMoving = Math.abs(requestedSpeeds.vxMetersPerSecond) > VELOCITY_DEADBAND ||
                Math.abs(requestedSpeeds.vyMetersPerSecond) > VELOCITY_DEADBAND ||
                Math.abs(requestedSpeeds.omegaRadiansPerSecond) > VELOCITY_DEADBAND;

        // Calculate compensation
        ChassisSpeeds compensatedSpeeds = new ChassisSpeeds(
                requestedSpeeds.vxMetersPerSecond,
                requestedSpeeds.vyMetersPerSecond,
                requestedSpeeds.omegaRadiansPerSecond);

        if (isMoving) {
            // Calculate the drift error
            double xError = (expectedDelta.getX() - actualDelta.getX()) / dt;
            double yError = (expectedDelta.getY() - actualDelta.getY()) / dt;

            // Update drift bias with a moving average
            xDriftBias = 0.95 * xDriftBias + 0.05 * xError;
            yDriftBias = 0.95 * yDriftBias + 0.05 * yError;

            // Apply compensations - add biases to counter systematic drift
            compensatedSpeeds.vxMetersPerSecond += xDriftBias * DRIFT_CORRECTION_FACTOR;
            compensatedSpeeds.vyMetersPerSecond += yDriftBias * DRIFT_CORRECTION_FACTOR;
        }

        // Log data
        logDriftData(requestedSpeeds, compensatedSpeeds, currentPose, expectedDelta, actualDelta);

        // Update previous values for next calculation
        previousPose = currentPose;
        previousTime = currentTime;

        return compensatedSpeeds;
    }

    /**
     * Log drift compensation data to NetworkTables for analysis.
     */
    private void logDriftData(ChassisSpeeds requested, ChassisSpeeds compensated,
            Pose2d currentPose, Translation2d expectedDelta, Translation2d actualDelta) {
        table.getEntry("Requested/vx").setDouble(requested.vxMetersPerSecond);
        table.getEntry("Requested/vy").setDouble(requested.vyMetersPerSecond);
        table.getEntry("Requested/omega").setDouble(requested.omegaRadiansPerSecond);

        table.getEntry("Compensated/vx").setDouble(compensated.vxMetersPerSecond);
        table.getEntry("Compensated/vy").setDouble(compensated.vyMetersPerSecond);
        table.getEntry("Compensated/omega").setDouble(compensated.omegaRadiansPerSecond);

        table.getEntry("Bias/x").setDouble(xDriftBias);
        table.getEntry("Bias/y").setDouble(yDriftBias);
        table.getEntry("Bias/rot").setDouble(rotDriftBias);

        table.getEntry("Delta/expectedX").setDouble(expectedDelta.getX());
        table.getEntry("Delta/expectedY").setDouble(expectedDelta.getY());
        table.getEntry("Delta/actualX").setDouble(actualDelta.getX());
        table.getEntry("Delta/actualY").setDouble(actualDelta.getY());
    }

    /**
     * Reset the drift compensator.
     * 
     * @param pose Current robot pose to reset from
     */
    public void reset(Pose2d pose) {
        previousPose = pose;
        previousTime = Timer.getFPGATimestamp();
        xDriftBias = 0.0;
        yDriftBias = 0.0;
        rotDriftBias = 0.0;
    }
}
