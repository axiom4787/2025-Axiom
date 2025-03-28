package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.DriveSubsystem;
import swervelib.SwerveModule;

/**
 * Utility for logging swerve module states to AdvantageScope with proper
 * optimization math.
 */
public class SwerveModuleStateLogger {

    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("CorrectModuleStates");

    /**
     * Log the current, desired, and optimized module states to AdvantageScope.
     * 
     * @param driveSubsystem The drive subsystem containing the swerve modules
     * @param desiredStates  The desired states for each module
     */
    public static void logStates(DriveSubsystem driveSubsystem, SwerveModuleState[] desiredStates) {
        SwerveModule[] modules = driveSubsystem.getSwerveDrive().getModules();

        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];

            // Get current state (what the module is actually doing)
            SwerveModuleState currentState = new SwerveModuleState(
                    module.getState().speedMetersPerSecond,
                    module.getState().angle);

            // Get desired state from parameter (what we want the module to do)
            SwerveModuleState desiredState = desiredStates[i];

            // Calculate optimized state using our custom optimization
            SwerveModuleState optimizedState = optimizeState(desiredState, currentState.angle);

            // Log all states to AdvantageScope
            logModuleState(i, currentState, desiredState, optimizedState);
        }
    }

    /**
     * Logs a single module's states to NetworkTables for AdvantageScope.
     */
    private static void logModuleState(int moduleIndex,
            SwerveModuleState currentState,
            SwerveModuleState desiredState,
            SwerveModuleState optimizedState) {
        String prefix = "Module" + moduleIndex + "/";

        // Log current state
        table.getEntry(prefix + "Current/Speed").setDouble(currentState.speedMetersPerSecond);
        table.getEntry(prefix + "Current/Angle").setDouble(currentState.angle.getDegrees());

        // Log desired state
        table.getEntry(prefix + "Desired/Speed").setDouble(desiredState.speedMetersPerSecond);
        table.getEntry(prefix + "Desired/Angle").setDouble(desiredState.angle.getDegrees());

        // Log optimized state
        table.getEntry(prefix + "Optimized/Speed").setDouble(optimizedState.speedMetersPerSecond);
        table.getEntry(prefix + "Optimized/Angle").setDouble(optimizedState.angle.getDegrees());
    }

    /**
     * Optimizes a swerve module state to minimize rotation while maintaining
     * desired direction of travel.
     * This is the correct swerve math that minimizes module rotation by flipping
     * direction when appropriate.
     * 
     * @param desiredState The desired module state
     * @param currentAngle The current angle of the module
     * @return An optimized state that minimizes the rotation of the module
     */
    public static SwerveModuleState optimizeState(SwerveModuleState desiredState, Rotation2d currentAngle) {
        // Get the desired angle in degrees within the correct scope
        double targetAngle = adjustAngle(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;

        // Calculate the difference between desired and current angles
        double angleDifference = targetAngle - currentAngle.getDegrees();

        // If the angle difference requires more than 90 degrees of rotation,
        // we can flip the direction and change the angle to reduce turning
        if (Math.abs(angleDifference) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle += (angleDifference > 90) ? -180 : 180;
        }

        // Return the optimized state
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * Adjusts an angle to be within the appropriate scope for optimization.
     * 
     * @param currentAngle Current angle in degrees
     * @param targetAngle  Target angle in degrees
     * @return Adjusted target angle in the appropriate scope
     */
    private static double adjustAngle(double currentAngle, double targetAngle) {
        // Calculate the upper and lower bounds of the scope based on current angle
        double lowerBound;
        double upperBound;
        double lowerOffset = currentAngle % 360;

        if (lowerOffset >= 0) {
            lowerBound = currentAngle - lowerOffset;
            upperBound = currentAngle + (360 - lowerOffset);
        } else {
            upperBound = currentAngle - lowerOffset;
            lowerBound = currentAngle - (360 + lowerOffset);
        }

        // Adjust the target angle to be within the scope
        while (targetAngle < lowerBound) {
            targetAngle += 360;
        }
        while (targetAngle > upperBound) {
            targetAngle -= 360;
        }

        return targetAngle;
    }
}
