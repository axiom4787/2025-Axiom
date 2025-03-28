package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Utility class for properly optimizing swerve module states
 * to minimize module rotation through proper 90-degree optimization.
 */
public class SwerveOptimizedModuleState {

    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("ModuleOptimization");

    /**
     * Optimizes a single swerve module state to minimize rotation.
     * Uses proper 90-degree optimization: if turning more than 90 degrees,
     * flips direction of travel and rotates to the equivalent angle.
     *
     * @param desiredState The desired module state
     * @param currentAngle The current angle of the module
     * @return An optimized state that minimizes the rotation of the module
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = normalizeAngle(desiredState.angle.getDegrees(), currentAngle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;

        // Calculate the smallest rotational difference (accounting for wraparound)
        double angleDifference = normalizeAngleDifference(targetAngle - currentAngle.getDegrees());

        // If the module needs to rotate more than 90 degrees (or less than -90
        // degrees),
        // we can flip the direction and use the opposite angle
        if (Math.abs(angleDifference) > 90.0) {
            targetSpeed = -targetSpeed;
            // Add or subtract 180 degrees as needed to keep within proper range
            targetAngle = normalizeAngle(targetAngle + (angleDifference > 0 ? -180.0 : 180.0),
                    currentAngle.getDegrees());

            // Recalculate the angle difference for telemetry
            angleDifference = normalizeAngleDifference(targetAngle - currentAngle.getDegrees());
        }

        // Log optimization data
        String prefix = "Optimization/";
        table.getEntry(prefix + "OriginalAngle").setDouble(desiredState.angle.getDegrees());
        table.getEntry(prefix + "CurrentAngle").setDouble(currentAngle.getDegrees());
        table.getEntry(prefix + "OriginalSpeed").setDouble(desiredState.speedMetersPerSecond);
        table.getEntry(prefix + "OptimizedAngle").setDouble(targetAngle);
        table.getEntry(prefix + "OptimizedSpeed").setDouble(targetSpeed);
        table.getEntry(prefix + "AngleDifference").setDouble(angleDifference);
        table.getEntry(prefix + "Flipped").setBoolean(targetSpeed != desiredState.speedMetersPerSecond);

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * Normalize an angle to be within a range centered around a reference angle.
     * This avoids unnecessary full rotations by keeping angles close to the current
     * angle.
     * 
     * @param angleRadians   Angle to normalize in degrees
     * @param referenceAngle Reference angle in degrees
     * @return Normalized angle in degrees
     */
    private static double normalizeAngle(double angle, double referenceAngle) {
        // First, limit the reference angle to [0, 360)
        referenceAngle = referenceAngle % 360;
        if (referenceAngle < 0)
            referenceAngle += 360;

        // Then find the closest equivalent angle to the reference
        double normalizedAngle = angle % 360;
        if (normalizedAngle < 0)
            normalizedAngle += 360;

        // Calculate the distance if we add or subtract 360
        double distanceWithoutChange = Math.abs(normalizedAngle - referenceAngle);
        double distanceWithAdd = Math.abs(normalizedAngle + 360 - referenceAngle);
        double distanceWithSubtract = Math.abs(normalizedAngle - 360 - referenceAngle);

        // Return the angle that's closest to the reference
        if (distanceWithoutChange <= distanceWithAdd && distanceWithoutChange <= distanceWithSubtract) {
            return normalizedAngle;
        } else if (distanceWithAdd <= distanceWithSubtract) {
            return normalizedAngle + 360;
        } else {
            return normalizedAngle - 360;
        }
    }

    /**
     * Normalizes an angle difference to be within [-180, 180) degrees.
     * 
     * @param angleDifference The angle difference to normalize
     * @return The normalized angle difference
     */
    private static double normalizeAngleDifference(double angleDifference) {
        angleDifference = angleDifference % 360;
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }
        return angleDifference;
    }

    /**
     * Optimizes all swerve module states in an array.
     * 
     * @param desiredStates The desired module states
     * @param currentAngles The current angles of the modules
     * @return An array of optimized states
     */
    public static SwerveModuleState[] optimizeAll(SwerveModuleState[] desiredStates, Rotation2d[] currentAngles) {
        SwerveModuleState[] optimizedStates = new SwerveModuleState[desiredStates.length];

        for (int i = 0; i < desiredStates.length; i++) {
            optimizedStates[i] = optimize(desiredStates[i], currentAngles[i]);
        }

        return optimizedStates;
    }
}
