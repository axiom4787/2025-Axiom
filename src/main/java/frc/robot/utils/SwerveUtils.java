package frc.robot.utils; // Or your appropriate package

import edu.wpi.first.math.MathUtil;

public class SwerveUtils {

    /**
     * Wraps an angle in radians to the range [-pi, pi].
     *
     * @param angleRadians Angle in radians.
     * @return Angle in radians wrapped to [-pi, pi].
     */
    public static double wrapAngle(double angleRadians) {
        return MathUtil.angleModulus(angleRadians);
    }

    /**
     * Finds the shortest difference between two angles in radians.
     *
     * @param angleA First angle in radians.
     * @param angleB Second angle in radians.
     * @return Shortest difference in radians, in the range [-pi, pi].
     */
    public static double angleDifference(double angleARadians, double angleBRadians) {
        return wrapAngle(angleARadians - angleBRadians);
    }


    /**
     * Steps the current angle towards the target angle, taking the shortest path and respecting the maximum step size.
     * Handles wrapping around +/- pi.
     *
     * @param currentAngleRad Current angle in radians.
     * @param targetAngleRad Target angle in radians.
     * @param maxStepRad Maximum change in angle allowed in radians (should be positive).
     * @return The new angle in radians after taking the step.
     */
     public static double stepTowardsCircular(double currentAngleRad, double targetAngleRad, double maxStepRad) {
        double difference = angleDifference(targetAngleRad, currentAngleRad);
        double step = Math.copySign(Math.min(Math.abs(difference), Math.abs(maxStepRad)), difference);
        return wrapAngle(currentAngleRad + step);
     }
}