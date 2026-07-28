package frc.minolib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtility {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-9);
    }

    /** Extension methods for wpi geometry objects */
    public static class GeomExtensions {
        public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
            return EqualsUtility.epsilonEquals(twist.dx, other.dx)
                && EqualsUtility.epsilonEquals(twist.dy, other.dy)
                && EqualsUtility.epsilonEquals(twist.dtheta, other.dtheta);
        }

        public static boolean epsilonEquals(Pose2d pose, Pose2d other) {
            return EqualsUtility.epsilonEquals(pose.getTranslation().getX(), other.getTranslation().getX())
                && EqualsUtility.epsilonEquals(pose.getTranslation().getY(), other.getTranslation().getY())
                && EqualsUtility.epsilonEquals(pose.getRotation().getRadians(), other.getRotation().getRadians());
        }
    }
}