package frc.minolib.math;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** A utility class that provides various mathematical functions and operations. */
public class MathUtility {

    /** A small constant used for floating-point comparisons. */
    private static final double kEps = 1E-9;

    /**
     * Converts Cartesian coordinates to polar coordinates.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @return a Pair containing the radius and angle in radians
     */
    public static Pair<Double, Double> cart2pol(final double x, final double y) {
        final double r = Math.sqrt(x * x + y * y);
        final double theta = Math.atan2(y, x);
        return new Pair<>(r, theta);
    }

    /**
     * Converts polar coordinates to Cartesian coordinates.
     *
     * @param r the radius
     * @param theta the angle in radians
     * @return a Pair containing the x and y coordinates
     */
    public static Pair<Double, Double> pol2cart(final double r, final double theta) {
        final double x = r * Math.cos(theta);
        final double y = r * Math.sin(theta);
        return new Pair<>(x, y);
    }

    /**
     * Constrains an angle to be within the range [-pi, pi).
     *
     * @param angle the angle to constrain
     * @return the constrained angle
     */
    public static double constrainAngleNegPiToPi(final double angle) {
        double x = (angle + Math.PI) % (2.0 * Math.PI);
        if (x < 0.0) {
            x += 2.0 * Math.PI;
        }

        return x - Math.PI;
    }

    /**
     * Returns the angle placed within [-pi, pi) of the reference angle.
     *
     * @param angle the angle to place in scope
     * @param referenceAngle the reference angle
     * @return the angle placed within the scope of the reference angle
     */
    public static double placeInScope(final double angle, final double referenceAngle) {
        return referenceAngle + constrainAngleNegPiToPi(angle - referenceAngle);
    }

    /**
     * Clamps a value between a minimum and maximum value.
     *
     * @param val the value to clamp
     * @param min the minimum value
     * @param max the maximum value
     * @return the clamped value
     */
    public static double clamp(final double val, final double min, final double max) {
        return Math.max(Math.min(val, max), min);
    }

    /**
     * Obtains a new Pose2d from a constant curvature velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp. Borrowed from 254:
     * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/geometry/Pose2d.java
     *
     * @param delta the Twist2d representing the change in pose
     * @return the new Pose2d
     */
    public static Pose2d exp(final Twist2d delta) {
        final double sin_theta = Math.sin(delta.dtheta);
        final double cos_theta = Math.cos(delta.dtheta);
        final double s = Math.abs(delta.dtheta) < kEps ? 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta : sin_theta / delta.dtheta;
        final double c = Math.abs(delta.dtheta) < kEps ? 0.5 * delta.dtheta : (1.0 - cos_theta) / delta.dtheta;

        return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s), new Rotation2d(cos_theta, sin_theta));
    }

    class Camera {
        
        /**
         * Converts cartesian coordinates in camera frame to bearing/elevation that would be returned by a
         * pinhole camera model. X-forward, Y-left, Z-up.
         */
        public static Pair<Double, Double> cart2BEPinhole(final Matrix<N3, N1> cartesian) {
            final double x = cartesian.get(0, 0);
            final double y = cartesian.get(1, 0);
            final double z = cartesian.get(2, 0);

            final double bearing = Math.atan2(y, x);
            final double elevation = Math.atan2(z, x);

            // Negate bearing because camera returns +bearing to the right.
            return new Pair<>(-bearing, elevation);
        }

        /**
         * Returns the camera-frame vector given the bearing and elevation in a pinhole model. X-forward,
         * Y-left, Z-up.
         */
        public static Matrix<N3, N1> pinholeBE2Cart(final double bearing, final double elevation) {
            // Negate bearing because camera +bearing is to the right.
            final double x = 1.0;
            final double y = x * Math.tan(-bearing);
            final double z = x * Math.tan(elevation);
            return MatBuilder.fill(Nat.N3(), Nat.N1(), x, y, z);
        }

        /**
         * Converts cartesian coordinates in camera frame to bearing/elevation in a spherical coordinate
         * system. X-forward, Y-left, Z-up.
         */
        public static Pair<Double, Double> cart2BESph(final Matrix<N3, N1> cartesian) {
            final double x = cartesian.get(0, 0);
            final double y = cartesian.get(1, 0);
            final double z = cartesian.get(2, 0);

            final double bearing = Math.atan2(y, x);
            final double elevation = Math.atan2(z, Math.sqrt(x * x + y * y));

            return new Pair<>(bearing, elevation);
        }

        /**
         * Converts bearing/elevation in a spherical coordinate system to cartesian coordinates.
         * X-forward, Y-left, Z-up.
         */
        public static Matrix<N3, N1> beSph2Cart(final double bearing, final double elevation) {
            final double x = Math.cos(bearing) * Math.cos(elevation);
            final double y = Math.sin(bearing) * Math.cos(elevation);
            final double z = Math.sin(elevation);
            return MatBuilder.fill(Nat.N3(), Nat.N1(), x, y, z);
        }

        /**
         * Converts X/Y pixels to yaw/pitch radians. +X right, +Y down. Estimtes camera center and focal
         * length from image size and FOV.
         */
        public static Pair<Double, Double> XYToYawPitchWithHeightAndFOV(final double x, final double y, final double imageWidth, final double imageHeight, final double fovWidth, final double fovHeight) {
            final double centerX = (imageWidth / 2.0) - 0.5;
            final double centerY = (imageHeight / 2.0) - 0.5;
            final double horizontalFocalLength = imageWidth / (2.0 * Math.tan(fovWidth / 2.0));
            final double verticalFocalLength = imageHeight / (2.0 * Math.tan(fovHeight / 2.0));
            return XYToYawPitch(x, y, centerX, centerY, horizontalFocalLength, verticalFocalLength);
        }

        /** Converts X/Y pixels to yaw/pitch radians. +X right, +Y down. */
        public static Pair<Double, Double> XYToYawPitch(final double x, final double y, final double centerX, final double centerY, final double horizontalFocalLength, final double verticalFocalLength) {
            return new Pair<>(Math.atan((x - centerX) / horizontalFocalLength), -Math.atan((y - centerY) / verticalFocalLength));
        }

        /** Converts yaw/pitch radians to X/Y pixels. +X right, +Y down. */
        public static Pair<Double, Double> yawPitchToXY(final double yaw, final double pitch, final double imageWidth, final double imageHeight, final double fovWidth, final double fovHeight) {
            final double centerX = (imageWidth / 2.0) - 0.5;
            final double centerY = (imageHeight / 2.0) - 0.5;
            final double horizontalFocalLength = imageWidth / (2.0 * Math.tan(fovWidth / 2.0));
            final double verticalFocalLength = imageHeight / (2.0 * Math.tan(fovHeight / 2.0));

            return new Pair<>((Math.tan(yaw) * horizontalFocalLength) + centerX, (Math.tan(-pitch) * verticalFocalLength) + centerY);
        }
    }
}