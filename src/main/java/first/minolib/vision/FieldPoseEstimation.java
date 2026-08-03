package first.minolib.vision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

/** Represents a robot pose estimate from vision with associated uncertainty and metadata. */
public class FieldPoseEstimation {

    private final Pose2d visionRobotPoseMeters;
    private final double timestampSeconds;
    private final Matrix<N3, N1> visionMeasurementStdDevs;

    /**
     * Creates a new vision field pose estimate.
     *
     * @param visionRobotPoseMeters The estimated robot pose on the field in meters
     * @param timestampSeconds The timestamp when this estimate was captured
     * @param visionMeasurementStdDevs Standard deviations representing measurement uncertainty
     */
    
    public FieldPoseEstimation(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        this.visionRobotPoseMeters = visionRobotPoseMeters;
        this.timestampSeconds = timestampSeconds;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }

    public Pose2d getVisionRobotPoseMeters() {
        return visionRobotPoseMeters;
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
        return visionMeasurementStdDevs;
    }
}