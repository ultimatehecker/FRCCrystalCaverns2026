package first.minolib.vision;

import org.wpilib.math.geometry.Pose3d;

public record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance, FidicualObservationType type) {}