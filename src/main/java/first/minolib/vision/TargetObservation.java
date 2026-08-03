package first.minolib.vision;

import org.wpilib.math.geometry.Rotation2d;

public record TargetObservation(Rotation2d tx, Rotation2d ty) {}
