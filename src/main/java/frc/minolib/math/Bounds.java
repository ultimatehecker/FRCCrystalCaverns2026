package frc.minolib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public record Bounds(double minX, double maxX, double minY, double maxY) {
  public boolean contains(Translation2d translation) {
    return translation.getX() >= minX()
        && translation.getX() <= maxX()
        && translation.getY() >= minY()
        && translation.getY() <= maxY();
    }

    public Translation2d clamp(Translation2d translation) {
        return new Translation2d(MathUtil.clamp(translation.getX(), minX(), maxX()), MathUtil.clamp(translation.getY(), minY(), maxY()));
    }
}