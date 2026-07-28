package frc.minolib.wpilib;

import edu.wpi.first.math.geometry.Pose3d;

public class Fiducial {
    private final Type type;
    // An ID of -1 indicates this is an unlabeled fiducial (e.g. retroreflective tape)
    private final int id;
    private final Pose3d pose;
    private final double size;
  
    public static enum Type {
        RETROREFLECTIVE(0),
        APRILTAG(1);
    
        private final int value;
    
        Type(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }
  
    public Fiducial(final Type type, final int id, final Pose3d pose, final double size) {
        this.type = type;
        this.id = id;
        this.pose = pose;
        this.size = size;
    }
  
    public Type getType() {
        return type;
    }
  
    public int id() {
        return id;
    }
  
    public Pose3d getPose() {
        return pose;
    }
  
    public double getX() {
        return pose.getX();
    }
  
    public double getY() {
        return pose.getY();
    }
  
    public double getZ() {
        return pose.getZ();
    }
  
    public double getXRot() {
        return pose.getRotation().getX();
    }
  
    public double getYRot() {
        return pose.getRotation().getY();
    }
  
    public double getZRot() {
        return pose.getRotation().getZ();
    }
  
    public double getSize() {
        return size;
    }
  
    // Struct for serialization.
    public static final FiducialStruct struct = new FiducialStruct();
}