package frc.minolib.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class CameraConfiguration {
    public enum CameraLocation {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        NONE
    }

    private String cameraName;

    public Angle cameraMountingRollRadians = Radians.of(0);
    public Angle cameraMountingYawRadians = Radians.of(0);
    public Angle cameraMountingPitchRadians = Radians.of(0);

    public Distance cameraHeightOffsetMeters = Meters.of(0);
    public Distance cameraLengthOffsetMeters = Meters.of(0);
    public Distance cameratWidthOffsetMeters = Meters.of(0);
    
    public CameraLocation cameraLocation = CameraLocation.NONE;

    public CameraConfiguration(String cameraName) {
        this.cameraName = cameraName;
    }

    public CameraConfiguration() {
        this("le");
    }

    public String getName() {
        return cameraName;
    }

    public CameraConfiguration withCameraName(String name) {
        this.cameraName = name;
        return this;
    }

    public CameraConfiguration withMountingRoll(Angle rollRadians) {
        this.cameraMountingRollRadians = rollRadians;
        return this;
    }

    public CameraConfiguration withMountingYaw(Angle yawRadians) {
        this.cameraMountingYawRadians = yawRadians;
        return this;
    }

    public CameraConfiguration withMountingPitch(Angle pitchRadians) {
        this.cameraMountingPitchRadians = pitchRadians;
        return this;
    }

    public CameraConfiguration withHeightOffset(Distance heightOffsetMeters) {
        this.cameraHeightOffsetMeters = heightOffsetMeters;
        return this;
    }

    public CameraConfiguration withLengthOffset(Distance lengthOffsetMeters) {
        this.cameraLengthOffsetMeters = lengthOffsetMeters;
        return this;
    }

    public CameraConfiguration withWidthOffset(Distance widthOffsetMeters) {
        this.cameratWidthOffsetMeters = widthOffsetMeters;
        return this;
    }

    public CameraConfiguration withCameraLocation(CameraLocation cameraLocation) {
        this.cameraLocation = cameraLocation;
        return this;
    }

    public String getCameraName() {
        return this.cameraName;
    }

    public Translation3d getTranslationOffset() {
        return new Translation3d(this.cameraLengthOffsetMeters, this.cameraHeightOffsetMeters, this.cameratWidthOffsetMeters);
    }

    public Rotation3d getRotationOffset() {
        return new Rotation3d(this.cameraMountingRollRadians, this.cameraMountingPitchRadians, this.cameraMountingYawRadians);
    }

    public Transform3d getTransformOffset() {
        return new Transform3d(
            new Translation3d(this.cameraLengthOffsetMeters.in(Meters), this.cameratWidthOffsetMeters.in(Meters), this.cameraHeightOffsetMeters.in(Meters)), 
            new Rotation3d(this.cameraMountingRollRadians.in(Radians), this.cameraMountingPitchRadians.in(Radians), this.cameraMountingYawRadians.in(Radians))
        );
    }

    public Translation2d getTranslationToRobotCenter() {
        return new Translation2d(this.cameraLengthOffsetMeters, this.cameratWidthOffsetMeters);
    }

    public CameraLocation getLocation() {
        return cameraLocation;
    }
}
