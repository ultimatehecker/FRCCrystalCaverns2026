package first.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

public class FieldConstants {
    public static final double kFieldLength = Units.inchesToMeters(648);
    public static final double kFieldWidth = Units.inchesToMeters(324);

    public static final Pose2d kCenter = new Pose2d(kFieldLength / 2.0, FieldConstants.kFieldWidth / 2.0, Rotation2d.kZero);
    public static final Translation2d kCenterOrigin = kCenter.getTranslation();

    public static final Pose2d kFirstRedCrystalStartingPose = kCenter.transformBy(new Transform2d(6.669830, -2.771845, Rotation2d.kZero));
    public static final Pose2d kFirstBlueCrystalStartingPose = kCenter.transformBy(new Transform2d(-6.669830, -2.771845, Rotation2d.kZero));

    public static final Pose2d kFirstCaveCrystalStartingPose = kCenter.transformBy(new Transform2d(0, -0.762000, Rotation2d.kZero));
    public static final Pose2d kFirstHumanPlayerCrystalStartingPose = kCenter.transformBy(new Transform2d(0, 0.762000, Rotation2d.kZero));

    public static final double kFarCrystalStagingSeparation = 1.604022; // m
    public static final double kCenterCrystalStagingSeparation = 0.889000; // m

    public static final Pose2d[] kCrystalStartingPositions = new Pose2d[16];
    static {
        for (int i = 0; i < 4; i++) {
            kCrystalStartingPositions[i] = kFirstRedCrystalStartingPose.transformBy(new Transform2d(0, kFarCrystalStagingSeparation * i, Rotation2d.kZero));
        }
        for (int i = 0; i < 4; i++) {
            kCrystalStartingPositions[i + 4] = kFirstBlueCrystalStartingPose.transformBy(new Transform2d(0, kFarCrystalStagingSeparation * i, Rotation2d.kZero));
        }
        for (int i = 0; i < 4; i++) {
            kCrystalStartingPositions[i + 8] = kFirstCaveCrystalStartingPose.transformBy(new Transform2d(0, kCenterCrystalStagingSeparation * -i, Rotation2d.kZero));
        }
        for (int i = 0; i < 4; i++) {
            kCrystalStartingPositions[i + 12] = kFirstHumanPlayerCrystalStartingPose.transformBy(new Transform2d(0, kCenterCrystalStagingSeparation * i, Rotation2d.kZero));
        }
    }

    public static final Pose2d kBlueCaveCenter = kCenter.transformBy(new Transform2d(-4.129830, 0, Rotation2d.kZero));
    public static final Translation2d kBlueCaveOrigin = kBlueCaveCenter.getTranslation();

    public static final Translation2d[] kBlueCaveVertices = new Translation2d[8];
    static {
        for (int i = 0; i < kBlueCaveVertices.length; i++) {
            kBlueCaveVertices[i] = new Translation2d(1.170789, Rotation2d.fromDegrees(45 * i)).plus(kBlueCaveOrigin);
        }
    }

    public static final Translation2d[] kBlueLowerBranches = new Translation2d[] {
            new Translation2d(0.762663, 0.315905).plus(kBlueCaveOrigin),
            new Translation2d(0.315905, 0.762663).plus(kBlueCaveOrigin),
            new Translation2d(-0.315905, 0.762663).plus(kBlueCaveOrigin), // classifier side
            new Translation2d(-0.762663, 0.315905).plus(kBlueCaveOrigin),
            new Translation2d(-0.762663, -0.315905).plus(kBlueCaveOrigin), // HP side
            new Translation2d(-0.315905, -0.762663).plus(kBlueCaveOrigin),
            new Translation2d(0.315905, -0.762663).plus(kBlueCaveOrigin), // scoring table side
            new Translation2d(0.762663, -0.315905).plus(kBlueCaveOrigin)
    };

    public static final Translation2d[] kBlueUpperBranches = new Translation2d[] {
            new Translation2d(0.218704, 0.218704).plus(kBlueCaveOrigin),
            new Translation2d(-0.218704, 0.218704).plus(kBlueCaveOrigin), // classifier side
            new Translation2d(-0.218704, -0.218704).plus(kBlueCaveOrigin), // HP side
            new Translation2d(0.218704, -0.218704).plus(kBlueCaveOrigin) // scoring table side
    };

    public static final Translation2d[] kRedCaveVertices = Arrays.stream(kBlueCaveVertices)
            .map(pointAtBlue -> new Translation2d(kFieldLength - pointAtBlue.getX(), pointAtBlue.getY()))
            .toArray(Translation2d[]::new);

    public static final Translation2d[] kRedLowerBranches = Arrays.stream(kBlueLowerBranches)
            .map(pointAtBlue -> new Translation2d(kFieldLength - pointAtBlue.getX(), pointAtBlue.getY()))
            .toArray(Translation2d[]::new);

    public static final Translation2d[] kRedUpperBranches = Arrays.stream(kBlueUpperBranches)
            .map(pointAtBlue -> new Translation2d(kFieldLength - pointAtBlue.getX(), pointAtBlue.getY()))
            .toArray(Translation2d[]::new);

    public static final double kLowerBranchHeight = Units.inchesToMeters(19);
    public static final double kUpperBranchHeight = Units.inchesToMeters(32);

    public static final Pose3d kBlueGemstoneStartingPose = new Pose3d(kBlueCaveCenter.getX(), kBlueCaveCenter.getY(), Units.inchesToMeters(32.813), Rotation3d.kZero);
    public static final Pose3d kRedGemstoneStartingPose = new Pose3d(kFieldLength - kBlueCaveCenter.getX(), kBlueCaveCenter.getY(), Units.inchesToMeters(32.813), Rotation3d.kZero);
    
    public static final Translation2d[] kJewelryVertices = new Translation2d[6];
    static {
        for (int i = 0; i < kJewelryVertices.length; i++) {
            kJewelryVertices[i] = new Translation2d(0.293294, Rotation2d.fromDegrees(60 * i)).plus(kCenterOrigin);
        }
    }

    public static final double kJewelryPositionTolerance = Units.inchesToMeters(12);
    public static final double kJewelryRotationTolerance = Units.degreesToRadians(30);
    public static final double kJewelryMinimumZ = Units.inchesToMeters(12);

    public static final Translation2d kRedMineCornerA = new Translation2d(kFieldLength / 2.0 - 6.493848, 0);
    public static final Translation2d kBlueMineCornerA = new Translation2d(kFieldLength / 2.0 + 6.493848, 0);
    public static final Translation2d kRedMineCornerB = new Translation2d(0, kFieldWidth / 2.0 - 3.044640);
    public static final Translation2d kBlueMineCornerB = new Translation2d(kFieldLength, kFieldWidth / 2.0 - 3.044640);

    public static final List<Pose2d> kMines = new ArrayList<Pose2d>();
    static {
        kMines.add(new Pose2d(
            (kBlueMineCornerA.getX() + kBlueMineCornerB.getX()) / 2.0,
            (kBlueMineCornerA.getY() + kBlueMineCornerB.getY()) / 2.0,
            Rotation2d.fromDegrees(120)
        ));

        kMines.add(new Pose2d(
            (kRedMineCornerA.getX() + kRedMineCornerB.getX()) / 2.0,
            (kRedMineCornerA.getY() + kRedMineCornerB.getY()) / 2.0,
            Rotation2d.fromDegrees(60)
        ));
    }

    public static final Translation2d[] kBlueCaveExitTrussOneCorners = new Translation2d[] {
            new Translation2d(-3.169557, 2.693090).plus(kCenterOrigin),
            new Translation2d(-3.483882, 2.693090).plus(kCenterOrigin),
            new Translation2d(-3.483882, 2.997890).plus(kCenterOrigin),
            new Translation2d(-3.169557, 2.997890).plus(kCenterOrigin),
    };
    
    public static final Translation2d[] kBlueCaveExitTrussTwoCorners = new Translation2d[] {
            new Translation2d(-6.531882, 2.693090).plus(kCenterOrigin),
            new Translation2d(-6.846207, 2.693090).plus(kCenterOrigin),
            new Translation2d(-6.846207, 2.997890).plus(kCenterOrigin),
            new Translation2d(-6.531882, 2.997890).plus(kCenterOrigin),
    };

    public static final Translation2d[] kRedCaveExitTrussOneCorners = Arrays.stream(kBlueCaveExitTrussOneCorners).map(pointAtBlue -> new Translation2d(kFieldLength - pointAtBlue.getX(), pointAtBlue.getY())).toArray(Translation2d[]::new);
    public static final Translation2d[] kRedCaveExitTrussTwoCorners = Arrays.stream(kBlueCaveExitTrussTwoCorners).map(pointAtBlue -> new Translation2d(kFieldLength - pointAtBlue.getX(), pointAtBlue.getY())).toArray(Translation2d[]::new);
}