package first.minolib.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class CTRESwerveDrivetrainConstants {
    SwerveDrivetrainConstants driveTrainConstants;
    SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    public CTRESwerveDrivetrainConstants(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        this.driveTrainConstants = driveTrainConstants;
        this.moduleConstants = modules;
    }

    public SwerveDrivetrainConstants getDriveTrainConstants() {
        return driveTrainConstants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return moduleConstants;
    }
}