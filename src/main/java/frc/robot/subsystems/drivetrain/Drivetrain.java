package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Drivetrain
 */
public class Drivetrain {

    private SwerveModule _trModule;
    private SwerveModule _tlModule;
    private SwerveModule _brModule;
    private SwerveModule _blModule;

    private DBugSwerveDriveKinematics _kinematics;

    public Drivetrain() {
        _trModule = new SwerveModule(Constants.Drivetrain.TRModule);
        _tlModule = new SwerveModule(Constants.Drivetrain.TLModule);
        _brModule = new SwerveModule(Constants.Drivetrain.BRModule);
        _blModule = new SwerveModule(Constants.Drivetrain.BLModule);

        _kinematics = new DBugSwerveDriveKinematics(Constants.Drivetrain.TRModule.position, 
                                                Constants.Drivetrain.TLModule.position,
                                                Constants.Drivetrain.BRModule.position,
                                                Constants.Drivetrain.BLModule.position);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] moduleStates =
            _kinematics.toSwerveModuleStates(
                fieldRelative
                    ? new ChassisSpeeds() // todo add gyro //ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(-xSpeed, -ySpeed, -rot));
        DBugSwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond * Constants.Joysticks.speedScalar);

        _trModule.setDesiredState(moduleStates[0]);
        _tlModule.setDesiredState(moduleStates[1]);
        _brModule.setDesiredState(moduleStates[2]);
        _blModule.setDesiredState(moduleStates[3]);
    }
}