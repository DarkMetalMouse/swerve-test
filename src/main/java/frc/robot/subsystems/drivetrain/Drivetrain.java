package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Drivetrain
 */
public class Drivetrain {

    private SwerveModule _trModule;
    private SwerveModule _tlModule;
    private SwerveModule _brModule;
    private SwerveModule _blModule;

    private TalonSRX _pigeonTalon;
    private PigeonIMU _pigeon;

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
        
        _pigeonTalon = new TalonSRX(Constants.Drivetrain.pigeonTalonId);
        _pigeon = new PigeonIMU(_pigeonTalon);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;
        rot *= 2;

        SwerveModuleState[] moduleStates =
            _kinematics.toSwerveModuleStates(
                fieldRelative && _pigeon.getState() == PigeonState.Ready
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getHeading()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        DBugSwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond * Constants.Joysticks.speedScalar);

        _trModule.setDesiredState(moduleStates[0]);
        _tlModule.setDesiredState(moduleStates[1]);
        _brModule.setDesiredState(moduleStates[2]);
        _blModule.setDesiredState(moduleStates[3]);
    }

    private double getHeading() {
        FusionStatus status = new FusionStatus();
        _pigeon.getFusedHeading(status);
        
        return status.heading;
    }

    public void resetYaw() {
        _pigeon.setFusedHeading(0);
    }
}