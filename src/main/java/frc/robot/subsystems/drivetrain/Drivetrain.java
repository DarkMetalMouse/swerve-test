package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Drivetrain
 */
public class Drivetrain {

    private SwerveModule[] _modules;


    private TalonSRX _pigeonTalon;
    private PigeonIMU _pigeon;

    private SwerveDriveKinematics _kinematics;

    public Drivetrain() {
        _modules = new SwerveModule[] {
            new SwerveModule(Constants.Drivetrain.TRModule),
            new SwerveModule(Constants.Drivetrain.TLModule),
            new SwerveModule(Constants.Drivetrain.BRModule),
            new SwerveModule(Constants.Drivetrain.BLModule)
        };


        _kinematics = new SwerveDriveKinematics(Constants.Drivetrain.TRModule.position, 
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
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond * Constants.Joysticks.speedScalar);


        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public void setDriveRPM(double voltage) {
        for (SwerveModule swerveModule : _modules) {
            swerveModule.setDriveRPM(voltage);
        }
    }

    private double getHeading() {
        FusionStatus status = new FusionStatus();
        _pigeon.getFusedHeading(status);
        
        return status.heading;
    }

    public void resetYaw() {
        _pigeon.setFusedHeading(0);
    }

    public void printSetpoints() {
        for (int i = 0; i < _modules.length; i++) {
            SmartDashboard.putNumber("steer " + i   , _modules[i].getSteeringSetpoint());
            SmartDashboard.putNumber("drive " + i, _modules[i].getDriveSetpoint());
            SmartDashboard.putNumber("speed " + i, _modules[i].getDriveRPM());
        }
    }
}