package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Drivetrain
 */
public class Drivetrain {

    private SwerveModule[] _modules;

    private TalonSRX _pigeonTalon;
    private PigeonIMU _pigeon;

    private SwerveDriveOdometry _odometry;

    public Drivetrain() {
        _modules = new SwerveModule[] {
            new SwerveModule(Constants.Drivetrain.TRModule),
            new SwerveModule(Constants.Drivetrain.TLModule),
            new SwerveModule(Constants.Drivetrain.BRModule),
            new SwerveModule(Constants.Drivetrain.BLModule)
        };

        _pigeonTalon = new TalonSRX(Constants.Drivetrain.pigeonTalonId);
        _pigeon = new PigeonIMU(_pigeonTalon);

        _odometry = new SwerveDriveOdometry(Constants.Drivetrain.kinematics, getRotation2d());
    }

    double last = 0;
    double mid = 0;

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;
        rot *= 2;

        SwerveModuleState[] moduleStates = Constants.Drivetrain.kinematics
                .toSwerveModuleStates(fieldRelative && _pigeon.getState() == PigeonState.Ready
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setDesiredStates(moduleStates);
    }

    public void setDesiredStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond * Constants.Joysticks.speedScalar);

        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setDesiredState(moduleStates[i]);
        }

        short[] accel = new short[3];
        _pigeon.getBiasedAccelerometer(accel);
        double val = Math.sqrt(
            Math.pow((double)accel[0] / Math.pow(2, 14), 2) 
            + Math.pow((double)accel[1] / Math.pow(2, 14), 2)
            ) * 9.80663;
        SmartDashboard.putNumber("accel", val);
        // if(last < mid && val < mid && mid > 10){
            // System.out.println(mid);
        // }
        // if(val != mid) {
            last = mid;
            mid = val;
        // }
        
        // System.out.println("REAL: " + Constants.Drivetrain.kinematics.toChassisSpeeds(_modules[0].getState(), _modules[1].getState(), _modules[2].getState(),
        //     _modules[3].getState()));

        // System.out.println("WANT: " + Constants.Drivetrain.kinematics.toChassisSpeeds(moduleStates[0], moduleStates[1], moduleStates[2],
        //     moduleStates[3]));

        // System.out.println(_odometry.getPoseMeters());
        
        
    }

    public void periodic() {
        // Update the odometry in the periodic block
        _odometry.update(getRotation2d(), _modules[0].getState(), _modules[1].getState(), _modules[2].getState(),
                _modules[3].getState());
    }

    public Pose2d getPose() {
        return _odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        _odometry.resetPosition(pose, getRotation2d());
    }

    public void setDriveRPM(double voltage) {
        for (SwerveModule swerveModule : _modules) {
            swerveModule.setDriveDrive(1);
        }
    }

    private double getHeading() {
        FusionStatus status = new FusionStatus();
        _pigeon.getFusedHeading(status);
        
        return status.heading;
    }

    private Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetYaw() {
        _pigeon.setFusedHeading(0);
    }

    double max = 0;
    
    public void printSetpoints() {
        double sum = 0, avg;
        for (int i = 0; i < _modules.length; i++) {
            SmartDashboard.putNumber("steer " + i   , _modules[i].getSteeringSetpoint());
            SmartDashboard.putNumber("drive " + i, _modules[i].getDriveSetpoint());
            sum += _modules[i].getDriveRPM();
            // System.out.println(("speed " + i +" " + _modules[i].getDriveRPM()));
        }
        avg = sum / _modules.length;
        max = Math.max(avg, max);
        System.out.println("average speed " +max);
    }
}