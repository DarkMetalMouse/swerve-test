package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.util.PIDFGains;


/**
 * SwerveModule
 */
public class SwerveModule {

    private CANSparkMax _driveSparkMax;
    private CANPIDController _drivePID;
    private CANEncoder _driveEncoder;
    private CANSparkMax _steeringSparkMax;
    private CANPIDController _steeringPID;
    private CANEncoder _steeringEncoder;
    private Translation2d _position;

    public SwerveModule(SwerveModuleConstants constants) {
        configSparkMax(_driveSparkMax, constants.idDrive, _drivePID, _driveEncoder, constants.driveGains);
        configSparkMax(_steeringSparkMax, constants.idSteering, _steeringPID, _steeringEncoder, constants.steeringGains);
        _position = constants.position;
    }

    private static void configSparkMax(CANSparkMax sparkMax,int id, CANPIDController pidController, CANEncoder encoder, PIDFGains gains) {
        sparkMax = new CANSparkMax(id, MotorType.kBrushless);
        pidController = sparkMax.getPIDController();
        encoder = sparkMax.getEncoder();
        sparkMax.restoreFactoryDefaults();
        sparkMax.setInverted(false);
        setPIDGains(pidController, gains);
    }

    private static void setPIDGains(CANPIDController pidController, PIDFGains gains) {
        pidController.setI(gains.getI());
        pidController.setP(gains.getP());
        pidController.setD(gains.getD());
        pidController.setFF(gains.getF());
        pidController.setIZone(gains.getIZone());
        pidController.setOutputRange(-1.0,1.0);
    }
    
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimizeAngle(desiredState, Rotation2d.fromDegrees(getAngle()));


        _steeringPID.setReference(Math.floor(getAbsSteeringPos()) + (state.angle.getDegrees() / 360), ControlType.kPosition);
        _drivePID.setReference(driveVelocityToRPM(state.speedMetersPerSecond), ControlType.kVelocity);
    }

    private static SwerveModuleState optimizeAngle(SwerveModuleState desiredState, Rotation2d currentRadian ) {
        Rotation2d angleDelta = currentRadian.minus(desiredState.angle);
        double speed = desiredState.speedMetersPerSecond;
        if(Math.abs(angleDelta.getDegrees()) > 90) {
            speed = -speed;
            angleDelta = Rotation2d.fromDegrees(180).minus(angleDelta);
        }
        return new SwerveModuleState(speed,angleDelta);


    }

    private double driveVelocityToRPM(double velocity) {
        // divide by distance per revolution, multiply by a minute to get RPM
        return velocity / (Constants.Drivetrain.SwerveModuleConstants.driveDPRMeters) * 60; 
    }

    private double getAbsSteeringPos() {
        return _steeringEncoder.getPosition() * Constants.Drivetrain.SwerveModuleConstants.steeringRatio;
    }

    private double getAngle() {
        double pos = getAbsSteeringPos();
        pos = pos - Math.floor(pos);
        return pos * 360;
    }
}