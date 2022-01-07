package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
    private CANCoder _driveCanCoder;

    private double _steerSetpoint;
    private double _driveSetpoint;

    public SwerveModule(SwerveModuleConstants constants) {
        _driveSparkMax = configSparkMax(constants.idDrive, _drivePID, _driveEncoder, constants.driveGains);
        _steeringSparkMax = configSparkMax(constants.idSteering, _steeringPID, _steeringEncoder, constants.steeringGains);

        _drivePID = _driveSparkMax.getPIDController();
        _driveEncoder = _driveSparkMax.getEncoder();

        _steeringPID = _steeringSparkMax.getPIDController();
        _steeringEncoder = _steeringSparkMax.getEncoder();

        setPIDGains(_drivePID, constants.driveGains);
        setPIDGains(_steeringPID, constants.steeringGains);

        _steerSetpoint = 0;

        _driveCanCoder = new CANCoder(constants.canCoderId);
        this._driveCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        this._driveCanCoder.configMagnetOffset(360 - constants.cancoderZeroPosition);
        calibrateSteering();
    }

    private static CANSparkMax configSparkMax(int id, CANPIDController pidController, CANEncoder encoder, PIDFGains gains) {
        CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
        sparkMax.setInverted(false);

        return sparkMax;
    }

    public void calibrateSteering() {
        this._steeringEncoder.setPosition(_driveCanCoder.getAbsolutePosition() / 360 / Constants.Drivetrain.SwerveModuleConstants.steeringRatio);
    }

    private static void setPIDGains(CANPIDController pidController, PIDFGains gains) {
        pidController.setI(gains.getI());
        pidController.setP(gains.getP());
        pidController.setD(gains.getD());
        pidController.setFF(gains.getF());
        pidController.setIZone(gains.getIZone());
        pidController.setOutputRange(-1.0,1.0);
    }

    public void setDriveSteering(double percent) {
        this._steeringSparkMax.set(percent);
    }
    public void setDriveDrive(double voltage) {
        this._driveSparkMax.setVoltage(voltage);
    }

    public void stop() {
        setDriveDrive(0);
        // setDriveSteering(0);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimizeAngle(desiredState, Rotation2d.fromDegrees(getAngle()));

        _steerSetpoint = addDeltaFromZeroToEncoder(state.angle.getDegrees());
        _driveSetpoint = driveVelocityToRPM(state.speedMetersPerSecond);

        if (state.speedMetersPerSecond != 0) {
            _steeringPID.setReference(_steerSetpoint, ControlType.kPosition);
        }
        // _driveSparkMax.set(1 * Math.signum(state.speedMetersPerSecond));
        if (state.speedMetersPerSecond == 0)
            _driveSparkMax.set(0);
        else
            _drivePID.setReference(_driveSetpoint, ControlType.kVelocity);
    }

    public static SwerveModuleState optimizeAngle(SwerveModuleState desiredState, Rotation2d currentRadian) {
        Rotation2d angle = desiredState.angle.minus(currentRadian);
        double speed = desiredState.speedMetersPerSecond;
        if(Math.abs(angle.getDegrees()) > 90) {
            speed = -speed;
            if (angle.getRadians() > 0) {
                angle = angle.minus(Rotation2d.fromDegrees(180));
            } else {
                angle = angle.plus(Rotation2d.fromDegrees(180));
            }
        }
        return new SwerveModuleState(speed,angle);


    }

    public double addDeltaFromZeroToEncoder(double angle) {
        double pos = getAbsSteeringPos();
        return (pos + (angle / 360)) / Constants.Drivetrain.SwerveModuleConstants.steeringRatio;
    }

    private double driveVelocityToRPM(double velocity) {
        // divide by distance per revolution, multiply by a minute to get RPM
        return velocity / (Constants.Drivetrain.SwerveModuleConstants.driveDPRMeters) * 60;
    }

    public double getAbsSteeringPos() {
        return _steeringEncoder.getPosition() * Constants.Drivetrain.SwerveModuleConstants.steeringRatio;
    }

    public double getAngle() {
        double pos = getAbsSteeringPos();
        pos = pos - Math.floor(pos);
        return pos * 360;
    }

    public double getDrivePercent() {
        return _driveSparkMax.get();
    }

    public double getSteeringSetpoint() {
        return _steerSetpoint;
    }

    public double getDriveSetpoint() {
        return _driveSetpoint;
    }

    public double getDriveRPM() {
        return _driveEncoder.getVelocity();
    }

    public void setDriveRPM(double RPM) {
        _drivePID.setReference(RPM,ControlType.kVelocity);
    }

    public void resetSteeringEncoder() {
        _steeringEncoder.setPosition(0);
    }
}