// CR: In general, please document this whole file (class and methods).

package frc.robot.subsystems.drivetrain;

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

    private CTREAbsMagEncoder _absEncoder;

    private double _setpoint;
    // CR: The SwerveModuleConstants should not be taken from the class itself,
    // you should define a type outside of the config (common for example) - that is implemented by the config.
    public SwerveModule(SwerveModuleConstants constants) {
        // CR: I don't like the way you call a function to create a configured instance of Spark Max.
        // Please think about creating a class that extends the Spark Max and configure it the way you want.
        // CR: In addition, the using of SparkMax instance here, coupling you to the usage of Spark Max.
        _driveSparkMax = configSparkMax(constants.idDrive, _drivePID, _driveEncoder, constants.driveGains);
        _steeringSparkMax = configSparkMax(constants.idSteering, _steeringPID, _steeringEncoder, constants.steeringGains);
        
        _drivePID = _driveSparkMax.getPIDController();
        _driveEncoder = _driveSparkMax.getEncoder();

        _steeringPID = _steeringSparkMax.getPIDController();
        _steeringEncoder = _steeringSparkMax.getEncoder();

        setPIDGains(_drivePID, constants.driveGains);
        setPIDGains(_steeringPID, constants.steeringGains);
                
        _setpoint = 0;

        // CR: The usage of CTREAbsMagEncoder here, coupling you to the usage of CTRE Absolute Magentic Encoder.
        _absEncoder = new CTREAbsMagEncoder(constants.absEncoderPort, constants.steeringZeroValue);
        calibrateSteering();
    }

    public void calibrateSteering(){
        // CR: Hmmm... 4096.0... Nice number (please move to config).
        this._steeringEncoder.setPosition(_absEncoder.getPosition() / 4096.0 / Constants.Drivetrain.SwerveModuleConstants.steeringRatio);
    }

    private static CANSparkMax configSparkMax(int id, CANPIDController pidController, CANEncoder encoder, PIDFGains gains) {
        CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
        sparkMax.setInverted(false);
        
        return sparkMax;
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
    public void setDriveDrive(double percent) {
        this._driveSparkMax.set(percent);
    }

    public void stop() {
        setDriveDrive(0);
        setDriveSteering(0);
    }
    
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimizeAngle(desiredState, Rotation2d.fromDegrees(getAngle()));

        _setpoint = addDeltaFromZeroToEncoder(state.angle.getDegrees());
        _steeringPID.setReference(_setpoint, ControlType.kPosition);
        // _drivePID.setReference(driveVelocityToRPM(state.speedMetersPerSecond), ControlType.kVelocity);
    }

    // CR: How you optimize the angle here? Please rename this method.
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
        // CR: Put 60 in config.
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
        return _setpoint;
    }

    public void resetSteeringEncoder() {
        _steeringEncoder.setPosition(0);
    }

    public void close() {
        _absEncoder.close();
    }
}