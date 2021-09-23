package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
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
        double radian = (_steeringEncoder.getPosition() - Math.floor(_steeringEncoder.getPosition())) 
                            * 2 * Math.PI 
                            * Constants.Drivetrain.SwerveModuleConstants.steeringDPR;
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(radian));
        // https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/lib/util/CTREModuleState.java
    }

}