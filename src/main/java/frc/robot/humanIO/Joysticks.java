package frc.robot.humanIO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class Joysticks {
    private Joystick _driveJoystick;
    private Joystick _steerJoystick;

    public Joysticks() {
        _driveJoystick = new Joystick(Constants.Joysticks.drivePort);
        _steerJoystick = new Joystick(Constants.Joysticks.steerPort);
    }

    private static double calculateDeadband(double value) {
        return Math.abs(value) > Constants.Joysticks.deadband ? value : 0;
    }

    public double getSteerX() {
        return calculateDeadband(-_steerJoystick.getX());
    }

    public double getDriveY() {
        return calculateDeadband(-_driveJoystick.getY());
    }
    public double getDriveX() {
        return calculateDeadband(-_driveJoystick.getX());
    }

    public SwerveModuleState getDesiredState() {
        double y = -_driveJoystick.getX();
        double x = -_driveJoystick.getY();
        double speed = calculateDeadband(Math.hypot(x, y)) 
                                            * Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond 
                                            * Constants.Joysticks.speedScalar;
        Rotation2d angle = new Rotation2d(x, y);
        if(y <= 0) {
            angle = Rotation2d.fromDegrees(angle.getDegrees() + 360);
        }
        return new SwerveModuleState(speed,angle);
    }

    public boolean isTriggerPressed() {
        return _driveJoystick.getTriggerPressed();
    }

}
