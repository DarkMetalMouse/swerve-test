package frc.robot.humanIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Joysticks {

    private XboxController _controller;

    public Joysticks() {
        _controller = new XboxController(0);
    }

    private static double calculateDeadband(double value) {
        return Math.abs(value) > Constants.Joysticks.deadband ? Math.copySign(value * value, value) : 0;
    }

    private static double calculateDeadband(double value, double other) {
        return Math.abs(value) > Constants.Joysticks.deadband  || Math.abs(other) > Constants.Joysticks.deadband ? value : 0;
    }

    public double getSteerX() {
        return calculateDeadband(_controller.getRightX());
    }

    public double getDriveY() {
        return calculateDeadband(-_controller.getLeftY(),-_controller.getLeftX());
    }
    public double getDriveX() {
        return calculateDeadband(_controller.getLeftX(),-_controller.getLeftY());
    }

    public SwerveModuleState getDesiredState() {
        double y = _controller.getLeftX();
        double x = _controller.getLeftY();
        double speed = calculateDeadband(Math.hypot(x, y)) 
                                            * Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond 
                                            * Constants.Joysticks.speedScalar;
        Rotation2d angle = new Rotation2d(x, y);
        if(y <= 0) {
            angle = Rotation2d.fromDegrees(angle.getDegrees() + 360);
        }
        return new SwerveModuleState(speed,angle);
    }

    public boolean backButtonPressed() {
        return _controller.getBackButtonPressed();
    }

    public boolean yButtonPressed() {
        return _controller.getYButtonPressed();
    }

}
