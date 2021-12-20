package frc.robot.humanIO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class Joysticks {

    private XboxController _controller;

    public Joysticks() {
        _controller = new XboxController(0);
    }

    private static double calculateDeadband(double value) {
        return Math.abs(value) > Constants.Joysticks.deadband ? value : 0;
    }

    private static double calculateDeadband(double value, double other) {
        return Math.abs(value) > Constants.Joysticks.deadband  || Math.abs(other) > Constants.Joysticks.deadband ? value : 0;
    }

    public double getSteerX() {
        return calculateDeadband(_controller.getX(Hand.kRight));
    }

    public double getDriveY() {
        return calculateDeadband(-_controller.getY(Hand.kLeft),-_controller.getX(Hand.kLeft));
    }
    public double getDriveX() {
        return calculateDeadband(_controller.getX(Hand.kLeft),-_controller.getY(Hand.kLeft));
    }

    public SwerveModuleState getDesiredState() {
        double y = _controller.getX(Hand.kLeft);
        double x = _controller.getY(Hand.kLeft);
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
