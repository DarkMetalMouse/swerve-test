/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.autonomous.SwerveTrajectoryFollower;
import frc.robot.humanIO.Joysticks;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  
  private static boolean _fieldRelative = true;
  private Trajectory _trajectory = getTrajectory();
  private SwerveTrajectoryFollower _follower;
  private boolean _followerFinished;
  public static Drivetrain drivetrain;
  public static Joysticks joysticks;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    joysticks = new Joysticks();
    drivetrain = new Drivetrain();
    drivetrain.resetYaw();
    _follower = getFollower(_trajectory);
    SmartDashboard.putNumber("RPM",0);
  }

  private static Trajectory getTrajectory() {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Autonomous.kMaxSpeedMetersPerSecond,
        Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Drivetrain.kinematics);

    // An example trajectory to follow. All units in meters.
    return TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)), config);

  }

  private static SwerveTrajectoryFollower getFollower(Trajectory trajectory) {

    var thetaController = new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0,
        Constants.Autonomous.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveTrajectoryFollower swerveTrajectoryFollower = new SwerveTrajectoryFollower(trajectory,
        drivetrain::getPose, Constants.Drivetrain.kinematics,
        new PIDController(Constants.Autonomous.kPXController, 0, 0),
        new PIDController(Constants.Autonomous.kPYController, 0, 0), thetaController, drivetrain::setDesiredStates);

    return swerveTrajectoryFollower;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drivetrain.printSetpoints();
    drivetrain.periodic();

    SmartDashboard.putNumber("RPM", SmartDashboard.getNumber("RPM", 0));

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(_trajectory.getInitialPose());
    _follower.initialize();
    _followerFinished = false;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (!_followerFinished) {
      if (!_follower.isFinished()) {
        _follower.execute();
      } else {
        // Stop execution and the robot.
        _followerFinished = true;
        drivetrain.drive(0, 0, 0, false);
      }
    }
  }

  @Override
  public void teleopInit() {
    drivetrain.drive(0.1, 0, 0, false);
    drivetrain.drive(0, 0, 0, false);
    drivetrain.setDriveRPM(0);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // drivetrain.setDriveRPM(SmartDashboard.getNumber("RPM", 0));


    // drivetrain.drive(joysticks.getDriveX() * SwerveModuleConstants.freeSpeedMetersPerSecond, joysticks.getDriveY()* SwerveModuleConstants.freeSpeedMetersPerSecond, joysticks.getSteerX(), _fieldRelative);
    if (joysticks.backButtonPressed()) {
      drivetrain.resetYaw();
    }
    if (joysticks.yButtonPressed()) {
      _fieldRelative = !_fieldRelative;
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
