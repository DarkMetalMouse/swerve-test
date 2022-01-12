package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.PIDFGains;

/**
 * Constants
 */
public class Constants {

    public static final class Drivetrain {
        public static class SwerveModuleConstants {
            public static final double neoMaxSpeedRPM = 5600;
            public static final double driveRatio = 1.0 / 8.14; 
            public static final double steeringRatio = 1.0 / 12.8; 
            public static final double wheelRadiusMeters = 2 * 2.54 / 100; // 2 inches (in meters)
            public static final double wheelCircumferenceMeters = wheelRadiusMeters * 2 * Math.PI; 
            public static final double driveDPRMeters = wheelCircumferenceMeters * driveRatio;
            public static final double freeSpeedMetersPerSecond = neoMaxSpeedRPM / 60 * driveDPRMeters;

            
            public final Translation2d position;
            public final int idDrive;
            public final PIDFGains driveGains;
            public final int idSteering;
            public final PIDFGains steeringGains;

            public SwerveModuleConstants(Translation2d position, int idDrive, int idSteering) {
                this(position, idDrive, idSteering, new PIDFGains(0.0002, 0, 0.01, 1.0/6/902.0, 50, 0), new PIDFGains(0.35, 0, 0, 0, 1, 0));
            }
            public SwerveModuleConstants(Translation2d position, int idDrive, int idSteering, PIDFGains driveGains, PIDFGains steeringGains) {
                this.position = position;
                this.idDrive = idDrive;
                this.driveGains = driveGains;
                this.idSteering = idSteering;
                this.steeringGains = steeringGains;
            }
        }

        public static final double wheelBase = 0.43;
        public static final double trackWidth = wheelBase;

        public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(new Translation2d(-trackWidth/2, wheelBase/2), 3, 4);
        public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(new Translation2d(trackWidth/2, wheelBase/2), 1, 2);
        public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(new Translation2d(-trackWidth/2, -wheelBase/2), 5, 6);
        public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(new Translation2d(trackWidth/2, -wheelBase/2), 7, 8);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(TRModule.position,
                TLModule.position, BRModule.position, BLModule.position);

        public static final int pigeonTalonId = 9;
    }
    public static final class Joysticks {
        public static final double speedScalar = 0.7;
        public static final double deadband = 0.2;
        public static final int drivePort = 0;
        public static final int steerPort = 1;
    }

    public static final class Autonomous {
        // TODO: Calibrate
        public static final double kMaxSpeedMetersPerSecond = 3.6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 8.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3.6 / Math.hypot(Drivetrain.trackWidth /2, Drivetrain.wheelBase / 2);
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond * 2;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}