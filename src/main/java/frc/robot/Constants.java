package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.util.PIDFGains;

/**
 * Constants
 */
public class Constants {

    public static final class Drivetrain {
        public static class SwerveModuleConstants {
            public static double driveDPR = 1.0 / 8.14; 
            public static double steeringDPR = 1.0 / 12.8; 

            public final Translation2d position;
            public final int idDrive;
            public final PIDFGains driveGains;
            public final int idSteering;
            public final PIDFGains steeringGains;

            public SwerveModuleConstants(Translation2d position, int idDrive, int idSteering) {
                this(position, idDrive, idSteering, new PIDFGains(1, 0, 0, 0, 0.1, 0), new PIDFGains(1, 0, 0, 0, 0.1, 0));
            }
            public SwerveModuleConstants(Translation2d position, int idDrive, int idSteering, PIDFGains driveGains, PIDFGains steeringGains) {
                this.position = position;
                this.idDrive = idDrive;
                this.driveGains = driveGains;
                this.idSteering = idSteering;
                this.steeringGains = steeringGains;
            }
        }
        public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(new Translation2d(-0.215, 0.215), 1, 2);
        public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(new Translation2d(0.215, 0.215), 3, 4);
        public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(new Translation2d(-0.215, -0.215), 5, 6);
        public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(new Translation2d(0.215, -0.215), 7, 8);
    }
}