package frc.robot.subsystems.drivetrain;

import org.junit.BeforeClass;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import frc.utils.Utils;

/**
 * DrivetrainTest
 */
public class DrivetrainTest {

    static Drivetrain drivetrain;
    static SwerveDriveKinematics _kinematics;

    @BeforeClass
    public static void init() {
        drivetrain = new Drivetrain();
        _kinematics = (SwerveDriveKinematics) Utils.ReflectAndSpy(drivetrain, "_kinematics");
    }

    // @Test
    // public void toSwerveModuleStates() {
    //     SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(new ChassisSpeeds(1,0,1));
    //     for (SwerveModuleState swerveModuleState : moduleStates) {
    //         System.out.println(swerveModuleState);
    //     }
    // }
}