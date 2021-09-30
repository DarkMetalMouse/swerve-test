package frc.robot.subsystems.drivetrain;

import org.junit.BeforeClass;
import org.junit.Test;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.utils.Utils;

/**
 * DrivetrainTest
 */
public class DrivetrainTest {

    static Drivetrain drivetrain;
    static DBugSwerveDriveKinematics _kinematics;

    @BeforeClass
    public static void init() {
        drivetrain = new Drivetrain();
        _kinematics = (DBugSwerveDriveKinematics) Utils.ReflectAndSpy(drivetrain, "_kinematics");
    }

    @Test
    public void toSwerveModuleStates() {
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(new ChassisSpeeds(1,0,1));
        for (SwerveModuleState swerveModuleState : moduleStates) {
            System.out.println(swerveModuleState);
        }
    }
}