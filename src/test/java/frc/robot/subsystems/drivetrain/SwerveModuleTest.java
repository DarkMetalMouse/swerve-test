package frc.robot.subsystems.drivetrain;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.when;

import com.revrobotics.CANEncoder;

import org.junit.BeforeClass;
import org.junit.Test;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.utils.Utils;

public class SwerveModuleTest {

    static SwerveModule _swerve;
    static CANEncoder _steeringEncoder;

    @BeforeClass
    public static void init() {
        _swerve = new SwerveModule(Constants.Drivetrain.TRModule);
        _steeringEncoder = (CANEncoder) Utils.ReflectAndSpy(_swerve, "_steeringEncoder");
    }

    // @Test
    public void optimizeAngle(){
        SwerveModuleState ret = runOptimizeAngle(70,0);
        assertEquals(-70, ret.angle.getDegrees(), 0.01);
        assertEquals(1, ret.speedMetersPerSecond,0.01);

        ret = runOptimizeAngle(90,0);
        assertEquals(-90, ret.angle.getDegrees(), 0.01);
        assertEquals(1, ret.speedMetersPerSecond,0.01);

        ret = runOptimizeAngle(91,0);
        assertEquals(89, ret.angle.getDegrees(), 0.01);
        assertEquals(-1, ret.speedMetersPerSecond,0.01);

        ret = runOptimizeAngle(270,90);
        assertEquals(0, ret.angle.getDegrees(), 0.01);
        assertEquals(-1, ret.speedMetersPerSecond,0.01);
        
        ret = runOptimizeAngle(270,1);
        assertEquals(-89, ret.angle.getDegrees(), 0.01);
        assertEquals(-1, ret.speedMetersPerSecond,0.01);
    }

    private SwerveModuleState runOptimizeAngle(double current, double wanted) {
        return SwerveModule.optimizeAngle(new SwerveModuleState(1,Rotation2d.fromDegrees(wanted)), Rotation2d.fromDegrees(current));
    }

    @Test
    public void addDeltaToEncoder() {
        when(_steeringEncoder.getPosition()).thenReturn((double) 12.7);
        assertEquals(12.7 + (90.0 / 360.0) / Constants.Drivetrain.SwerveModuleConstants.steeringRatio, _swerve.addDeltaFromZeroToEncoder(90),0.01);
        
        when(_steeringEncoder.getPosition()).thenReturn((double) 18);
        assertEquals((18 + (10.0 / 360.0) / Constants.Drivetrain.SwerveModuleConstants.steeringRatio), _swerve.addDeltaFromZeroToEncoder(10),0.01);
        
        when(_steeringEncoder.getPosition()).thenReturn((double) 12.9);
        assertEquals((12.9 - (40.0 / 360.0) / Constants.Drivetrain.SwerveModuleConstants.steeringRatio), _swerve.addDeltaFromZeroToEncoder(-40),0.01);

        when(_steeringEncoder.getPosition()).thenReturn((double) -1);
        assertEquals((-1 + (40.0 / 360.0) / Constants.Drivetrain.SwerveModuleConstants.steeringRatio), _swerve.addDeltaFromZeroToEncoder(40),0.01);

        when(_steeringEncoder.getPosition()).thenReturn((double) -13);
        assertEquals((-13 + (40.0 / 360.0) / Constants.Drivetrain.SwerveModuleConstants.steeringRatio), _swerve.addDeltaFromZeroToEncoder(40),0.01);

        when(_steeringEncoder.getPosition()).thenReturn((double) -13);
        assertEquals((-13 - (40.0 / 360.0) / Constants.Drivetrain.SwerveModuleConstants.steeringRatio), _swerve.addDeltaFromZeroToEncoder(-40),0.01);

    }
}
