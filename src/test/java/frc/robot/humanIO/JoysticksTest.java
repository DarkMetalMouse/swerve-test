package frc.robot.humanIO;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.when;

import org.junit.BeforeClass;
import org.junit.Test;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.utils.Utils;

public class JoysticksTest {

    static Joysticks _joysticks;
    static Joystick _joystick;

    @BeforeClass
    public static void init() {
        _joysticks = new Joysticks();
        _joystick = (Joystick) Utils.ReflectAndSpy(_joysticks, "_driveJoystick");
    }

    @Test
    public void getDesiredState() {

        when(_joystick.getY()).thenReturn((double) -1.0);
        when(_joystick.getX()).thenReturn((double) 0.0);
        assertEquals(360, _joysticks.getDesiredState().angle.getDegrees(), 0.01);

        when(_joystick.getY()).thenReturn((double) -1.0);
        when(_joystick.getX()).thenReturn((double) 1.0);
        assertEquals(360 - 45, _joysticks.getDesiredState().angle.getDegrees(), 0.01);

        when(_joystick.getY()).thenReturn((double) 0.0);
        when(_joystick.getX()).thenReturn((double) 1.0);
        assertEquals(360 - 90, _joysticks.getDesiredState().angle.getDegrees(), 0.01);

        when(_joystick.getY()).thenReturn((double) 1.0);
        when(_joystick.getX()).thenReturn((double) 1.0);
        assertEquals(360 - 135, _joysticks.getDesiredState().angle.getDegrees(), 0.01);

        when(_joystick.getY()).thenReturn((double) 1.0);
        when(_joystick.getX()).thenReturn((double) 0.0);
        assertEquals(360 - 180, _joysticks.getDesiredState().angle.getDegrees(), 0.01);

        when(_joystick.getY()).thenReturn((double) 1.0);
        when(_joystick.getX()).thenReturn((double) -1.0);
        assertEquals(360 - 225, _joysticks.getDesiredState().angle.getDegrees(), 0.01);

        when(_joystick.getY()).thenReturn((double) 0.0);
        when(_joystick.getX()).thenReturn((double) -1.0);
        assertEquals(360 - 270, _joysticks.getDesiredState().angle.getDegrees(), 0.01);

        when(_joystick.getY()).thenReturn((double) -1.0);
        when(_joystick.getX()).thenReturn((double) -1.0);
        assertEquals(360 - 315, _joysticks.getDesiredState().angle.getDegrees(), 0.01);

        when(_joystick.getY()).thenReturn((double) 0.0);
        when(_joystick.getX()).thenReturn((double) 0.0);
        assertEquals(0, _joysticks.getDesiredState().speedMetersPerSecond, 0.01);

        when(_joystick.getY()).thenReturn((double) 0.5);
        when(_joystick.getX()).thenReturn((double) 0.0);
        assertEquals(0.5 * Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond
                * Constants.Joysticks.speedScalar, _joysticks.getDesiredState().speedMetersPerSecond, 0.01);

        when(_joystick.getY()).thenReturn((double) 0.5);
        when(_joystick.getX()).thenReturn((double) -0.5);
        assertEquals(0.707 * Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond
                * Constants.Joysticks.speedScalar, _joysticks.getDesiredState().speedMetersPerSecond, 0.01);

    }

}