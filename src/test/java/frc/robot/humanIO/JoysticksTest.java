package frc.robot.humanIO;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.when;

import org.junit.BeforeClass;
import org.junit.Test;

import edu.wpi.first.wpilibj.Joystick;
import frc.utils.Utils;

public class JoysticksTest {

  static Joysticks _joysticks;
  static Joystick _joystick;

  @BeforeClass
  public static void init() {
    _joysticks = new Joysticks();
    _joystick = (Joystick) Utils.ReflectAndSpy(_joysticks, "joystick");
  }

  @Test
  public void getDesiredState(){

    when(_joystick.getX()).thenReturn((double) 1.0);

    assertEquals(180, _joysticks.getDesiredState().angle.getDegrees(), 0.01);
  }

}