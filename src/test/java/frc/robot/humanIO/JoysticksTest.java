package frc.robot.humanIO;

import org.junit.BeforeClass;

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

  
}