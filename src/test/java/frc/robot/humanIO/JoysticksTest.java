package frc.robot.humanIO;

import frc.robot.Robot;

public class JoysticksTest {

  static Joysticks _joysticks;
  static Joystick _joystick;

  @BeforeClass
  public static void init() {
    _joysticks = new Joysticks();
    _joystick = (Joystick) Utils.ReflectAndSpy(_joysticks, "joystick");
  }
}