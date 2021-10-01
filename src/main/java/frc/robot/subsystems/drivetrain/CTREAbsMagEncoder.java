package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/**
 * CTREMagEncoder
 */
public class CTREAbsMagEncoder {

    // private static final int CPR = 4096;

    private DutyCycle _encoder;
    private DigitalInput _source;

    public CTREAbsMagEncoder(int channel) {
        _source = new DigitalInput(channel);
        _encoder = new DutyCycle(_source);
    }

    public double getAngle() {
        return _encoder.getOutput() * 360;
    }


}