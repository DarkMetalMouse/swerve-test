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
    private int _offset;

    public CTREAbsMagEncoder(int channel, int zeroValue) {
        _source = new DigitalInput(channel);
        _encoder = new DutyCycle(_source);
        _offset = 4096 - zeroValue;
    }

    public int getFrequency() {
        return _encoder.getFrequency();
    }

    public int getPosition() {
        // double freq = _encoder.getFrequency();
        double output = _encoder.getOutput();

        int pos = (int) Math.round(output * 4098) - 1;
        pos = Math.max(0,pos);
        pos = Math.min(4095, pos);


        return (pos+ _offset) % 4096;
    }


}