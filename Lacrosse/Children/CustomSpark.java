package frc.robot.Children;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class CustomSpark extends CANSparkMax {
    public double maxChange;
    public double maxRPM;
    public RelativeEncoder relativeEncoder;

    public CustomSpark(int channel, double maxChange, double maxRPM, boolean inverted, RelativeEncoder relativeEncoder) {
        this(channel, MotorType.kBrushless, maxChange, maxRPM, inverted, relativeEncoder);
    }
    
    public CustomSpark(int channel, MotorType type, double maxChange, double maxRPM, boolean inverted, RelativeEncoder relativeEncoder) {
        super(channel, type);
        this.maxChange = maxChange;
        this.maxRPM = maxRPM;
        relativeEncoder = getEncoder();

        setInverted(inverted);
        relativeEncoder.setVelocityConversionFactor(1d);
        relativeEncoder.setPositionConversionFactor(1d);

        this.relativeEncoder = relativeEncoder;
    }

    public void limitedSet(double value) {
        set(Math.abs(value - get()) > maxChange 
            ? Math.signum(value - get()) * maxChange + get() 
            : value);
    }

    public void RPMSet(double targetRpm) {
        set(get() 
            + (Math.abs((targetRpm - relativeEncoder.getVelocity())) / (2 * maxRPM)) > maxChange 
            ? Math.signum(targetRpm - relativeEncoder.getVelocity()) * maxChange 
            : (targetRpm - relativeEncoder.getVelocity()) / (2 * maxRPM));
    }
}
