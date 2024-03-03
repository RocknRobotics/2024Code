package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Relay;

import frc.robot.Constants.hookConstants;

public class HookMaster {
    public Relay leftHook;
    public Relay rightHook;
    public long startMillis; //BEWARE: Will not work in approximately 292471154.678 years (not accounting for leap years)
    public boolean extended;
    public boolean timeFulfilled;

    public HookMaster() {
        leftHook = new Relay(hookConstants.leftHookID, Relay.Direction.kBoth);
        rightHook = new Relay(hookConstants.rightHookID, Relay.Direction.kBoth);

        startMillis = System.currentTimeMillis();
        extended = false;
        timeFulfilled = false;
        startMillis = Long.MIN_VALUE / 2;
    }

    //returns false while count < hookConstants.countsTillTop
    public void extend() {
        leftHook.set(Relay.Value.kForward); //Might need to switch forward and reverse
        rightHook.set(Relay.Value.kForward);
    }

    //returns false while count < hookConstants.countsTillTop
    public void retract() {
        leftHook.set(Relay.Value.kReverse);
        rightHook.set(Relay.Value.kReverse);
    }

    public void stop() {
        leftHook.set(Relay.Value.kOff);
        rightHook.set(Relay.Value.kOff);
    }

    public void update(boolean shareButtonPressed) {
        if(shareButtonPressed) {
            startMillis = System.currentTimeMillis();

            if(extended) {
                retract();
                extended = false;
            } else {
                extend();
                extended = true;
            }
        }

        long duration = System.currentTimeMillis() - startMillis;

        if(duration >= Constants.hookConstants.extendTimeMillis) {
            stop();
        } else {
            if(extended) {
                extend();
            } else {
                retract();
            }
        }
    }
}