package frc.robot;

import edu.wpi.first.wpilibj.Relay;
import frc.robot.Constants.hookConstants;

public class HookMaster {
    public Relay leftHook;
    public Relay rightHook;

    public HookMaster() {
        leftHook = new Relay(hookConstants.leftHookID, Relay.Direction.kBoth);
        rightHook = new Relay(hookConstants.rightHookID, Relay.Direction.kBoth);
    }

    //returns false while count < hookConstants.countsTillTop
    public void extend() {
        leftHook.set(Relay.Value.kForward); //Might need to switch forward and reverse
        rightHook.set(Relay.Value.kReverse);
    }

    //returns false while count < hookConstants.countsTillTop
    public void retract() {
        leftHook.set(Relay.Value.kReverse);
        rightHook.set(Relay.Value.kForward);
    }

    public void stop() {
        leftHook.set(Relay.Value.kOff);
        rightHook.set(Relay.Value.kOff);
    }

    public void update(int drivePov) {
        if(drivePov == 0) {
            extend();
        } else if(drivePov == 180) {
            retract();
        } else {
            stop();
        }
    }
}